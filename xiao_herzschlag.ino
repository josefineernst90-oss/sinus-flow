/*
  Xiao nRF52840 Sense Plus — BLE Bewegungs-Schalter + Vibration (DRV2605, ERM)  "XiaoSwitch"
  -----------------------------------------------------------------------------------------
  Xiao -> App : Bewegungsstärke (Notify), die App entscheidet per Schwelle.
  App  -> Xiao: Vibrationsbefehl (Write) -> DRV2605 treibt den Motor mit der Stärke.

  Core "Seeeduino nRF52" (Bluefruit-basiert). NICHT ArduinoBLE.
  Bibliotheken: bluefruit.h (Core), Wire (Core), InternalFileSystem (Core), "Adafruit DRV2605 Library"

  Datenmodell (identisch zur App):
    Service        19b10000-e8f2-537e-4f6c-d104768a1214
    Motion-Char    19b10001-...  (Read | Notify)  uint8 = Bewegungsstärke (g*100, 0..255)
    Vibe-Char      19b10002-...  (Write)          uint8 = Vibrationsstärke (0..255)
    Herz-Char      19b10003-...  (Read | Write)   3 Byte = [BPM 30..180, Stärke 0..255, An 0/1]
    Atem-Char      19b10004-...  (Read | Write)   4 Byte = [Atemzüge/min 4..30, Stärke 0..255, An 0/1, Stil 0..4]
                                                  (Stil 0 = Pulsfolge, 1 = Welle, 2 = Brandung, 3 = Klopfen, 4 = Rauschen)
    Plan-Char      19b10005-...  (Read | Write)   8 Byte = [Automatik An 0/1, Intervall Min (u16 LE), Dauer Min (u16 LE),
                                                           Ankündigung Stärke 0..255, Klopf-Härte 1..31, Panikmodus An 0/1]
                                                  (5 Byte werden auch akzeptiert)
    Akku           Standard-Battery-Service 0x180F / 0x2A19 (Read | Notify), Prozent

  DRV2605 (I2C 0x5A): VIN->3V3, GND->GND, SDA->D4, SCL->D5, Motor an die Motor-Ausgänge.
  RTP-Format: signed (Standard). 0 = aus, 127 = Vollgas, negative Werte = aktives Bremsen.

  Rhythmus
    - Nur Herzschlag: läuft durchgehend.
    - Nur Atmung:     Einatmen (40 %) mit dem gewählten Muster, Ausatmen (60 %) still.
    - Beide:          Einatmen = Atem-Muster, Ausatmen = Herzschlag (wird zum Ende hin langsamer).

  Automatik (nur ohne Verbindung)
    Alle X Minuten (von Start zu Start) für Y Minuten. Ablauf je Auslösung:
      1. Ankündigung: zwei kräftige Schläge (eigene Stärke, damit man sie beim Gehen spürt).
      2. Warten auf Stillstand: erst nach 10 s Ruhe startet das Programm.
         Kommt keine Ruhe, wird nach 2 min erneut angekündigt; nach insgesamt 3 Versuchen fällt die Session aus.
      3. Programm läuft Y Minuten.
      4. Ende-Signal: ausklingender Herzschlag, dann zwei weiche Klopfer.
    Die erste Auslösung kommt X Minuten nach dem Trennen bzw. dem Einschalten.
    Mit Verbindung läuft immer direkt das, was die Seite zeigt.

  Panikmodus
    Zweimal auf das Gerät tippen startet ihn, zweimal tippen stoppt ihn (sonst nach 5 min Ende).
    Fester Rhythmus 4-7-8: 4 s einatmen, 7 s halten, 8 s ausatmen.
      Einatmen = dein Einatem-Muster, Halten = sehr leiser langsamer Herzschlag, Ausatmen = Herzschlag (langsamer werdend).
    Stärke und Muster kommen aus dem Programm, Stärke +20 %. Der Timer läuft unabhängig weiter;
    kollidiert er mit dem Panikmodus, hat der Panikmodus Vorrang und die Session wird um 2 min verschoben.

  Verbindung / Schlaf
    - Die Verbindung wird nie von der Biene getrennt.
    - Ohne Verbindung schläft nach 2 Minuten nur das Bluetooth (Werbung aus). Das Gerät selbst läuft weiter.
    - Doppelklopfen weckt das Bluetooth (und startet zugleich den Panikmodus, falls an).
    - Ohne funktionierende IMU kann nichts wecken, dann bleibt die Biene immer auffindbar.

  Konfiguration
    - Wird beim Trennen im Flash gespeichert (nur wenn etwas geändert wurde) und beim Start geladen.

  Keine LED-Ausgaben. (Die rote Lade-LED ist Hardware und lässt sich nicht abschalten.)
*/

#include <bluefruit.h>
#include "Wire.h"
#include "Adafruit_DRV2605.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

// ---- Debug ----
const bool DEBUG_STATUS = true;    // alle 2 s Statuszeile im Serial Monitor

// ---- Pins ----
#define IMU_INT1_PIN  11   // P0.11, intern mit LSM6DS3 INT1 verdrahtet

// ---- Zeitkonstanten ----
const float         NOISE_FLOOR  = 0.05;    // g — darunter gilt es als Ruhe (Bewegungs-Notify)
const unsigned long SAMPLE_MS    = 40;      // ~25 Hz Abtastung
const unsigned long VIBE_MS      = 350;     // Pulsdauer je Befehl
const unsigned long BAT_MS       = 30000;   // Akku alle 30 s messen
const unsigned long STATUS_MS    = 2000;    // Statuszeile
const unsigned long TAP_SPERRE_MS = 1500;   // nach einem Doppelklopfen kurz keine neue Erkennung

// ---- Bluetooth-Schlaf ----
const unsigned long BLE_SLEEP_MS = 120000;  // 2 min ohne Verbindung -> Bluetooth schläft

// ---- Automatik: Stillstand ----
const float         STILL_DELTA  = 0.05f;   // g, geglättete Bewegung darunter = Ruhe
const unsigned long STILL_MS     = 10000;   // so lange Ruhe, bis das Programm startet
const unsigned long WAIT_MS      = 120000;  // so lange wird pro Versuch auf Ruhe gewartet
const int           MAX_VERSUCHE = 3;       // Ankündigungen je Auslösung (1 + 2 Aufschübe)

// ---- Panikmodus ----
const unsigned long PANIK_EIN_MS    = 4000;
const unsigned long PANIK_HALTEN_MS = 7000;
const unsigned long PANIK_AUS_MS    = 8000;
const unsigned long PANIK_ZYKLUS_MS = PANIK_EIN_MS + PANIK_HALTEN_MS + PANIK_AUS_MS;
const unsigned long PANIK_MAX_MS    = 300000;   // spätestens nach 5 min Ende
const float         PANIK_BOOST     = 1.2f;     // Stärke +20 %
const float         PANIK_HALTEN_SKALA = 0.55f; // Herzschlag beim Halten leiser
const unsigned long PANIK_HALTEN_PERIODE_MS = 1400;   // ~43 bpm beim Halten

// ---- Motor ----
#define MOTOR_LRA 0   // 0 = ERM (Exzentermotor, dein Motor), 1 = LRA
const int RTP_MIN = 30;    // kleinste spürbare Stärke
const int RTP_MAX = 127;   // signed: 127 = Vollgas

// ---- Herzschlag ----
const unsigned long S1_MS     = 200;     // Dauer von "lub" (ERM braucht ~40 ms zum Anlaufen, daher lang mit Plateau)
const unsigned long S2_MS     = 150;     // Dauer von "dub"
const float         S2_FAKTOR = 0.70f;   // "dub" ist leiser als "lub"
const int           HERZ_MIN  = 50;      // kleinste Herz-Stärke (Stärke-Regler ganz unten), kräftiger als RTP_MIN

// Anstoß + Gegenschub, damit der ERM-Motor knackig statt matschig schlägt
const unsigned long KICK_MS      = 18;     // Anstoß-Dauer am Pulsbeginn
const unsigned long BRAKE_MS     = 14;     // Bremsdauer am Pulsende
const float         KICK_FAKTOR  = 1.5f;   // Anstoß = 1,5 x Stärke (max. 127)
const float         BRAKE_FAKTOR = 0.5f;   // Bremse = 50 % der Stärke, negativ

// ---- Atmung ----
const float ATEM_EINATMEN_ANTEIL = 0.40f;  // 40 % Einatmen, 60 % Ausatmen
const int   ATEM_MAX             = 110;    // höchste Atem-Stärke

// Stile 2-4 bleiben in einem niedrigen Band, damit der Motor nicht "hochdreht" (Windrad-Effekt)
const int ATEM_LOW_MAX = 95;       // höchste Stärke für Brandung / Klopfen / Rauschen

// Stil 0: Pulsfolge — kurze Taps, die beim Einatmen dichter und stärker werden
const float TAP_F0       = 2.5f;   // Taps pro Sekunde am Anfang des Einatmens
const float TAP_F1       = 7.0f;   // Taps pro Sekunde am Ende des Einatmens
const float TAP_MS       = 45.0f;  // Dauer eines Taps
const float TAP_KICK_MS  = 12.0f;  // Anstoß am Tap-Beginn
const float TAP_BRAKE_MS = 12.0f;  // Bremse nach dem Tap
const float TAP_AMP_MIN  = 0.40f;  // erster Tap = 40 % der Atem-Stärke, letzter = 100 %

// Herzschlag im Ausatmen
const unsigned long AUSATMEN_VERZOEGERUNG_MS = 60;   // kurze Pause zwischen Einatmen und erstem Schlag
const float         AUS_F0 = 0.92f;   // Schlagabstand am Beginn des Ausatmens (x Grundabstand)
const float         AUS_F1 = 1.08f;   // Schlagabstand am Ende (langsamer)

// ---- LSM6DS3-Register ----
#define REG_WHO_AM_I    0x0F
#define REG_TAP_SRC     0x1C   // Bit4 = Doppelklopf erkannt (Lesen löscht den Latch)
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_OUTX_L_XL   0x28
#define REG_TAP_CFG     0x58   // Bit7 = INTERRUPTS_ENABLE, Bit3..1 = Tap X/Y/Z, Bit0 = LIR (Latch)
#define REG_TAP_THS_6D  0x59   // Bit4..0 = Klopf-Schwelle
#define REG_INT_DUR2    0x5A   // DUR | QUIET | SHOCK
#define REG_WAKE_UP_THS 0x5B   // Bit7 = Doppelklopf statt Einzelklopf
#define REG_MD1_CFG     0x5E   // Bit3 = Doppelklopf auf INT1

// ---- Konfigurationsdatei ----
#define CFG_FILE      "/cfg.bin"
#define CFG_MAGIC     0xA9     // 0xA8 (13 Byte, ältere Version) wird noch gelesen
#define CFG_MAGIC_OLD 0xA8
#define CFG_LEN       16
#define CFG_LEN_OLD   13

// ---- Minimaler LSM6DS3-Treiber ----
class SimpleLSM6DS3 {
public:
  SimpleLSM6DS3(uint8_t addr) : _addr(addr), _wire(&Wire) {}

  int begin() {
#if defined(PIN_LSM6DS3TR_C_POWER)
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, LOW);
    delay(30);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
    delay(100);
#endif
    if (probe(&Wire)) { _wire = &Wire; }
#if defined(PIN_WIRE1_SDA)
    else if (probe(&Wire1)) { _wire = &Wire1; }
#endif
    else return -1;

    writeRegister(REG_CTRL3_C, 0x01);   // Software-Reset
    delay(20);
    writeRegister(REG_CTRL3_C, 0x44);   // BDU + Auto-Increment
    return 0;
  }

  bool writeRegister(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
  }

  bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom(_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _wire->read();
    return true;
  }

  float readFloatAccelX() { return readAxis(0); }
  float readFloatAccelY() { return readAxis(2); }
  float readFloatAccelZ() { return readAxis(4); }

private:
  uint8_t   _addr;
  TwoWire*  _wire;

  bool probe(TwoWire* w) {
    w->begin();
    w->beginTransmission(_addr);
    w->write((uint8_t)REG_WHO_AM_I);
    if (w->endTransmission(false) != 0) return false;
    if (w->requestFrom(_addr, (uint8_t)1) != 1) return false;
    uint8_t id = w->read();
    return (id == 0x69 || id == 0x6A || id == 0x6B);
  }

  float readAxis(uint8_t offset) {
    uint8_t b[6];
    if (!readRegisters(REG_OUTX_L_XL, b, 6)) return 0;
    int16_t raw = (int16_t)(b[offset] | (b[offset + 1] << 8));
    return raw * 0.061f / 1000.0f;
  }
};

// ---- Hardware-Objekte ----
SimpleLSM6DS3    imu(0x6A);
Adafruit_DRV2605 drv;
bool             haveDrv = false;
bool             imuOk   = false;

// ---- UUIDs (Little-Endian) ----
const uint8_t MOTION_SERVICE_UUID[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19};
const uint8_t MOTION_CHAR_UUID[16]    = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x01,0x00,0xb1,0x19};
const uint8_t VIBE_CHAR_UUID[16]      = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x02,0x00,0xb1,0x19};
const uint8_t HEART_CHAR_UUID[16]     = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x03,0x00,0xb1,0x19};
const uint8_t BREATH_CHAR_UUID[16]    = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x04,0x00,0xb1,0x19};
const uint8_t PLAN_CHAR_UUID[16]      = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x05,0x00,0xb1,0x19};

BLEService        motionService(MOTION_SERVICE_UUID);
BLECharacteristic motionChar(MOTION_CHAR_UUID);
BLECharacteristic vibeChar(VIBE_CHAR_UUID);
BLECharacteristic heartChar(HEART_CHAR_UUID);
BLECharacteristic breathChar(BREATH_CHAR_UUID);
BLECharacteristic planChar(PLAN_CHAR_UUID);
BLEBas            blebas;

// ---- Akku ----
#ifndef PIN_VBAT
#define PIN_VBAT    (32u)
#endif
#ifndef VBAT_ENABLE
#define VBAT_ENABLE (14u)
#endif
#define VBAT_CAL  (1.000f)
unsigned long batNextAt = 0;

// ---- Konfiguration (wird im Flash gespeichert) ----
volatile uint8_t  heartBpm       = 60;
volatile uint8_t  heartStrength  = 160;
volatile bool     heartOn        = false;
volatile uint8_t  breathRate     = 12;     // Atemzüge pro Minute
volatile uint8_t  breathStrength = 120;
volatile bool     breathOn       = false;
volatile uint8_t  breathStyle    = 0;      // 0 Pulsfolge, 1 Welle, 2 Brandung, 3 Klopfen, 4 Rauschen
volatile bool     schedOn        = false;  // Automatik
volatile uint16_t schedIntervalMin = 30;   // alle X Minuten
volatile uint16_t schedDurationMin = 3;    // für Y Minuten
volatile uint8_t  announceStrength = 200;  // Stärke von Ankündigung und Ende-Signal (0 = stumm)
volatile uint8_t  tapThs         = 6;      // Klopf-Härte 1 (empfindlich) .. 31 (hart), je 62,5 mg
volatile bool     panicEnabled   = true;   // Doppelklopfen startet den Panikmodus

volatile bool cfgDirty    = false;   // seit dem letzten Speichern geändert
volatile bool savePending = false;   // beim Trennen gesetzt, loop() speichert
volatile bool tapCfgDirty = false;   // Klopf-Härte muss in den Sensor geschrieben werden

// ---- Zustand ----
float pax = 0, pay = 0, paz = 0;
bool  primed    = false;
bool  wasMoving = false;
float motionEma = 1.0f;              // geglättete Bewegung (g pro Abtastung)
unsigned long stillSince = 0;        // seit wann Ruhe herrscht (0 = nicht ruhig)
unsigned long lastStatusAt = 0;
unsigned long vibeOffAt    = 0;
int  currentVibe = 0;
int  targetVibe  = 0;
unsigned long lastVibeTick = 0;
unsigned long lastSampleAt = 0;
unsigned long lastTapAt    = 0;
uint8_t lastMotionVal = 0;

// Session: läuft das Programm gerade? Mit Verbindung immer, ohne Verbindung nach Automatik.
volatile bool sessionActive = true;

enum SchedState { SCH_OFF, SCH_IDLE, SCH_WAIT, SCH_RUN };
SchedState    schedState      = SCH_OFF;
unsigned long schedNextStart  = 0;
unsigned long schedSessionEnd = 0;
unsigned long schedWaitStart  = 0;
int           schedAttempts   = 0;
bool          schedDeferred   = false;   // Session wurde vom Panikmodus verdrängt

// Panikmodus
bool          panicActive  = false;
unsigned long panicStartAt = 0;

// Bluetooth-Schlaf
bool          bleSleeping  = false;
unsigned long bleIdleSince = 0;

// ==========================================================================
// Konfiguration speichern / laden
// ==========================================================================
void saveConfig() {
  uint8_t b[CFG_LEN];
  b[0]  = CFG_MAGIC;
  b[1]  = heartBpm;
  b[2]  = heartStrength;
  b[3]  = heartOn ? 1 : 0;
  b[4]  = breathRate;
  b[5]  = breathStrength;
  b[6]  = breathOn ? 1 : 0;
  b[7]  = breathStyle;
  b[8]  = schedOn ? 1 : 0;
  b[9]  = schedIntervalMin & 0xFF;
  b[10] = (schedIntervalMin >> 8) & 0xFF;
  b[11] = schedDurationMin & 0xFF;
  b[12] = (schedDurationMin >> 8) & 0xFF;
  b[13] = announceStrength;
  b[14] = tapThs;
  b[15] = panicEnabled ? 1 : 0;

  InternalFS.remove(CFG_FILE);
  File f(InternalFS);
  if (f.open(CFG_FILE, FILE_O_WRITE)) {
    f.write(b, CFG_LEN);
    f.flush();
    f.close();
    Serial.println("Konfiguration gespeichert");
  } else {
    Serial.println("Konfiguration: Speichern fehlgeschlagen");
  }
}

void loadConfig() {
  uint8_t b[CFG_LEN];
  File f(InternalFS);
  if (!f.open(CFG_FILE, FILE_O_READ)) { Serial.println("Keine gespeicherte Konfiguration"); return; }
  int n = f.read(b, CFG_LEN);
  f.close();
  bool neu = (n == CFG_LEN     && b[0] == CFG_MAGIC);
  bool alt = (n == CFG_LEN_OLD && b[0] == CFG_MAGIC_OLD);
  if (!neu && !alt) { Serial.println("Konfiguration ungueltig - Standardwerte"); return; }

  int bpm = b[1];     if (bpm < 30) bpm = 30;   if (bpm > 180) bpm = 180;
  int rate = b[4];    if (rate < 4) rate = 4;   if (rate > 30) rate = 30;
  int iv = b[9] | (b[10] << 8);   if (iv < 1) iv = 1;   if (iv > 1440) iv = 1440;
  int du = b[11] | (b[12] << 8);  if (du < 1) du = 1;   if (du > iv) du = iv;

  heartBpm = (uint8_t)bpm;
  heartStrength = b[2];
  heartOn = b[3] & 1;
  breathRate = (uint8_t)rate;
  breathStrength = b[5];
  breathOn = b[6] & 1;
  breathStyle = (b[7] > 4) ? 0 : b[7];
  schedOn = b[8] & 1;
  schedIntervalMin = (uint16_t)iv;
  schedDurationMin = (uint16_t)du;
  if (neu) {
    announceStrength = b[13];
    int th = b[14]; if (th < 1) th = 1; if (th > 31) th = 31;
    tapThs = (uint8_t)th;
    panicEnabled = b[15] & 1;
  }
  Serial.println("Konfiguration geladen");
}

// ==========================================================================
// IMU-Konfiguration
// ==========================================================================
void applyTapThreshold() {
  imu.writeRegister(REG_TAP_THS_6D, tapThs & 0x1F);
}

void configureIMU() {
  imu.writeRegister(REG_CTRL2_G,     0x00);            // Gyro aus
  imu.writeRegister(REG_CTRL1_XL,    0x60);            // Accel 416 Hz, ±2g (nötig für sichere Klopf-Erkennung)
  imu.writeRegister(REG_TAP_CFG,     0x8F);            // Interrupts an, Tap X/Y/Z, Latch
  applyTapThreshold();
  imu.writeRegister(REG_INT_DUR2,    0x7F);            // Doppelklopf-Zeitfenster
  imu.writeRegister(REG_WAKE_UP_THS, 0x80);            // Doppelklopf aktiv
  imu.writeRegister(REG_MD1_CFG,     0x08);            // Doppelklopf auf INT1
  uint8_t s = 0;
  imu.readRegisters(REG_TAP_SRC, &s, 1);               // alten Latch löschen
}

// ==========================================================================
// Rhythmus-Engine
// ==========================================================================
unsigned long beatStartAt = 0;
unsigned long beatPeriod  = 1000;
unsigned long beatGap     = 320;
bool          beatActive  = false;     // nur im Ausatem-Modus
long          beatCycleIdx = -1;
unsigned long beatExStart = 0, beatExEnd = 0;
unsigned long breathStartAt = 0;

int  lastRhythmVal = -1000;   // -1000 = noch nichts gesendet
bool heartWasRun   = false;
bool breathWasRun  = false;

static float pulseEnv(float t, float dur) {
  if (t < 0 || t >= dur) return 0;
  float x = t / dur;
  if (x < 0.15f) return x / 0.15f;
  float d = (1.0f - x) / 0.85f;
  return d * d;
}

// Hüllkurve mit Plateau: schneller Anstieg, kurz halten, dann abklingen
static float heartEnv(float t, float dur) {
  if (t < 0 || t >= dur) return 0;
  float x = t / dur;
  if (x < 0.12f) return x / 0.12f;
  if (x < 0.50f) return 1.0f;
  float d = (1.0f - x) / 0.50f;
  return d * d;
}

// Abstand lub -> dub
static unsigned long gapFor(unsigned long period) {
  unsigned long gap = (unsigned long)(period * 0.36f);
  if (gap < 230) gap = 230;
  if (gap > 360) gap = 360;
  unsigned long maxGap = (unsigned long)(period * 0.95f);
  if (maxGap > S2_MS && gap + S2_MS > maxGap) gap = maxGap - S2_MS;   // bei sehr hohem Puls enger
  return gap;
}

// Maximale Herz-Stärke (RTP) aus dem Stärke-Regler, optional skaliert
static int heartMaxV(float scale) {
  if (heartStrength == 0) return 0;
  int m = (int)(map(heartStrength, 1, 255, HERZ_MIN, RTP_MAX) * scale + 0.5f);
  if (m < RTP_MIN) m = RTP_MIN;
  if (m > RTP_MAX) m = RTP_MAX;
  return m;
}

static void heartStartBeat(unsigned long startAt, float factor) {
  beatStartAt = startAt;
  beatPeriod  = (unsigned long)((60000UL / heartBpm) * factor);
  beatGap     = gapFor(beatPeriod);
}

// Form eines Schlags (lub-dub) t ms nach Schlagbeginn; Wert signed, kann negativ sein = Bremse
static int beatShapeT(float t, unsigned long gap, int maxv) {
  float tS2 = t - (float)gap;
  float v = heartEnv(t, (float)S1_MS) + S2_FAKTOR * heartEnv(tS2, (float)S2_MS);
  int val = (int)(v * maxv + 0.5f);

  if (maxv > 0) {
    int kick1 = (int)(maxv * KICK_FAKTOR);
    int kick2 = (int)(maxv * S2_FAKTOR * KICK_FAKTOR);
    int brake = -(int)(maxv * BRAKE_FAKTOR);
    if (kick1 > RTP_MAX) kick1 = RTP_MAX;
    if (kick2 > RTP_MAX) kick2 = RTP_MAX;

    if      (t   >= 0        && t   < KICK_MS)            val = max(val, kick1);   // Anstoß lub
    else if (tS2 >= 0        && tS2 < KICK_MS)            val = max(val, kick2);   // Anstoß dub
    else if (t   >= S1_MS    && t   < S1_MS + BRAKE_MS)   val = brake;             // Bremse nach lub
    else if (tS2 >= S2_MS    && tS2 < S2_MS + BRAKE_MS)   val = brake;             // Bremse nach dub
  }
  return val;
}

static int beatShape(unsigned long now) {
  return beatShapeT((float)(now - beatStartAt), beatGap, heartMaxV(1.0f));
}

// Herzschlag durchgehend (wenn keine Atmung läuft)
static int heartFree(unsigned long now) {
  if (!heartWasRun) {
    heartStartBeat(now, 1.0f);
    heartWasRun = true;
  }
  if (now - beatStartAt >= beatPeriod) {
    unsigned long next = beatStartAt + beatPeriod;
    heartStartBeat(next, 1.0f);
    if (now - next >= beatPeriod) heartStartBeat(now, 1.0f);
  }
  return beatShape(now);
}

// Herzschlag nur im Ausatmen, zum Ende hin langsamer
static int heartExhale(unsigned long now, unsigned long cycleStart, long cycleIdx,
                       unsigned long inhaleMs, unsigned long period) {
  unsigned long exStart = cycleStart + inhaleMs + AUSATMEN_VERZOEGERUNG_MS;
  unsigned long exEnd   = cycleStart + period;

  if (cycleIdx != beatCycleIdx) {
    beatActive = false;                       // letzter Schlag war im alten Zyklus längst zu Ende
    if ((long)(now - exStart) >= 0) {
      beatCycleIdx = cycleIdx;
      beatExStart = exStart;
      beatExEnd   = exEnd;
      heartStartBeat(exStart, AUS_F0);
      beatActive = (exStart + beatGap + S2_MS <= exEnd);
    }
  }
  if (!beatActive) return 0;

  if (now - beatStartAt >= beatPeriod) {
    unsigned long next = beatStartAt + beatPeriod;
    float prog = (float)(long)(next - beatExStart) / (float)(beatExEnd - beatExStart);
    if (prog < 0) prog = 0;
    if (prog > 1) prog = 1;
    float factor = AUS_F0 + (AUS_F1 - AUS_F0) * prog;
    unsigned long per = (unsigned long)((60000UL / heartBpm) * factor);
    if (next + gapFor(per) + S2_MS <= beatExEnd) heartStartBeat(next, factor);
    else beatActive = false;
  }
  if (!beatActive) return 0;
  return beatShape(now);
}

// Herzschläge in einem Zeitfenster (tw = ms seit Fensterbeginn), Abstand wandert von f0 auf f1.
// Zustandslos, daher auch für den Panikmodus geeignet.
static int windowBeatValue(float tw, float windowLen, float basePeriod, float f0, float f1, int maxv) {
  float start = 0;
  while (start < windowLen) {
    float per = basePeriod * (f0 + (f1 - f0) * start / windowLen);
    unsigned long gap = gapFor((unsigned long)per);
    if (start + gap + S2_MS > windowLen) return 0;      // passt nicht mehr ins Fenster
    if (tw < start + per) return beatShapeT(tw - start, gap, maxv);
    start += per;
  }
  return 0;
}

// Atmung: nur das Einatmen trägt ein Muster, das Ausatmen bleibt still.
// boost > 1 macht es kräftiger (Panikmodus).
static int breathInhale(unsigned long posMs, unsigned long inhaleMs, float boost = 1.0f) {
  if (breathStrength == 0) return 0;
  float x = (float)posMs / (float)inhaleMs;       // 0..1 im Einatmen

  // ---- Stile 2-4: niedriges Band, kein Hochdrehen ----
  if (breathStyle >= 2) {
    int top = (int)(map(breathStrength, 1, 255, RTP_MIN, ATEM_LOW_MAX) * boost + 0.5f);
    if (top > RTP_MAX) top = RTP_MAX;

    if (breathStyle == 2) {
      // Brandung: 2-3 lange, weiche Wellen, jede etwas voller als die vorige
      int n = (inhaleMs >= 3000) ? 3 : 2;
      float L = (float)inhaleMs / n;
      int k = (int)((float)posMs / L);
      if (k >= n) k = n - 1;
      float u = ((float)posMs - k * L) / L;                         // 0..1 in der Welle
      float env = 0.5f - 0.5f * cosf(2.0f * PI * u);                // 0 -> 1 -> 0
      if (env < 0.03f) return 0;
      float amp = top * (0.85f + 0.15f * (n > 1 ? (float)k / (n - 1) : 1.0f));
      if (amp < RTP_MIN) amp = RTP_MIN;
      return (int)(RTP_MIN + (amp - RTP_MIN) * env + 0.5f);
    }

    if (breathStyle == 3) {
      // Klopfen: wenige langsame, gleich starke, weiche Schläge wie ein Takt auf der Brust
      int n = (int)(inhaleMs / 700 + 0.5f);
      if (n < 2) n = 2;
      if (n > 4) n = 4;
      float spacing = (float)inhaleMs / n;
      float since = fmodf((float)posMs, spacing);                   // ms seit dem letzten Klopfen
      const float KNOCK_MS = 90.0f;
      float amp = (float)top * 0.85f;
      if (amp < RTP_MIN) amp = RTP_MIN;
      if (since < KNOCK_MS) {
        int val = (int)(amp * heartEnv(since, KNOCK_MS) + 0.5f);
        if (since < 14.0f) { int kick = (int)(amp * 1.25f); if (kick > RTP_MAX) kick = RTP_MAX; val = max(val, kick); }
        return val;
      }
      if (since < KNOCK_MS + 10.0f) return -(int)(amp * 0.3f);
      return 0;
    }

    // Rauschen: unregelmäßig moduliertes Grundrumpeln, das langsam lauter wird (wie Luftstrom)
    float tS = (float)posMs / 1000.0f;
    float nz = (sinf(2.0f * PI * 7.3f * tS) + sinf(2.0f * PI * 11.7f * tS + 1.3f) + sinf(2.0f * PI * 17.9f * tS + 2.1f)) / 3.0f;  // -1..1
    nz = 0.5f + 0.5f * nz;                                          // 0..1
    float env = 0.55f + 0.45f * (0.5f - 0.5f * cosf(PI * x));       // 0.55 -> 1
    int v = (int)(top * env * (0.50f + 0.50f * nz) + 0.5f);
    if (v < RTP_MIN) v = RTP_MIN;
    return v;
  }

  int maxb = (int)(map(breathStrength, 1, 255, RTP_MIN, ATEM_MAX) * boost + 0.5f);
  if (maxb > RTP_MAX) maxb = RTP_MAX;

  if (breathStyle == 1) {
    // Welle: weich ansteigend, am Ende des Einatmens abgeschnitten
    float b = 0.5f - 0.5f * cosf(PI * x);
    return (int)(b * maxb + 0.5f);
  }

  // Pulsfolge: Taps, deren Abstand schrumpft und deren Stärke steigt
  float tS = (float)posMs / 1000.0f;
  float D  = (float)inhaleMs / 1000.0f;
  float phase = TAP_F0 * tS + 0.5f * (TAP_F1 - TAP_F0) * tS * tS / D;
  float fr    = phase - floorf(phase);
  float f     = TAP_F0 + (TAP_F1 - TAP_F0) * (tS / D);
  float since = fr / f * 1000.0f;                 // ms seit Beginn des letzten Taps

  float amp = maxb * (TAP_AMP_MIN + (1.0f - TAP_AMP_MIN) * x);
  if (since < TAP_MS) {
    int val = (int)(amp * pulseEnv(since, TAP_MS) + 0.5f);
    if (since < TAP_KICK_MS) {
      int kick = (int)(amp * 1.4f);
      if (kick > RTP_MAX) kick = RTP_MAX;
      val = max(val, kick);
    }
    return val;
  }
  if (since < TAP_MS + TAP_BRAKE_MS) return -(int)(amp * 0.4f);
  return 0;
}

// Panikmodus 4-7-8: el = ms seit Start
static int panicValue(unsigned long el) {
  unsigned long pos = el % PANIK_ZYKLUS_MS;
  if (pos < PANIK_EIN_MS) return breathInhale(pos, PANIK_EIN_MS, PANIK_BOOST);

  if (pos < PANIK_EIN_MS + PANIK_HALTEN_MS) {
    float tw = (float)(pos - PANIK_EIN_MS);
    return windowBeatValue(tw, (float)PANIK_HALTEN_MS, (float)PANIK_HALTEN_PERIODE_MS, 1.0f, 1.0f,
                           heartMaxV(PANIK_BOOST * PANIK_HALTEN_SKALA));
  }
  float tw = (float)(pos - PANIK_EIN_MS - PANIK_HALTEN_MS);
  return windowBeatValue(tw, (float)PANIK_AUS_MS, 60000.0f / heartBpm, AUS_F0, AUS_F1, heartMaxV(PANIK_BOOST));
}

// ==========================================================================
// Signale (blockierend, kurz): Ankündigung, Ende, Bestätigung
// ==========================================================================
static int announceLevel() {
  if (announceStrength == 0) return 0;
  return (int)map(announceStrength, 1, 255, RTP_MIN, RTP_MAX);
}

static void motorSet(int v) {
  if (v >  RTP_MAX) v =  RTP_MAX;
  if (v < -RTP_MAX) v = -RTP_MAX;
  drv.setRealtimeValue((uint8_t)(int8_t)v);
}

// Ein kurzer Schlag mit Anstoß und Bremse
static void buzz(int level, unsigned long ms) {
  if (!haveDrv || level <= 0) return;
  int kick = (int)(level * 1.4f); if (kick > RTP_MAX) kick = RTP_MAX;
  motorSet(kick);  delay(20);
  motorSet(level); delay(ms > 20 ? ms - 20 : 0);
  motorSet(-(level * 2 / 5)); delay(BRAKE_MS);
  motorSet(0);
}

// Ein Herzschlag (lub-dub) blockierend, anschließend Pause bis period vorbei ist
static void beatBlocking(int maxv, unsigned long period) {
  if (!haveDrv) return;
  unsigned long gap = gapFor(period);
  unsigned long t0 = millis();
  unsigned long len = gap + S2_MS + BRAKE_MS + 10;
  while (millis() - t0 < len) {
    motorSet(beatShapeT((float)(millis() - t0), gap, maxv));
    delay(5);
  }
  motorSet(0);
  unsigned long used = millis() - t0;
  if (period > used) delay(period - used);
}

static void signalDone() { lastRhythmVal = -1000; }

// Ankündigung: zwei kräftige Schläge
void announceSignal() {
  int lvl = announceLevel();
  buzz(lvl, 150);
  delay(130);
  buzz(lvl, 150);
  signalDone();
}

// Zwei weiche Klopfer
static void softKnocks() {
  int lvl = (int)(announceLevel() * 0.8f);
  if (lvl > 0 && lvl < RTP_MIN) lvl = RTP_MIN;
  buzz(lvl, 90);
  delay(260);
  buzz(lvl, 90);
}

// Ende: ausklingender Herzschlag, dann zwei weiche Klopfer
void endSignal() {
  int lvl = announceLevel();
  if (lvl <= 0) { signalDone(); return; }
  beatBlocking(lvl,                    1100);
  beatBlocking((int)(lvl * 0.70f),     1400);
  beatBlocking((int)(lvl * 0.45f),     1600);
  delay(300);
  softKnocks();
  signalDone();
}

// ==========================================================================
// Panikmodus
// ==========================================================================
void startPanic(unsigned long now) {
  if (!haveDrv) return;
  panicActive = true;
  panicStartAt = now;
  heartWasRun = false;
  breathWasRun = false;
  Serial.println("Panikmodus an");
}

void stopPanic(unsigned long now, bool full) {
  if (!panicActive) return;
  panicActive = false;
  if (haveDrv) motorSet(0);
  lastRhythmVal = -1000;
  currentVibe = 0; targetVibe = 0;
  if (full) endSignal(); else { softKnocks(); signalDone(); }
  if (schedDeferred) {                       // verdrängte Session um 2 min verschieben
    schedNextStart = millis() + 120000UL;
    schedDeferred = false;
  }
  Serial.println("Panikmodus aus");
  (void)now;
}

// ==========================================================================
// Rhythmus ausgeben
// ==========================================================================
void rhythmUpdate(unsigned long now) {
  if (!haveDrv) return;

  // Panikmodus hat Vorrang vor allem anderen
  if (panicActive) {
    if (now - panicStartAt >= PANIK_MAX_MS) { stopPanic(now, true); return; }
    int val = panicValue(now - panicStartAt);
    if (val >  RTP_MAX) val =  RTP_MAX;
    if (val < -RTP_MAX) val = -RTP_MAX;
    if (val != lastRhythmVal) { motorSet(val); lastRhythmVal = val; }
    return;
  }

  bool runH = heartOn  && sessionActive;
  bool runB = breathOn && sessionActive;

  if (!runH && !runB) {
    heartWasRun = false;
    breathWasRun = false;
    if (lastRhythmVal != -1000) {
      drv.setRealtimeValue(0);
      currentVibe = 0; targetVibe = 0;
      lastRhythmVal = -1000;
    }
    return;
  }

  // Atem-Takt
  unsigned long period = 60000UL / breathRate;
  unsigned long inhaleMs = (unsigned long)(period * ATEM_EINATMEN_ANTEIL);
  unsigned long cycleStart = 0, pos = 0;
  long cycleIdx = 0;

  if (runB && !breathWasRun) {
    breathStartAt = now;
    breathWasRun = true;
    beatActive = false;
    beatCycleIdx = -1;
  }
  if (!runB) breathWasRun = false;

  if (runB) {
    unsigned long el = now - breathStartAt;
    cycleIdx   = (long)(el / period);
    pos        = el % period;
    cycleStart = now - pos;
  }

  // Herz wurde gerade (wieder) eingeschaltet: im Ausatem-Modus beginnt es im nächsten Ausatmen
  // (steckt der Zyklus noch im Einatmen, ist das schon das aktuelle)
  if (runH && !heartWasRun) {
    heartWasRun = true;
    beatActive = false;
    beatCycleIdx = (runB && pos >= inhaleMs) ? cycleIdx : -1;
    if (!runB) heartStartBeat(now, 1.0f);
  }
  if (!runH) heartWasRun = false;

  int val = 0;
  if (runH) {
    if (runB) val += heartExhale(now, cycleStart, cycleIdx, inhaleMs, period);
    else      val += heartFree(now);
  }
  if (runB && pos < inhaleMs) val += breathInhale(pos, inhaleMs);

  if (val >  RTP_MAX) val =  RTP_MAX;
  if (val < -RTP_MAX) val = -RTP_MAX;
  if (val != lastRhythmVal) { drv.setRealtimeValue((uint8_t)(int8_t)val); lastRhythmVal = val; }
}

// ==========================================================================
// Automatik: Ankündigung -> auf Stillstand warten -> Session -> Ende-Signal
// (nur ohne Verbindung)
// ==========================================================================
void scheduleUpdate(unsigned long now) {
  if (Bluefruit.connected()) {          // mit Verbindung läuft immer direkt, was die Seite zeigt
    sessionActive = true;
    schedState = SCH_OFF;
    schedDeferred = false;
    return;
  }
  if (!schedOn || (!heartOn && !breathOn)) {
    sessionActive = false;
    schedState = SCH_OFF;
    schedDeferred = false;
    return;
  }

  unsigned long ivMs = (unsigned long)schedIntervalMin * 60000UL;
  unsigned long duMs = (unsigned long)schedDurationMin * 60000UL;

  // Panikmodus verdrängt die Session; nach dem Panikmodus kommt sie 2 min später
  if (panicActive) {
    sessionActive = false;
    if (schedState == SCH_WAIT || schedState == SCH_RUN) { schedState = SCH_IDLE; schedDeferred = true; }
    else if (schedState == SCH_IDLE && (long)(now - schedNextStart) >= 0) schedDeferred = true;
    else if (schedState == SCH_OFF) { schedState = SCH_IDLE; schedNextStart = now + ivMs; }
    return;
  }

  switch (schedState) {
    case SCH_OFF:                        // erste Auslösung ein Intervall nach dem Trennen / Einschalten
      schedState = SCH_IDLE;
      schedNextStart = now + ivMs;
      sessionActive = false;
      break;

    case SCH_IDLE:
      sessionActive = false;
      if ((long)(now - schedNextStart) >= 0) {
        announceSignal();
        now = millis();
        schedAttempts = 1;
        schedWaitStart = now;
        stillSince = 0; motionEma = 1.0f; primed = false;
        schedNextStart = now + ivMs;     // Intervall von Start zu Start
        schedState = SCH_WAIT;
        Serial.println("Automatik: angekuendigt, warte auf Stillstand");
      }
      break;

    case SCH_WAIT:
      sessionActive = false;
      if (stillSince && now - stillSince >= STILL_MS) {
        sessionActive = true;
        schedSessionEnd = now + duMs;
        schedState = SCH_RUN;
        Serial.println("Automatik: Session laeuft");
      } else if (now - schedWaitStart >= WAIT_MS) {
        if (schedAttempts < MAX_VERSUCHE) {
          announceSignal();
          now = millis();
          schedAttempts++;
          schedWaitStart = now;
          Serial.println("Automatik: erneut angekuendigt");
        } else {
          schedState = SCH_IDLE;         // aufgegeben, nächste Auslösung kommt nach dem Intervall
          if ((long)(now - schedNextStart) >= 0) schedNextStart = now + 30000UL;
          Serial.println("Automatik: Session ausgefallen");
        }
      }
      break;

    case SCH_RUN:
      if ((long)(now - schedSessionEnd) >= 0) {
        sessionActive = false;
        schedState = SCH_IDLE;
        if (haveDrv) { drv.setRealtimeValue(0); lastRhythmVal = -1000; }
        endSignal();
        now = millis();
        if ((long)(now - schedNextStart) >= 0) schedNextStart = now + 30000UL;
        Serial.println("Automatik: Session beendet");
      }
      break;
  }
}

// ==========================================================================
// Bluetooth-Schlaf / Doppelklopfen
// ==========================================================================
void bleSleep() {
  Bluefruit.Advertising.stop();
  bleSleeping = true;
  Serial.println("Bluetooth schlaeft - zweimal klopfen weckt");
}

// Doppelklopfen: weckt das Bluetooth und startet/stoppt den Panikmodus
void onDoubleTap(unsigned long now) {
  lastTapAt = now;
  bool woke = false;
  if (bleSleeping) {
    Bluefruit.Advertising.start(0);
    bleSleeping = false;
    bleIdleSince = now;
    woke = true;
    Serial.println("Bluetooth geweckt");
  }
  if (panicEnabled) {
    if (panicActive) stopPanic(now, false);
    else             startPanic(now);
  } else if (woke && haveDrv) {          // ohne Panikmodus: kurzes Doppelsurren als Bestätigung
    buzz(110, 40); delay(80); buzz(110, 40);
    signalDone();
  }
}

// ==========================================================================
// Akku
// ==========================================================================
uint8_t lipoPercent(float v) {
  static const float V[] = {3.30,3.60,3.70,3.75,3.79,3.83,3.87,3.92,3.98,4.06,4.15};
  static const float P[] = {   0,   5,  15,  25,  40,  50,  60,  70,  80,  90, 100};
  const int n = sizeof(V)/sizeof(V[0]);
  if (v <= V[0])   return 0;
  if (v >= V[n-1]) return 100;
  for (int i = 1; i < n; i++) {
    if (v < V[i]) {
      float f = (v - V[i-1]) / (V[i] - V[i-1]);
      return (uint8_t)(P[i-1] + f * (P[i] - P[i-1]) + 0.5f);
    }
  }
  return 100;
}

float readBatteryVoltage() {
  digitalWrite(VBAT_ENABLE, LOW);
  delay(5);
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(PIN_VBAT); delayMicroseconds(150); }
  float raw  = sum / 16.0f;
  float vPin = raw * (3.0f / 4096.0f);
  return vPin * (1510.0f / 510.0f) * VBAT_CAL;
}

uint8_t readBatteryPercent() {
  float v = readBatteryVoltage();
  uint8_t pct = lipoPercent(v);
  Serial.print("VBAT = "); Serial.print(v, 3); Serial.print(" V  -> "); Serial.print(pct); Serial.println(" %");
  return pct;
}

void updateBattery() {
  uint8_t pct = readBatteryPercent();
  blebas.write(pct);
  if (Bluefruit.connected()) blebas.notify(pct);     // die Seite zeigt den Stand sofort an
}

// ==========================================================================
// BLE Callbacks
// ==========================================================================
void vibe_write_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_handle; (void)chr;
  if (len < 1 || !haveDrv) return;
  uint8_t strength = data[0];
  if (strength == 0) { targetVibe = 0; vibeOffAt = 0; return; }
  targetVibe = map(strength, 1, 255, RTP_MIN, RTP_MAX);
  vibeOffAt  = millis() + VIBE_MS;
}

void heart_write_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_handle; (void)chr;
  if (len < 3) return;
  int bpm = data[0];
  if (bpm < 30) bpm = 30;
  if (bpm > 180) bpm = 180;
  heartBpm      = (uint8_t)bpm;
  heartStrength = data[1];
  heartOn       = (data[2] & 1) && haveDrv;
  cfgDirty = true;
}

void breath_write_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_handle; (void)chr;
  if (len < 3) return;
  int rate = data[0];
  if (rate < 4)  rate = 4;
  if (rate > 30) rate = 30;
  breathRate     = (uint8_t)rate;
  breathStrength = data[1];
  breathOn       = (data[2] & 1) && haveDrv;
  if (len >= 4) breathStyle = (data[3] > 4) ? 0 : data[3];
  cfgDirty = true;
}

void plan_write_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_handle; (void)chr;
  if (len < 5) return;
  int iv = data[1] | (data[2] << 8);
  int du = data[3] | (data[4] << 8);
  if (iv < 1) iv = 1;
  if (iv > 1440) iv = 1440;
  if (du < 1) du = 1;
  if (du > iv) du = iv;
  schedOn          = data[0] & 1;
  schedIntervalMin = (uint16_t)iv;
  schedDurationMin = (uint16_t)du;
  if (len >= 8) {
    announceStrength = data[5];
    int th = data[6];
    if (th < 1) th = 1;
    if (th > 31) th = 31;
    if (th != tapThs) { tapThs = (uint8_t)th; tapCfgDirty = true; }
    panicEnabled = data[7] & 1;
  }
  cfgDirty = true;
}

void connect_cb(uint16_t handle) {
  (void)handle;
  batNextAt = 0;                         // Akkustand gleich beim Verbinden aktualisieren
  Serial.println("Verbunden");
}

void disconnect_cb(uint16_t handle, uint8_t reason) {
  (void)handle; (void)reason;
  targetVibe = 0; vibeOffAt = 0;
  bleIdleSince = millis();
  if (cfgDirty) savePending = true;     // gespeichert wird in loop()
  Serial.println("Getrennt - werbe wieder...");
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(motionService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 1600);   // 100 ms / 1 s
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ==========================================================================
// setup
// ==========================================================================
void setup() {
  // LEDs aus (active-low) und nie mehr angefasst
  pinMode(LED_BLUE, OUTPUT);    digitalWrite(LED_BLUE, HIGH);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1000) delay(10);
  Serial.println("== XiaoSwitch Start ==");

  InternalFS.begin();
  loadConfig();

  // IMU nur ansprechen, wenn der interne Bus frei ist (sonst hängt Wire1)
  imuOk = false;
#if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL)
  pinMode(PIN_WIRE1_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE1_SCL, INPUT_PULLUP);
  delay(5);
  bool busFrei = digitalRead(PIN_WIRE1_SDA) && digitalRead(PIN_WIRE1_SCL);
#else
  bool busFrei = true;
#endif
  if (busFrei && imu.begin() == 0) {
    imuOk = true;
    configureIMU();
    nrf_gpio_cfg_input(IMU_INT1_PIN, NRF_GPIO_PIN_NOPULL);   // Doppelklopf-Interrupt
    Serial.println("IMU ok");
  } else {
    Serial.println("IMU nicht verfuegbar - Bluetooth bleibt immer auffindbar, ohne Bewegung, Stillstand und Klopfen");
  }

  // DRV2605
  if (drv.begin()) {
    haveDrv = true;
    drv.writeRegister8(0x01, 0x80);   // Reset (setzt auch das RTP-Format auf signed)
    delay(50);
#if MOTOR_LRA
    drv.useLRA();
    drv.selectLibrary(6);
#else
    drv.useERM();
    drv.selectLibrary(1);
    drv.writeRegister8(0x1D, drv.readRegister8(0x1D) | 0x20);   // ERM_OPEN_LOOP
#endif
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0);
    Serial.print("DRV2605 STATUS=0x"); Serial.println(drv.readRegister8(0x00), HEX);
  } else {
    Serial.println("DRV2605 nicht gefunden - Vibration deaktiviert.");
    heartOn = false; breathOn = false;
  }

  // Akku
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  // BLE (die Bluefruit-Bibliothek lässt sonst die blaue LED beim Werben blinken und bei Verbindung leuchten)
  Bluefruit.autoConnLed(false);
  Bluefruit.begin();
  Bluefruit.setTxPower(0);
  Bluefruit.setName("XiaoSwitch");
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
  Bluefruit.Periph.setConnInterval(16, 48);
  Serial.println("BLE init ok");

  blebas.begin();
  blebas.write(readBatteryPercent());

  motionService.begin();

  motionChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  motionChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  motionChar.setFixedLen(1);
  motionChar.begin();
  motionChar.write8(0);

  vibeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  vibeChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  vibeChar.setFixedLen(1);
  vibeChar.setWriteCallback(vibe_write_cb);
  vibeChar.begin();

  heartChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  heartChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  heartChar.setFixedLen(3);
  heartChar.setWriteCallback(heart_write_cb);
  heartChar.begin();
  uint8_t heartInit[3] = { heartBpm, heartStrength, (uint8_t)(heartOn ? 1 : 0) };
  heartChar.write(heartInit, 3);

  breathChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  breathChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  breathChar.setMaxLen(4);
  breathChar.setWriteCallback(breath_write_cb);
  breathChar.begin();
  uint8_t breathInit[4] = { breathRate, breathStrength, (uint8_t)(breathOn ? 1 : 0), breathStyle };
  breathChar.write(breathInit, 4);

  planChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  planChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  planChar.setMaxLen(8);
  planChar.setWriteCallback(plan_write_cb);
  planChar.begin();
  uint8_t planInit[8] = { (uint8_t)(schedOn ? 1 : 0),
                          (uint8_t)(schedIntervalMin & 0xFF), (uint8_t)(schedIntervalMin >> 8),
                          (uint8_t)(schedDurationMin & 0xFF), (uint8_t)(schedDurationMin >> 8),
                          announceStrength, tapThs, (uint8_t)(panicEnabled ? 1 : 0) };
  planChar.write(planInit, 8);

  startAdv();
  bleIdleSince = millis();
  cfgDirty = false;
  Serial.println("XiaoSwitch bereit - werbe...");
}

// ==========================================================================
// loop
// ==========================================================================
void loop() {
  unsigned long now = millis();

  // Doppelklopfen (der Sensor meldet es per Interrupt, kein ständiges Nachfragen über I2C)
  if (imuOk && nrf_gpio_pin_read(IMU_INT1_PIN)) {
    uint8_t s = 0;
    if (imu.readRegisters(REG_TAP_SRC, &s, 1) && (s & 0x10) && now - lastTapAt >= TAP_SPERRE_MS) {
      onDoubleTap(now);
      now = millis();
      uint8_t clr = 0; imu.readRegisters(REG_TAP_SRC, &clr, 1);   // Latch sicher löschen
    }
  }
  if (tapCfgDirty && imuOk) { tapCfgDirty = false; applyTapThreshold(); }

  scheduleUpdate(now);
  rhythmUpdate(now);

  bool rhythmRuns = haveDrv && !panicActive && sessionActive && (heartOn || breathOn);

  // Vibe: sofort auf Zielstärke, dann weich ausklingen (pausiert, solange Herz, Atem oder Panik laufen)
  if (!rhythmRuns && !panicActive && now - lastVibeTick >= SAMPLE_MS) {
    lastVibeTick = now;
    if (currentVibe < targetVibe) {
      currentVibe = targetVibe;
      drv.setRealtimeValue(currentVibe);
    } else if (currentVibe > targetVibe) {
      currentVibe -= 4; if (currentVibe < targetVibe) currentVibe = targetVibe;
      drv.setRealtimeValue(currentVibe);
    }
  }

  // Timer
  if (vibeOffAt && now >= vibeOffAt) { targetVibe = 0; vibeOffAt = 0; }
  if (now >= batNextAt) { batNextAt = now + BAT_MS; updateBattery(); }

  // Konfiguration speichern (nach dem Trennen)
  if (savePending && !Bluefruit.connected()) {
    savePending = false;
    cfgDirty = false;
    saveConfig();
  }

  // Bluetooth-Schlaf: 2 min ohne Verbindung (nur mit funktionierender IMU, sonst gäbe es keine Weckquelle)
  if (Bluefruit.connected()) {
    bleIdleSince = now;
  } else if (!bleSleeping && imuOk && now - bleIdleSince >= BLE_SLEEP_MS) {
    bleSleep();
  }

  // Statuszeile
  if (DEBUG_STATUS && now - lastStatusAt >= STATUS_MS) {
    lastStatusAt = now;
    Serial.print("t="); Serial.print(now / 1000);
    Serial.print("s imu="); Serial.print(imuOk);
    Serial.print(" drv="); Serial.print(haveDrv);
    Serial.print(" ble="); Serial.print(Bluefruit.connected() ? "verbunden" : (bleSleeping ? "schlaeft" : "werbend"));
    Serial.print(" session="); Serial.print(sessionActive);
    Serial.print(" auto="); Serial.print(schedOn);
    Serial.print(" zustand="); Serial.print((int)schedState);
    Serial.print(" panik="); Serial.println(panicActive);
  }

  // IMU auslesen: Bewegung (verbunden) und Stillstand (beim Warten auf die Session)
  bool wantMotion = imuOk && (Bluefruit.connected() || schedState == SCH_WAIT);
  if (wantMotion && now - lastSampleAt >= SAMPLE_MS) {
    lastSampleAt = now;
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    if (!primed) { pax = ax; pay = ay; paz = az; primed = true; }
    else {
      float dx = ax - pax, dy = ay - pay, dz = az - paz;
      float delta = sqrt(dx * dx + dy * dy + dz * dz);
      pax = ax; pay = ay; paz = az;

      motionEma = 0.75f * motionEma + 0.25f * delta;
      if (motionEma < STILL_DELTA) { if (!stillSince) stillSince = now; }
      else stillSince = 0;

      if (Bluefruit.connected()) {
        if (delta > NOISE_FLOOR) {
          int v = (int)(delta * 100.0 + 0.5);
          if (v > 255) v = 255;
          lastMotionVal = (uint8_t)v;
          motionChar.notify8((uint8_t)v);
          wasMoving = true;
        } else if (wasMoving) {
          lastMotionVal = 0;
          motionChar.notify8(0);
          wasMoving = false;
        }
      }
    }
  } else if (!wantMotion) {
    primed = false;
    stillSince = 0;
    motionEma = 1.0f;
  }

  // Im Leerlauf lange schlafen (spart Strom), bei Rhythmus kurz (genaues Timing), beim Warten/Verbinden mittel
  int dly = 50;
  if (rhythmRuns || panicActive) dly = 5;
  else if (Bluefruit.connected() || schedState == SCH_WAIT) dly = 20;
  delay(dly);
}
