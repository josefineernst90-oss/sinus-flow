/*
  Xiao nRF52840 Sense Plus — BLE Bewegungs-Schalter + Vibration (DRV2605, ERM)  "XiaoSwitch"
  -----------------------------------------------------------------------------------------
  Xiao -> App : Bewegungsstärke (Notify), die App entscheidet per Schwelle.
  App  -> Xiao: Vibrationsbefehl (Write) -> DRV2605 treibt den Motor mit der Stärke.

  Core "Seeeduino nRF52" (Bluefruit-basiert). NICHT ArduinoBLE.
  Bibliotheken: bluefruit.h (Core), Wire (Core), "Adafruit DRV2605 Library"

  Datenmodell (identisch zur App):
    Service        19b10000-e8f2-537e-4f6c-d104768a1214
    Motion-Char    19b10001-...  (Read | Notify)  uint8 = Bewegungsstärke (g*100, 0..255)
    Vibe-Char      19b10002-...  (Write)          uint8 = Vibrationsstärke (0..255)
    Herz-Char      19b10003-...  (Read | Write)   3 Byte = [BPM 30..180, Stärke 0..255, An 0/1]
    Atem-Char      19b10004-...  (Read | Write)   3 Byte = [Atemzüge/min 4..30, Stärke 0..255, An 0/1]

  DRV2605 (I2C 0x5A): VIN->3V3, GND->GND, SDA->D4, SCL->D5, Motor an die Motor-Ausgänge.
  RTP-Format: signed (Standard). 0 = aus, 127 = Vollgas, negative Werte = aktives Bremsen.

  Herzschlag und Atmung laufen parallel auf demselben Motor (Werte werden addiert).

  Ist die IMU nicht ansprechbar (interner Bus festgehalten), läuft alles andere trotzdem:
  BLE, Herzschlag, Atmung, Vibration. Es gibt dann keine Bewegungsdaten und keinen Schlafmodus.

  Debug: DEBUG_STATUS = true -> alle 2 s Statuszeile im Serial Monitor + blaues Aufblitzen.
*/

#include <bluefruit.h>
#include "Wire.h"
#include "Adafruit_DRV2605.h"

// ---- Debug ----
const bool DEBUG_STATUS = true;    // Statuszeile + blaues Aufblitzen
const bool DEBUG_STAGES = true;    // beim Start blau blinken: 1=Start 2=IMU 3=DRV 4=BLE 5=fertig

// ---- Pins ----
#define IMU_INT1_PIN  11   // P0.11, intern mit LSM6DS3 INT1 verdrahtet

// ---- Zeitkonstanten ----
const float         NOISE_FLOOR     = 0.05;    // g — darunter gilt es als Ruhe
const unsigned long SAMPLE_MS       = 40;      // ~25 Hz Abtastung
const unsigned long VIBE_MS         = 350;     // Pulsdauer je Befehl
const unsigned long BAT_MS          = 30000;   // Akku alle 30 s messen
const unsigned long IDLE_TIMEOUT_MS = 60000;   // 60 s ohne Bewegung (verbunden) -> systemOff
const unsigned long STATUS_MS       = 2000;    // Statuszeile

// false = ohne Verbindung (werbend) nie schlafen, bleibt immer auffindbar
const bool SLEEP_BEIM_WERBEN = false;

// ---- Motor ----
#define MOTOR_LRA 0   // 0 = ERM (Exzentermotor, dein Motor), 1 = LRA
const int RTP_MIN = 30;    // kleinste spürbare Stärke
const int RTP_MAX = 127;   // signed: 127 = Vollgas

// ---- Herzschlag ----
const bool          HERZ_BLEIBT_BEI_TRENNUNG = false;
const unsigned long S1_MS     = 130;     // Dauer von "lub"
const unsigned long S2_MS     = 100;     // Dauer von "dub"
const float         S2_FAKTOR = 0.65f;   // "dub" ist leiser als "lub"

// Anstoß + Gegenschub, damit der ERM-Motor knackig statt matschig schlägt
const unsigned long KICK_MS      = 18;     // Anstoß-Dauer am Pulsbeginn
const unsigned long BRAKE_MS     = 14;     // Bremsdauer am Pulsende
const float         KICK_FAKTOR  = 1.5f;   // Anstoß = 1,5 x Stärke (max. 127)
const float         BRAKE_FAKTOR = 0.5f;   // Bremse = 50 % der Stärke, negativ

// ---- Atmung ----
const bool  ATEM_BLEIBT_BEI_TRENNUNG = false;
const float ATEM_EINATMEN_ANTEIL     = 0.40f;  // 40 % Einatmen, 60 % Ausatmen
const int   ATEM_MAX                 = 80;     // höchste Atem-Stärke (bleibt unter dem Herzschlag)

// ---- LSM6DS3-Register ----
#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_OUTX_L_XL   0x28
#define REG_TAP_CFG     0x58   // Bit7 = INTERRUPTS_ENABLE (ohne das feuert Wake-up nie)
#define REG_WAKE_UP_THS 0x5B
#define REG_WAKE_UP_DUR 0x5C
#define REG_MD1_CFG     0x5E   // Bit5 = Wake-up auf INT1

// ---- GPREGRET Marker ----
#define SLEEP_MARKER 0x42

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

BLEService        motionService(MOTION_SERVICE_UUID);
BLECharacteristic motionChar(MOTION_CHAR_UUID);
BLECharacteristic vibeChar(VIBE_CHAR_UUID);
BLECharacteristic heartChar(HEART_CHAR_UUID);
BLECharacteristic breathChar(BREATH_CHAR_UUID);
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

// ---- Zustand ----
float pax = 0, pay = 0, paz = 0;
bool  primed    = false;
bool  wasMoving = false;
unsigned long lastMoveAt   = 0;
unsigned long ledOffAt     = 0;
unsigned long blueOffAt    = 0;
unsigned long lastStatusAt = 0;
unsigned long vibeOffAt    = 0;
int  currentVibe = 0;
int  targetVibe  = 0;
unsigned long lastVibeTick = 0;
unsigned long lastSampleAt = 0;
uint8_t lastMotionVal = 0;

volatile bool heartOn  = false;
volatile bool breathOn = false;

// ---- Debug-Helfer ----
void stage(int n) {
  if (!DEBUG_STAGES) return;
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BLUE, LOW);  delay(80);
    digitalWrite(LED_BLUE, HIGH); delay(150);
  }
  delay(400);
}

// ==========================================================================
// IMU-Konfiguration
// ==========================================================================
void configureIMU_Normal() {
  imu.writeRegister(REG_CTRL2_G,  0x00);   // Gyro aus
  imu.writeRegister(REG_CTRL1_XL, 0x20);   // Accel 26 Hz, ±2g
}

void configureIMU_WOM() {
  imu.writeRegister(REG_CTRL2_G,     0x00);  // Gyro aus
  imu.writeRegister(REG_CTRL1_XL,    0x20);  // Accel 26 Hz
  imu.writeRegister(REG_TAP_CFG,     0x80);  // INTERRUPTS_ENABLE, sonst feuert Wake-up nie
  imu.writeRegister(REG_WAKE_UP_DUR, 0x00);  // keine Latenz
  imu.writeRegister(REG_WAKE_UP_THS, 0x02);  // Schwelle ~62 mg
  imu.writeRegister(REG_MD1_CFG,     0x20);  // Wake-up auf INT1
}

// ==========================================================================
// Sleep
// ==========================================================================
void goToSleep() {
  Serial.print("Idle seit "); Serial.print(IDLE_TIMEOUT_MS / 1000); Serial.println(" s -> systemOff");
  Serial.flush();

  heartOn = false;
  breathOn = false;
  if (haveDrv) { drv.setRealtimeValue(0); currentVibe = 0; targetVibe = 0; }
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  configureIMU_WOM();
  delay(10);

  // INT1 weckt auf steigende Flanke
  nrf_gpio_cfg_sense_input(IMU_INT1_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
  // Warten bis INT1 low ist, sonst würde der Chip sofort wieder aufwachen
  uint32_t w0 = millis();
  while (nrf_gpio_pin_read(IMU_INT1_PIN) && millis() - w0 < 2000) delay(10);

  if (Bluefruit.connected()) Bluefruit.disconnect(Bluefruit.connHandle());
  Bluefruit.Advertising.stop();
  delay(100);

  sd_power_gpregret_clr(0, 0xFF);
  sd_power_gpregret_set(0, SLEEP_MARKER);
  sd_power_system_off();
  while (1);
}

// ==========================================================================
// Rhythmus-Engine: Herzschlag + Atmung parallel
// ==========================================================================
volatile uint8_t heartBpm      = 60;
volatile uint8_t heartStrength = 160;
volatile uint8_t breathRate     = 12;    // Atemzüge pro Minute
volatile uint8_t breathStrength = 120;

unsigned long beatStartAt = 0;
unsigned long beatPeriod  = 1000;
unsigned long beatGap     = 320;
unsigned long breathStartAt = 0;

int  lastRhythmVal = -1000;   // -1000 = noch nichts gesendet
bool heartWasOn    = false;
bool breathWasOn   = false;

static float pulseEnv(float t, float dur) {
  if (t < 0 || t >= dur) return 0;
  float x = t / dur;
  if (x < 0.15f) return x / 0.15f;
  float d = (1.0f - x) / 0.85f;
  return d * d;
}

static void heartStartBeat(unsigned long startAt) {
  beatStartAt = startAt;
  beatPeriod  = 60000UL / heartBpm;
  unsigned long gap = (unsigned long)(beatPeriod * 0.33f);
  if (gap < 150) gap = 150;
  if (gap > 320) gap = 320;
  beatGap = gap;
}

// Wert des Herzschlags (signed, kann negativ sein = Bremse)
static int heartValue(unsigned long now) {
  if (!heartWasOn) {
    heartStartBeat(now);
    heartWasOn = true;
  }

  if (now - beatStartAt >= beatPeriod) {
    unsigned long next = beatStartAt + beatPeriod;
    heartStartBeat(next);
    if (now - next >= beatPeriod) heartStartBeat(now);
  }

  float t   = (float)(now - beatStartAt);
  float tS2 = t - (float)beatGap;
  int maxv = (heartStrength == 0) ? 0 : (int)map(heartStrength, 1, 255, RTP_MIN, RTP_MAX);

  float v = pulseEnv(t, (float)S1_MS) + S2_FAKTOR * pulseEnv(tS2, (float)S2_MS);
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

// Wert der Atmung (0..ATEM_MAX): weiche Welle, Einatmen kürzer als Ausatmen
static int breathValue(unsigned long now) {
  if (!breathWasOn) {
    breathStartAt = now;
    breathWasOn = true;
  }
  if (breathStrength == 0) return 0;

  unsigned long period = 60000UL / breathRate;
  unsigned long pos    = (now - breathStartAt) % period;
  float p = (float)pos / (float)period;

  float b;
  if (p < ATEM_EINATMEN_ANTEIL) {
    b = 0.5f - 0.5f * cosf(PI * p / ATEM_EINATMEN_ANTEIL);                          // 0 -> 1
  } else {
    b = 0.5f + 0.5f * cosf(PI * (p - ATEM_EINATMEN_ANTEIL) / (1.0f - ATEM_EINATMEN_ANTEIL));   // 1 -> 0
  }

  int maxb = (int)map(breathStrength, 1, 255, RTP_MIN, ATEM_MAX);
  return (int)(b * maxb + 0.5f);
}

void rhythmUpdate(unsigned long now) {
  if (!haveDrv) return;

  if (!heartOn && heartWasOn)   heartWasOn  = false;
  if (!breathOn && breathWasOn) breathWasOn = false;

  if (!heartOn && !breathOn) {
    if (lastRhythmVal != -1000) {
      drv.setRealtimeValue(0);
      currentVibe = 0; targetVibe = 0;
      lastRhythmVal = -1000;
    }
    return;
  }

  int val = 0;
  if (heartOn)  val += heartValue(now);
  if (breathOn) val += breathValue(now);

  if (val >  RTP_MAX) val =  RTP_MAX;
  if (val < -RTP_MAX) val = -RTP_MAX;
  if (val != lastRhythmVal) { drv.setRealtimeValue((uint8_t)(int8_t)val); lastRhythmVal = val; }
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
}

void connect_cb(uint16_t handle) {
  (void)handle;
  digitalWrite(LED_BUILTIN, LOW); ledOffAt = millis() + 300;
  lastMoveAt = millis();
  Serial.println("Verbunden");
}

void disconnect_cb(uint16_t handle, uint8_t reason) {
  (void)handle; (void)reason;
  digitalWrite(LED_BUILTIN, HIGH);
  targetVibe = 0; vibeOffAt = 0;
  if (!HERZ_BLEIBT_BEI_TRENNUNG) heartOn = false;
  if (!ATEM_BLEIBT_BEI_TRENNUNG) breathOn = false;
  lastMoveAt = millis();
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
  pinMode(LED_BLUE, OUTPUT);    digitalWrite(LED_BLUE, HIGH);      // aus (active-low)
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);
  stage(1);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("== XiaoSwitch Start ==");

  bool fromSleep = (NRF_POWER->GPREGRET == SLEEP_MARKER);
  NRF_POWER->GPREGRET = 0;
  Serial.println(fromSleep ? "Wakeup aus systemOff (WOM)" : "Kaltstart");

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
    configureIMU_Normal();
    Serial.println("IMU ok");
  } else {
    Serial.println("IMU nicht verfuegbar (Bus belegt oder keine Antwort) - ohne Bewegung/Schlafmodus");
  }
  stage(2);

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
    Serial.print("DRV2605 STATUS=0x"); Serial.print(drv.readRegister8(0x00), HEX);
    Serial.print(" RATED=0x"); Serial.print(drv.readRegister8(0x16), HEX);
    Serial.print(" CLAMP=0x"); Serial.print(drv.readRegister8(0x17), HEX);
    Serial.print(" FB=0x");    Serial.print(drv.readRegister8(0x1A), HEX);
    Serial.print(" CTRL3=0x"); Serial.println(drv.readRegister8(0x1D), HEX);
  } else {
    Serial.println("DRV2605 nicht gefunden - Vibration deaktiviert.");
  }
  stage(3);

  // Akku
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  // BLE
  Bluefruit.begin();
  Bluefruit.setTxPower(0);
  Bluefruit.setName("XiaoSwitch");
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
  Bluefruit.Periph.setConnInterval(16, 48);
  Serial.println("BLE init ok");
  stage(4);

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
  uint8_t heartInit[3] = { 60, 160, 0 };
  heartChar.write(heartInit, 3);

  breathChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  breathChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  breathChar.setFixedLen(3);
  breathChar.setWriteCallback(breath_write_cb);
  breathChar.begin();
  uint8_t breathInit[3] = { 12, 120, 0 };
  breathChar.write(breathInit, 3);

  startAdv();
  lastMoveAt = millis();
  Serial.println("XiaoSwitch bereit - werbe...");
  stage(5);
}

// ==========================================================================
// loop
// ==========================================================================
void loop() {
  unsigned long now = millis();

  rhythmUpdate(now);

  // Vibe: sofort auf Zielstärke, dann weich ausklingen (pausiert, solange Herz oder Atem laufen)
  if (!heartOn && !breathOn && now - lastVibeTick >= SAMPLE_MS) {
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
  if (ledOffAt  && now >= ledOffAt)  { digitalWrite(LED_BUILTIN, HIGH); ledOffAt = 0; }
  if (blueOffAt && now >= blueOffAt) { digitalWrite(LED_BLUE, HIGH);    blueOffAt = 0; }
  if (now >= batNextAt) { batNextAt = now + BAT_MS; blebas.write(readBatteryPercent()); }

  // Statuszeile + blaues Aufblitzen
  if (DEBUG_STATUS && now - lastStatusAt >= STATUS_MS) {
    lastStatusAt = now;
    Serial.print("t="); Serial.print(now / 1000);
    Serial.print("s imu="); Serial.print(imuOk);
    Serial.print(" drv="); Serial.print(haveDrv);
    Serial.print(" ble="); Serial.print(Bluefruit.connected() ? "verbunden" : "werbend");
    Serial.print(" motion="); Serial.print(lastMotionVal);
    Serial.print(" herz="); Serial.print(heartOn);
    Serial.print(" atem="); Serial.println(breathOn);
    digitalWrite(LED_BLUE, LOW); blueOffAt = now + 30;
  }

  // IMU auslesen (nur wenn verbunden und IMU ok)
  if (now - lastSampleAt >= SAMPLE_MS) {
    lastSampleAt = now;
    if (imuOk && Bluefruit.connected()) {
      float ax = imu.readFloatAccelX();
      float ay = imu.readFloatAccelY();
      float az = imu.readFloatAccelZ();

      if (!primed) { pax = ax; pay = ay; paz = az; primed = true; }
      else {
        float dx = ax - pax, dy = ay - pay, dz = az - paz;
        float delta = sqrt(dx * dx + dy * dy + dz * dz);
        pax = ax; pay = ay; paz = az;

        if (delta > NOISE_FLOOR) {
          int v = (int)(delta * 100.0 + 0.5);
          if (v > 255) v = 255;
          lastMotionVal = (uint8_t)v;
          motionChar.notify8((uint8_t)v);
          wasMoving  = true;
          lastMoveAt = now;
        } else if (wasMoving) {
          lastMotionVal = 0;
          motionChar.notify8(0);
          wasMoving = false;
        }
      }
    } else {
      primed = false;
    }
  }

  // Idle-Timeout: nur mit funktionierender IMU (sonst gäbe es keine Weckquelle)
  if (heartOn || breathOn) lastMoveAt = now;
  if (!SLEEP_BEIM_WERBEN && !Bluefruit.connected()) lastMoveAt = now;
  if (imuOk && now - lastMoveAt > IDLE_TIMEOUT_MS) {
    goToSleep();
  }

  delay(5);
}