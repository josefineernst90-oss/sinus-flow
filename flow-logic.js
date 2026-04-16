'use strict';

// --- ZUSTAND ---
let sinusState = "STANDBY";
let nextPhaseIsEinatmen = true;
let sessionLogs = JSON.parse(localStorage.getItem('sinus_logs') || '[]');
let phaseStartTime = Date.now();
let lastToggleTime = 0; // Sperre gegen Springen

let isCalibrated = false;
let autopilotActive = false;
const REQUIRED_CYCLES = 5;

// --- AUDIO ---
let audioCtx;
function playPing(freq) {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') audioCtx.resume();
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.frequency.setValueAtTime(freq, audioCtx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + 0.1);
    osc.connect(gain); gain.connect(audioCtx.destination);
    osc.start(); osc.stop(audioCtx.currentTime + 0.1);
}

// --- LOGIK ---
function togglePhase() {
    let now = Date.now();
    // 1.5 Sekunden Sperre: Verhindert nervöses Hin-und-Her-Springen
    if (now - lastToggleTime < 1500) return; 
    
    let oldPhase = sinusState;
    sinusState = (sinusState === "EINATMEN") ? "AUSATMEN" : "EINATMEN";
    nextPhaseIsEinatmen = (sinusState === "EINATMEN");
    
    playPing(sinusState === "EINATMEN" ? 660 : 440);
    if (navigator.vibrate) navigator.vibrate(sinusState === "EINATMEN" ? 40 : [20, 20]);
    
    if (pointers[0]) pointers[0].color = generateColor();
    
    lastToggleTime = now;
    logPhaseChange(oldPhase);
}

function logPhaseChange(phase) {
    let duration = (Date.now() - phaseStartTime) / 1000;
    if (duration > 0.8) {
        sessionLogs.push({ phase: phase, duration: duration });
        localStorage.setItem('sinus_logs', JSON.stringify(sessionLogs));
    }
    
    // Kalibrierung prüfen
    if (!isCalibrated && sessionLogs.length >= REQUIRED_CYCLES) {
        isCalibrated = true;
        document.getElementById('autopilot-btn').style.opacity = "1";
        document.getElementById('stats-display').style.color = "#00ff7f";
    }
    
    phaseStartTime = Date.now();
    updateUI();
}

// Autopilot: Das Handy übernimmt die Führung
function runAutopilot() {
    if (!autopilotActive || sinusState === "STANDBY") return;
    
    let currentDuration = (Date.now() - phaseStartTime) / 1000;
    let relevant = sessionLogs.filter(l => l.phase === sinusState);
    let avg = relevant.length > 0 ? relevant.reduce((a,b) => a + b.duration, 0) / relevant.length : 5.0;

    // Automatischer U-Turn wenn Durchschnitt erreicht
    if (currentDuration >= avg) {
        togglePhase();
    }
}

// --- INPUTS ---
window.addEventListener('mousedown', () => {
    if (sinusState === "STANDBY") {
        sinusState = nextPhaseIsEinatmen ? "EINATMEN" : "AUSATMEN";
        phaseStartTime = Date.now();
    }
});

document.getElementById('autopilot-btn').addEventListener('click', function() {
    if (!isCalibrated) return alert("Erst 5 Zyklen atmen zum Kalibrieren!");
    autopilotActive = !autopilotActive;
    this.classList.toggle('active');
    this.innerText = autopilotActive ? "AUTOPILOT: AN" : "AUTOPILOT: AUS";
});

// Clean Session Button
document.getElementById('clean-session').addEventListener('click', () => {
    if(confirm("Löschen?")) { sessionLogs = []; localStorage.clear(); location.reload(); }
});