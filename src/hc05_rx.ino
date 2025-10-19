#include <Servo.h>
#include <SoftwareSerial.h>

/* -------------------- Bluetooth (HC-05) -------------------- */
// HC-05 Verdrahtung (UNO/Nano):
//   HC-05 TXD -> D10 (Arduino RX)
//   HC-05 RXD <- D11 (Arduino TX über Spannungsteiler 5V->3.3V)
//   GND gemeinsam, VCC an 5V
SoftwareSerial BT(11, 10);      // RX<-11, TX->10
#define BT_BAUD 9600

/* -------------------- Servos & Grenzen (mit iPhone FOV) ---- */
const uint8_t PIN_PAN  = 13;
const uint8_t PIN_TILT = 12;
Servo pan, tilt;

// Arbeitsbereich (logisch, ohne Offset)
const int PAN_MIN  = 30;
const int PAN_MAX  = 150;
const int TILT_MIN = 40;
const int TILT_MAX = 120;

// Offsets (Feinjustage – werden erst bei write() addiert)
const int PAN_OFFSET  = 31;
const int TILT_OFFSET = 40;

// Bewegung / Glättung
const float PAN_SPEED_DEG_S  = 60.0f;  // schneller, aber weich
const float TILT_SPEED_DEG_S = 30.0f;
const float DEADZONE_DEG     = 0.3f;

// Link-Timeout (nur außerhalb des Tilt-Modus relevant)
const unsigned long LINK_TIMEOUT_MS = 1500;

/* -------------------- Zustände -------------------- */
float panPos = 90,  panTarget = 90;
float tiltPos = 90, tiltTarget = 90;

bool tiltAdjustMode = false;           // 2x Klick toggelt
unsigned long lastPktMs = 0;
unsigned long lastUpdateMs = 0;

String lineBuf;

/* -------------------- Helfer -------------------- */
inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

inline void smoothMove(float &p, float t, float maxSpeed, float dt) {
  float d = t - p;
  if (fabs(d) <= DEADZONE_DEG) return;
  float step = constrain(d, -maxSpeed * dt, maxSpeed * dt);
  p += step;
}

inline void applyServos() {
  pan.write ((int)round(panPos  + PAN_OFFSET));
  tilt.write((int)round(tiltPos + TILT_OFFSET));
}

void printStatus() {
  Serial.print(F("[STAT] Pan="));  Serial.print(panPos,1);
  Serial.print(F(" Tilt="));       Serial.print(tiltPos,1);
  Serial.print(F("  TA="));        Serial.println(tiltAdjustMode ? F("ON") : F("OFF"));
}

/* -------------------- Parser -------------------- */
void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;

  // Wir haben gültige Daten empfangen
  lastPktMs = millis();

  if (cmd.startsWith("PT:")) {
    // Pan-Befehle NUR erlauben, wenn NICHT im Tilt-Modus
    if (!tiltAdjustMode) {
      float v = cmd.substring(3).toFloat();
      panTarget = clampf(v, PAN_MIN, PAN_MAX);
    }
  }
  else if (cmd.startsWith("TT:")) {
    float v = cmd.substring(3).toFloat();
    tiltTarget = clampf(v, TILT_MIN, TILT_MAX);
  }
  else if (cmd == "RST" || cmd == "BTN:3") {
    // Reset auf Nullstellung (Offsets wirken bei write())
    panTarget  = 90;
    tiltTarget = 90;
  }
  else if (cmd == "BTN:2" || cmd == "TA:ON" || cmd == "TA:1" || cmd == "TA:TRUE" || cmd == "TA:OFF" || cmd == "TA:0" || cmd == "TA:FALSE") {
    // Tilt-Modus toggeln oder explizit setzen
    if (cmd.startsWith("TA:")) {
      String v = cmd.substring(3); v.toUpperCase();
      tiltAdjustMode = (v == "ON" || v == "1" || v == "TRUE");
    } else {
      tiltAdjustMode = !tiltAdjustMode; // BTN:2
    }
    Serial.print(F("[TA] ")); Serial.println(tiltAdjustMode ? F("ON") : F("OFF"));
  }
  else if (cmd == "BTN:1") {
    // Bestätigen/Beenden – nur relevant, wenn gerade aktiv
    if (tiltAdjustMode) {
      tiltAdjustMode = false;
      Serial.println(F("[TA] OFF (confirmed)"));
    }
  }

  // Debug kompakt
  Serial.print(F("[CMD] ")); Serial.print(cmd);
  Serial.print(F(" | TA=")); Serial.println(tiltAdjustMode ? F("ON") : F("OFF"));
}

/* -------------------- Setup / Loop -------------------- */
void setup() {
  Serial.begin(115200);
  BT.begin(BT_BAUD);

  pan.attach(PIN_PAN);
  tilt.attach(PIN_TILT);

  // Start mittig (Offsets wirken in applyServos)
  panPos = panTarget = 90;
  tiltPos = tiltTarget = 90;
  applyServos();

  lastPktMs = lastUpdateMs = millis();

  Serial.println(F("== VIEWMATE Receiver (HC-05) =="));
  Serial.println(F("Regeln: 2x Klick = TiltModus an/aus, 1x = bestaetigen, 3x = Reset."));
  Serial.println(F("Im TiltModus: KEIN Pan (keine Befehle, kein Auto-Center, kein Nachlaufen)."));
  printStatus();
}

void loop() {
  /* --- Bluetooth lesen, zeilenweise parsen --- */
  while (BT.available()) {
    char c = BT.read();
    if (c == '\r' || c == '\n') {
      if (lineBuf.length() > 0) { handleCommand(lineBuf); lineBuf = ""; }
    } else {
      lineBuf += c;
      if (lineBuf.length() > 96) lineBuf = ""; // Schutz
    }
  }

  // /* --- Wenn Link weg: nur außerhalb des Tilt-Modus Pan zurück zur Mitte --- */
  // if (!tiltAdjustMode && (millis() - lastPktMs > LINK_TIMEOUT_MS)) {
  //   panTarget = 90;
  // }
  // Im Tilt-Modus: NICHTS an Pan anfassen (weder Target noch Bewegung)

  /* --- Weiche Bewegung (~50 Hz) --- */
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt >= 0.02f) {
    // Pan NUR bewegen, wenn NICHT im Tilt-Modus
    if (!tiltAdjustMode) {
      smoothMove(panPos, panTarget, PAN_SPEED_DEG_S, dt);
    }
    // Tilt IMMER bewegen (Targets kommen auch im Tilt-Modus)
    smoothMove(tiltPos, tiltTarget, TILT_SPEED_DEG_S, dt);

    applyServos();
    lastUpdateMs = now;
  }

  // optionaler Status alle 1.5 s
  static unsigned long dbg = 0;
  if (now - dbg > 1500) {
    dbg = now;
    printStatus();
  }
}

