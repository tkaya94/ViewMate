#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

// ========= Zielgerät & UUIDs =========
// Name so, wie er in nRF Connect steht (BLE-Name)
static const char* TARGET_NAME = "HC-05";

// Häufige UART-Profile von HM-10/BT05-Clones:
static BLEUUID UART_SERVICE("0000FFE0-0000-1000-8000-00805F9B34FB");
static BLEUUID UART_CHAR   ("0000FFE1-0000-1000-8000-00805F9B34FB");

// Falls nRF Connect andere UUIDs zeigt, hier austauschen:
// Nordic UART (Beispiel):
// static BLEUUID UART_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
// static BLEUUID UART_CHAR   ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // oft RX/TX gedreht; ggf. 6E400002 für Write

// ========= Joystick-Pins =========
const int JOY_X_PIN  = 34;   // VRx (Pan)
const int JOY_Y_PIN  = 35;   // VRy (Tilt)
const int JOY_SW_PIN = 27;   // SW (Taster, gegen GND)

// ========= Joystick / ADC =========
int   adcMidX = 2048, adcMidY = 2048;   // wird kalibriert
int   JOY_DEAD = 220;                   // ~5% Deadzone
const int   SAMPLES = 8;                // ADC Mehrfachmessung
const float EMA_ALPHA = 0.18f;          // Glättung (kleiner = ruhiger)

// ========= Bewegungslogik =========
const int PAN_MIN  = 30, PAN_MAX  = 150;
const int TILT_MIN = 40, TILT_MAX = 80;
float panTarget  = 90.0f;
float tiltTarget = 90.0f;

const bool  PAN_RETURN_TO_MID  = false; // Loslassen = stehenbleiben
float panSpeedDegPerSec  = 60.0f;
float tiltSpeedDegPerSec = 20.0f;

// ========= Taster / Multi-Click =========
const unsigned long DEBOUNCE_MS    = 25;
const unsigned long MULTI_CLICK_MS = 1200;
const unsigned long HOLD_CAL_MS    = 2000;  // 2s halten = Kalibrieren
bool          swPrev = HIGH;                // Pullup: HIGH = losgelassen
unsigned long lastSwChangeMs = 0;
unsigned long lastClickMs    = 0;
int           clickCount     = 0;
bool          tiltAdjustMode = false;

// ========= Senden & Debug =========
unsigned long lastSendMs = 0;
const unsigned long SEND_MS = 30;  // ~33 Hz
unsigned long lastDbgMs  = 0;
const unsigned long DBG_MS  = 200; // 5 Hz
unsigned long lastPingMs = 0;

// ========= BLE =========
BLEAdvertisedDevice*     g_found    = nullptr;
BLEClient*               g_client   = nullptr;
BLERemoteCharacteristic* g_uartChar = nullptr;

// ===== Helpers =====
float clampf(float v, float a, float b){ return v < a ? a : (v > b ? b : v); }
int med3(int a, int b, int c) { if (a>b) std::swap(a,b); if (b>c) std::swap(b,c); if (a>b) std::swap(a,b); return b; }
int readADCStable(int pin) {
  long acc = 0;
  for (int i=0;i<SAMPLES/2;i++) {
    int a = analogRead(pin);
    int b = analogRead(pin);
    int c = analogRead(pin);
    acc += med3(a,b,c);
  }
  return (int)(acc / (SAMPLES/2));
}

class AdvCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveName() && advertisedDevice.getName() == TARGET_NAME) {
      if (g_found) delete g_found;
      g_found = new BLEAdvertisedDevice(advertisedDevice);
      BLEDevice::getScan()->stop();
      Serial.println("[BLE] Zielgeraet gefunden, Scan gestoppt.");
    }
  }
};

bool bleConnect() {
  if (!g_found) return false;
  if (!g_client) g_client = BLEDevice::createClient();

  Serial.println("[BLE] Verbinde...");
  if (!g_client->connect(g_found)) { Serial.println("[BLE] Connect fehlgeschlagen"); return false; }

  // MTU höher anfragen (optional)
  g_client->setMTU(185);

  BLERemoteService* svc = g_client->getService(UART_SERVICE);
  if (!svc) { Serial.println("[BLE] Service nicht gefunden"); return false; }

  g_uartChar = svc->getCharacteristic(UART_CHAR);
  if (!g_uartChar) { Serial.println("[BLE] Characteristic nicht gefunden"); return false; }

  if (!g_uartChar->canWrite() && !g_uartChar->canWriteNoResponse()) {
    Serial.println("[BLE] Characteristic nicht schreibbar");
    return false;
  }

  Serial.println("[BLE] UART bereit.");
  return true;
}

void bleSendLine(const String& s) {
  if (!g_uartChar || !g_client || !g_client->isConnected()) return;
  std::string msg = s.c_str();
  // Write Without Response bevorzugen, wenn möglich
  if (g_uartChar->canWriteNoResponse()) {
    g_uartChar->writeValue((uint8_t*)msg.data(), msg.size(), false);
  } else {
    g_uartChar->writeValue((uint8_t*)msg.data(), msg.size(), true);
  }
}

void calibrateCenter() {
  Serial.print("[CAL] Mitte messen ... ");
  unsigned long t0 = millis();
  long sx=0, sy=0; int n=0;
  while (millis() - t0 < 200) {
    sx += readADCStable(JOY_X_PIN);
    sy += readADCStable(JOY_Y_PIN);
    n++;
  }
  adcMidX = sx / n; adcMidY = sy / n;
  Serial.print("X="); Serial.print(adcMidX);
  Serial.print(" Y="); Serial.println(adcMidY);
}

void handleSW() {
  unsigned long now = millis();
  bool sw = digitalRead(JOY_SW_PIN); // LOW=gedrückt, HIGH=losgelassen

  // Long-Hold: 2s gedrückt => Kalibrieren
  static bool holdArmed = false;
  static unsigned long holdStart = 0;
  if (sw == LOW && swPrev == HIGH && (now - lastSwChangeMs) > DEBOUNCE_MS) {
    holdStart = now; holdArmed = true;
  }
  if (holdArmed && sw == LOW && (now - holdStart) > HOLD_CAL_MS) {
    holdArmed = false;
    calibrateCenter();
    clickCount = 0; lastClickMs = now;
    Serial.println("[SW] HOLD -> Recalibrate center done");
  }
  if (sw == HIGH) holdArmed = false;

  // Entprellen + Release-Klick zählen
  if (sw != swPrev && (now - lastSwChangeMs) > DEBOUNCE_MS) {
    lastSwChangeMs = now; swPrev = sw;
    if (sw == HIGH) { // RELEASE
      unsigned long delta = now - lastClickMs;
      if (delta <= MULTI_CLICK_MS) clickCount++;
      else                         clickCount = 1;
      lastClickMs = now;

      Serial.print("[SW] Release, clicks="); Serial.println(clickCount);

      // Triple sofort toggeln
      if (clickCount >= 3) {
        tiltAdjustMode = !tiltAdjustMode;
        Serial.print("[SW] TRIPLE -> TiltAdjust ");
        Serial.println(tiltAdjustMode ? "ON" : "OFF");
        clickCount = 0;
      }
    }
  }

  // Fenster abgelaufen: auswerten
  if (clickCount > 0 && (now - lastClickMs) > MULTI_CLICK_MS) {
    if (clickCount == 2) {
      panTarget  = 90.0f;
      tiltTarget = 90.0f;
      bleSendLine("RST\n");
      Serial.println("[SW] DOUBLE -> RST");
    } else if (clickCount == 1 && tiltAdjustMode) {
      tiltAdjustMode = false;
      char buf[24]; snprintf(buf, sizeof(buf), "TT:%d\n", (int)round(tiltTarget));
      bleSendLine(buf);
      Serial.print("[SW] SINGLE -> Tilt fix @ "); Serial.println(tiltTarget,1);
    }
    clickCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(JOY_X_PIN, ADC_11db);
  analogSetPinAttenuation(JOY_Y_PIN, ADC_11db);

  calibrateCenter();

  BLEDevice::init("ESP32-BLE-Client");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCallbacks());
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);
  Serial.println("[BLE] Scanne nach Zielgeraet...");
  scan->start(10, false);
}

void loop() {
  unsigned long now = millis();

  // (Re)Connect
  if (!g_client || !g_client->isConnected()) {
    if (g_found && bleConnect()) {
      Serial.println("[BLE] Ready.");
    } else {
      BLEDevice::getScan()->start(5, false);
    }
  }

  // Joystick einlesen (stabilisiert)
  int rawX = readADCStable(JOY_X_PIN);
  int rawY = readADCStable(JOY_Y_PIN);
  int dx = rawX - adcMidX;
  int dy = rawY - adcMidY;

  if (abs(dx) < JOY_DEAD) dx = 0;
  if (abs(dy) < JOY_DEAD) dy = 0;

  const float JOY_RANGE = 2048.0f;
  float panCmd  = -(float)dx / JOY_RANGE; // links = links
  float tiltCmd =  (float)dy / JOY_RANGE;

  static float panFilt = 0.0f, tiltFilt = 0.0f;
  panFilt  += EMA_ALPHA * (panCmd  - panFilt);
  tiltFilt += EMA_ALPHA * (tiltCmd - tiltFilt);

  if (!tiltAdjustMode) tiltFilt = 0.0f;

  static unsigned long lastMs = now;
  float dt = (now - lastMs) / 1000.0f;
  if (dt > 0.03f) dt = 0.03f;
  lastMs = now;

  if (fabs(panFilt) > 0.001f) {
    panTarget += panFilt * panSpeedDegPerSec * dt;
  } else if (PAN_RETURN_TO_MID) {
    panTarget = 90.0f;
  }
  if (fabs(tiltFilt) > 0.001f && tiltAdjustMode) {
    tiltTarget += tiltFilt * tiltSpeedDegPerSec * dt;
  }

  panTarget  = clampf(panTarget,  PAN_MIN,  PAN_MAX);
  tiltTarget = clampf(tiltTarget, TILT_MIN, TILT_MAX);

  // Heartbeat (weckt manche Module)
  if (g_client && g_client->isConnected() && now - lastPingMs > 1000) {
    bleSendLine("PING\n");
    lastPingMs = now;
  }

  // senden (nur bei Änderung, getaktet)
  if (g_client && g_client->isConnected() && (now - lastSendMs >= SEND_MS)) {
    static int lastP = -999, lastT = -999;
    int p = (int)round(panTarget);
    int t = (int)round(tiltTarget);

    if (!tiltAdjustMode && p != lastP) {
      char buf[24]; snprintf(buf, sizeof(buf), "PT:%d\n", p);
      bleSendLine(buf); lastP = p;
    }
    if (t != lastT) {
      char buf[24]; snprintf(buf, sizeof(buf), "TT:%d\n", t);
      bleSendLine(buf); lastT = t;
    }
    lastSendMs = now;
  }

  // Taster-Logik
  handleSW();

  // Debug
  if (g_client && g_client->isConnected() && (now - lastDbgMs > DBG_MS)) {
    lastDbgMs = now;
    Serial.print("[DBG] rawX="); Serial.print(rawX);
    Serial.print(" rawY="); Serial.print(rawY);
    Serial.print(" panTarget="); Serial.print(panTarget,1);
    Serial.print(" tiltTarget="); Serial.print(tiltTarget,1);
    Serial.print(" mode="); Serial.println(tiltAdjustMode ? "ON" : "OFF");
  }
}
