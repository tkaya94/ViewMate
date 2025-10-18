#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

/* ===== Ziel & UUIDs (HM-10/BT05 UART) ===== */
static const char* TARGET_NAME = "HC-05";
static BLEUUID UART_SERVICE("0000FFE0-0000-1000-8000-00805F9B34FB");
static BLEUUID UART_CHAR   ("0000FFE1-0000-1000-8000-00805F9B34FB");

/* ===== Joystick-Pins ===== */
const int JOY_X_PIN = 34;  // Pan Auswahl: links/mitte/rechts
const int JOY_Y_PIN = 35;  // Tilt stufenlos (nur im Tilt-Mode)
const int JOY_SW_PIN = 27; // Multi-Click

/* ===== Presets (iPhone-13-Pro FOV berücksichtigt) ===== */
const int PAN_LEFT  = 30;
const int PAN_MID   = 90;
const int PAN_RIGHT = 150;

const int TILT_MIN  = 40;
const int TILT_MAX  = 80;

/* ===== Tuning ===== */
// Stick/Filter
int   JOY_DEAD   = 160;     // Deadzone (ADC, 0..4095)
const int SAMPLES = 6;      // ADC-Mittelung
float EMA_ALPHA   = 0.28f;  // 0.25–0.35: höher = direkter

// Tilt-Geschwindigkeit
float tiltSpeedDegPerSec = 30.0f;

// Sende-/Steuerfrequenzen
const unsigned long CTRL_DT_MS   = 10;  // 100 Hz Integrationsloop
const unsigned long SEND_TILT_MS = 16;  // ~62 Hz Tilt-Sendeloop (nur wenn Mode aktiv)

/* ===== Multi-Click ===== */
const unsigned long DEBOUNCE_MS    = 25;
const unsigned long MULTI_CLICK_MS = 1200;
const unsigned long HOLD_CAL_MS    = 2000;
bool swPrev = HIGH; unsigned long lastSwChangeMs=0, lastClickMs=0; int clickCount=0;
bool tiltAdjustMode=false;

/* ===== ADC-Mitte (Kalibrierung) ===== */
int adcMidX = 2048, adcMidY = 2048;

/* ===== Zeitmarken ===== */
unsigned long lastCtrlMs=0, lastSendTiltMs=0, lastDbgMs=0;
const unsigned long DBG_MS=500;

/* ===== BLE ===== */
BLEAdvertisedDevice* g_found=nullptr;
BLEClient* g_client=nullptr;
BLERemoteCharacteristic* g_uartChar=nullptr;

/* ===== Utils ===== */
float clampf(float v,float a,float b){ return v<a?a:(v>b?b:v); }
int med3(int a,int b,int c){ if(a>b) std::swap(a,b); if(b>c) std::swap(b,c); if(a>b) std::swap(a,b); return b; }
int readADCStable(int pin){
  long acc=0; for(int i=0;i<SAMPLES;i++){ int a=analogRead(pin),b=analogRead(pin),c=analogRead(pin); acc+=med3(a,b,c); }
  return (int)(acc/SAMPLES);
}
inline bool linkUp(){ return g_client && g_client->isConnected() && g_uartChar; }

class AdvCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    if (d.haveName() && d.getName()==TARGET_NAME) {
      if (g_found) delete g_found; g_found=new BLEAdvertisedDevice(d);
      BLEDevice::getScan()->stop();
      Serial.println("[BLE] Ziel gefunden, Scan gestoppt.");
    }
  }
};

bool bleConnect(){
  if(!g_found) return false;
  if(!g_client) g_client = BLEDevice::createClient();
  Serial.println("[BLE] Verbinde...");
  if(!g_client->connect(g_found)){ Serial.println("[BLE] Connect fail"); return false; }
  BLERemoteService* svc = g_client->getService(UART_SERVICE);
  if(!svc){ Serial.println("[BLE] Service fehlt"); return false; }
  g_uartChar = svc->getCharacteristic(UART_CHAR);
  if(!g_uartChar){ Serial.println("[BLE] Char fehlt"); return false; }
  if(!g_uartChar->canWrite() && !g_uartChar->canWriteNoResponse()){
    Serial.println("[BLE] Char nicht schreibbar"); return false;
  }
  Serial.println("[BLE] Verbunden & bereit.");
  return true;
}
void bleSendLine(const String& s){
  if(!linkUp()) return;
  String out=s; if(out.length()==0 || out[out.length()-1]!='\n') out+='\n';
  // HM-10/BT05 bleibt oft bei 20 Byte MTU → kurze Frames
  if (g_uartChar->canWriteNoResponse()) g_uartChar->writeValue((uint8_t*)out.c_str(), out.length(), false);
  else                                   g_uartChar->writeValue((uint8_t*)out.c_str(), out.length(), true);
}

void calibrateCenter(){
  Serial.print("[CAL] Mitte...");
  unsigned long t0=millis(); long sx=0, sy=0; int n=0;
  while(millis()-t0<250){ sx+=readADCStable(JOY_X_PIN); sy+=readADCStable(JOY_Y_PIN); n++; }
  adcMidX=sx/n; adcMidY=sy/n;
  Serial.print(" X="); Serial.print(adcMidX);
  Serial.print(" Y="); Serial.println(adcMidY);
}

void handleSW(){
  unsigned long now=millis();
  bool sw = digitalRead(JOY_SW_PIN);

  // Hold→Kalibrierung
  static bool holdArmed=false; static unsigned long holdStart=0;
  if (sw==LOW && swPrev==HIGH && (now-lastSwChangeMs)>DEBOUNCE_MS) { holdArmed=true; holdStart=now; }
  if (holdArmed && sw==LOW && (now-holdStart)>HOLD_CAL_MS) { holdArmed=false; calibrateCenter(); clickCount=0; lastClickMs=now; }
  if (sw==HIGH) holdArmed=false;

  // Click zählen (auf RELEASE)
  if (sw!=swPrev && (now-lastSwChangeMs)>DEBOUNCE_MS) {
    lastSwChangeMs=now; swPrev=sw;
    if (sw==HIGH) { if (now-lastClickMs<=MULTI_CLICK_MS) clickCount++; else clickCount=1; lastClickMs=now; }
  }
  if (clickCount>0 && (now-lastClickMs)>MULTI_CLICK_MS) {
    if (clickCount==2) {
      tiltAdjustMode=!tiltAdjustMode;
      bleSendLine(tiltAdjustMode ? "TA:ON" : "TA:OFF");
      Serial.print("[TA] "); Serial.println(tiltAdjustMode?"ON":"OFF");
    } else if (clickCount==1 && tiltAdjustMode) {
      tiltAdjustMode=false; bleSendLine("BTN:1"); // confirm
      Serial.println("[TA] OFF (confirm)");
    } else if (clickCount>=3) {
      bleSendLine("BTN:3"); // Reset
      Serial.println("[RST] triple");
    }
    clickCount=0;
  }
}

enum PanState { P_LEFT, P_MID, P_RIGHT };

void setup(){
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
  Serial.println("[BLE] Scan...");
  scan->start(10, false);

  lastCtrlMs = lastSendTiltMs = millis();
}

void loop(){
  unsigned long now=millis();

  // (Re)connect
  if (!linkUp()) {
    if (g_found && bleConnect()) {
      Serial.println("[BLE] READY");
    } else {
      BLEDevice::getScan()->start(5, false);
    }
  }

  // ===== 100 Hz: Joystick lesen & Logik =====
  if (now - lastCtrlMs >= CTRL_DT_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f; if (dt>0.05f) dt=0.05f;
    lastCtrlMs = now;

    int rawX = readADCStable(JOY_X_PIN);
    int rawY = readADCStable(JOY_Y_PIN);
    int dx = rawX - adcMidX;
    int dy = rawY - adcMidY;

    // --- PAN: 3 feste Zustände mit Hysterese ---
    static PanState panState = P_MID;
    static const int HYST = 60;  // zusätzliche Hysterese gegen Flattern

    if (dx < -(JOY_DEAD + HYST)) {
      if (panState != P_LEFT) { panState = P_LEFT;  if (linkUp()) bleSendLine(String("PT:") + PAN_LEFT); }
    } else if (dx >  (JOY_DEAD + HYST)) {
      if (panState != P_RIGHT){ panState = P_RIGHT; if (linkUp()) bleSendLine(String("PT:") + PAN_RIGHT); }
    } else if (abs(dx) <= JOY_DEAD) {
      if (panState != P_MID)  { panState = P_MID;   if (linkUp()) bleSendLine(String("PT:") + PAN_MID); }
    }
    // Hinweis: Wenn Tilt-Adjust aktiv ist, darf Empfänger Pan ignorieren – die Regel bleibt dort.

    // --- TILT: stufenlos nur im Tilt-Adjust-Mode ---
    if (tiltAdjustMode) {
      if (abs(dy) < JOY_DEAD) dy = 0;
      const float JOY_RANGE = 2048.0f;
      float tiltCmd = (float)dy / JOY_RANGE;

      static float tiltFilt = 0.0f;
      tiltFilt += EMA_ALPHA * (tiltCmd - tiltFilt);

      if (fabs(tiltFilt) > 0.001f) {
        static float tiltTarget = 90.0f; // nur Sender-seitig für Fluss
        tiltTarget += tiltFilt * tiltSpeedDegPerSec * dt;
        tiltTarget = clampf(tiltTarget, TILT_MIN, TILT_MAX);

        // ~62 Hz senden (separater Takt)
        if (now - lastSendTiltMs >= SEND_TILT_MS && linkUp()) {
          lastSendTiltMs = now;
          char buf[16]; snprintf(buf, sizeof(buf), "TT:%d", (int)round(tiltTarget));
          bleSendLine(buf);
        }
      }
    }
  }

  // Tasten-Logik
  handleSW();

  // Debug (optional)
  if (now - lastDbgMs > DBG_MS) {
    lastDbgMs = now;
    Serial.print("[DBG] mode="); Serial.print(tiltAdjustMode?"TA:ON":"TA:OFF");
    Serial.print("  dead="); Serial.print(JOY_DEAD);
    Serial.print("  alpha="); Serial.println(EMA_ALPHA, 2);
  }
}
