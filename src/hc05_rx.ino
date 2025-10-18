#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(11, 10);     // HC_TXD->10, HC_RXD<-11 (mit Teiler)
#define BT_BAUD 9600           // bei Bedarf 38400 oder 115200 testen

const int PIN_PAN  = 13;
const int PIN_TILT = 12;
Servo pan, tilt;

const int PAN_MIN=30, PAN_MAX=150, TILT_MIN=40, TILT_MAX=80;
const float PAN_SPEED=60.0f, TILT_SPEED=20.0f, DEADZONE=0.5f;

float panPos=90, tiltPos=90, panTarget=90, tiltTarget=90;
unsigned long lastPacketMs=0, lastMs=0;
const unsigned long LINK_TIMEOUT_MS=1500;

String inLine;

inline float clampf(float v,float a,float b){return v<a?a:(v>b?b:v);}
inline void moveToward(float &p,float t,float s){float d=t-p; if(fabs(d)<=DEADZONE)p=t; else p+=(d>0?+s:-s);}
void apply(){ pan.write((int)round(panPos)); tilt.write((int)round(tiltPos)); }

void handleCommand(String s){
  s.trim();
  if(s.length()==0) return;
  if(s.startsWith("PT:")){ panTarget=clampf(s.substring(3).toFloat(),PAN_MIN,PAN_MAX); lastPacketMs=millis(); }
  else if(s.startsWith("TT:")){ tiltTarget=clampf(s.substring(3).toFloat(),TILT_MIN,TILT_MAX); lastPacketMs=millis(); }
  else if(s.startsWith("RST")){ panTarget=90; tiltTarget=90; lastPacketMs=millis(); }
  Serial.print(F("[CMD] ")); Serial.println(s);
}

void setup(){
  Serial.begin(115200);
  BT.begin(BT_BAUD);
  pan.attach(PIN_PAN);
  tilt.attach(PIN_TILT);
  pan.write(90); tilt.write(90);
  lastPacketMs = millis();
  lastMs = millis();
  Serial.print(F("Servo-Empfaenger bereit @")); Serial.println(BT_BAUD);
}

void loop(){
  // UART lesen, Zeilen bilden (LF/CR/CRLF)
  while(BT.available()){
    char c = BT.read();
    if(c=='\n' || c=='\r'){
      if(inLine.length()>0){ handleCommand(inLine); inLine=""; }
    }else{
      inLine += c;
      if(inLine.length()>64) inLine="";
    }
  }

  // Timeout: Pan zur Mitte, Tilt halten
  if(millis()-lastPacketMs>LINK_TIMEOUT_MS) panTarget=90;

  // Bewegung ~50 Hz
  unsigned long now = millis();
  float dt = (now-lastMs)/1000.0f; if(dt>0.05f) dt=0.05f; lastMs=now;
  moveToward(panPos, panTarget, PAN_SPEED*dt);
  moveToward(tiltPos, tiltTarget, TILT_SPEED*dt);
  apply();

  // Debug
  static unsigned long dbg=0;
  if(now-dbg>1000){
    dbg=now;
    Serial.print(F("[STAT] Pan="));Serial.print(panPos,1);
    Serial.print(F(" Tilt="));Serial.print(tiltPos,1);
    Serial.print(F(" Link="));Serial.println((millis()-lastPacketMs)<=LINK_TIMEOUT_MS?F("OK"):F("TIMEOUT"));
  }
}
