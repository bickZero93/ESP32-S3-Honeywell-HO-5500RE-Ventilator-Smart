// Smart-Ventilator / ESP32-S3-Zero
// Public GitHub build v6.4.10 - Zugangsdaten vor dem Flashen anpassen
//
// Hardware:
// GPIO6 -> 2N7000 Gate -> Taste 1 POWER    | GPIO7  <- LED Speed 1
// GPIO5 -> 2N7000 Gate -> Taste 2 SPEED    | GPIO8  <- LED Speed 2
// GPIO4 -> 2N7000 Gate -> Taste 3 OSC      | GPIO9  <- LED Speed 3
// GPIO3 -> 2N7000 Gate -> Taste 5 MODE     | GPIO10 <- LED Breeze
//                                          | GPIO1  <- LED Nacht
// Taste 4 (Original-Timer) wird NICHT benutzt.
//
// Reale Logik:
// POWER/0: normal AUS <-> AN behaelt Speed/Modus/Drehen der Fan-Platine.
// Nur nach kompletter Netztrennung startet das naechste EIN auf Speed1 / Normal / Drehen AUS.
// SPEED: 1 -> 2 -> 3 -> 1
// OSC: Toggle
// MODE: Normal -> Breeze -> Night -> Normal
//
// ACHTUNG:
// Wenn der ESP32 aus der Ventilatorplatine versorgt wird,
// USB NICHT anschliessen solange der Ventilator am Netz haengt.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <stdarg.h>   // fuer anaSetAction (va_list)
#include <esp_system.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* HOSTNAME  = "ventilator";
const char* FW_VERSION = "6.4.10-FINAL-UI-MANUAL-OSC";

// Fallback-AP, damit du dich auch bei WLAN-Problemen noch verbinden kannst.
// Wird NUR gestartet, wenn die WLAN-Verbindung fehlschlaegt (siehe startNetwork).
const char* AP_SSID = "Ventilator-Setup";
const char* AP_PASS = "CHANGE_ME_AP";   // >= 8 Zeichen, sonst bleibt der AP offen!

// Zugangsdaten fuer /update. Ohne diese koennte jeder im Netz beliebige
// Firmware aufspielen - inklusive jedem, der sich auf den Fallback-AP verbindet.
const char* OTA_USER = "admin";
const char* OTA_PASS = "CHANGE_ME_OTA";

// Belegung von unten nach oben, Taste und zugehoerige LED liegen sich
// auf dem ESP32-S3-Zero direkt gegenueber:
//   GP6 T1 Power   <->  GP7  LED Speed 1
//   GP5 T2 Speed   <->  GP8  LED Speed 2
//   GP4 T3 Drehen  <->  GP9  LED Speed 3
//   GP3 T5 Modus   <->  GP10 LED Breeze
//                       GP1  LED Nacht (links oben, ADC1 reicht nur bis GP10)
constexpr uint8_t PIN_POWER = 6;
constexpr uint8_t PIN_SPEED = 5;
constexpr uint8_t PIN_OSC   = 4;
constexpr uint8_t PIN_MODE  = 3;   // Strapping-Pin: 100k von Gate nach GND vorsehen

constexpr uint16_t PRESS_MS = 180;
constexpr uint16_t GAP_MS   = 260;

#define ENABLE_LED_FEEDBACK 1
#if ENABLE_LED_FEEDBACK
// Alle Feedback-Pins MUESSEN auf ADC1 liegen (GP1..GP10). ADC2 (ab GP11)
// ist bei aktivem WLAN vom Funkteil belegt und liefert keine Messwerte.
constexpr uint8_t PIN_LED1_FB = 7;   // LED1 ANODE / Speed 1
constexpr uint8_t PIN_LED2_FB = 8;   // LED2 ANODE / Speed 2
constexpr uint8_t PIN_LED3_FB = 9;   // LED3 ANODE / Speed 3
constexpr uint8_t PIN_LED8_FB = 10;  // Breeze
constexpr uint8_t PIN_LED9_FB = 1;   // Night
#endif

WebServer server(80);
Preferences prefs;

// Individuelle Schwellwerte fuer die 5 genutzten Feedback-LEDs.
// Reihenfolge: LED1, LED2, LED3, LED8, LED9
constexpr int FB_DEFAULT_THR[5] = {2368,3072,2432,3200,2752};
int fbThresholds[5] = {2368,3072,2432,3200,2752};

// Finale phasenunabhaengige ADC-Erfassung.
// Der alte feste Takt konnte sich mit dem LED-Multiplex synchronisieren.
// JITTER_V6 variiert Startphase, Kanalstart, Richtung, Zyklusabstand und Serienzeit.
// Die Zufallsfolge ist absichtlich nur pseudozufaellig: fuer Phasenentkopplung
// reicht das vollstaendig und die ADC-Messung bleibt frei von RNG-Nebenwirkungen.
constexpr uint8_t CAL_SAMPLER_VERSION = 6;
constexpr uint16_t PROFILE_SAMPLE_CYCLES = 480;
constexpr uint16_t PROFILE_JITTER_MIN_US = 80;
constexpr uint16_t PROFILE_JITTER_MAX_US = 420;

// FINAL_EFFICIENT_BALANCED_V6:
// Zehn feste, histogrammkompatible CDF-Schwellen pro Kanal. Ein einzelner
// Schwellwert bei ~3.1 V war in der realen Messung zu grob: mehrere 18P-
// Zustaende lagen trotz stabiler Einzelprofile fast uebereinander.
// Die 10 CDF-Stuetzstellen erfassen nun die VerteilungsFORM jedes LED-Signals.
constexpr uint8_t RICH_CDF_LEVELS = 10;
constexpr uint16_t RICH_CDF_ADC[RICH_CDF_LEVELS] = {
  1536,1792,2048,2304,2560,2816,3072,3328,3584,3840
};

// Mehrere kurze Jitter-Fenster werden absichtlich ueber einige Sekunden
// verteilt. Das mittelt Breeze/Night-Zyklen und mechanische Drehen-Effekte,
// statt zufaellig nur eine kurze Phase zu erwischen.
constexpr uint16_t PROFILE_SERIES_GAP_MIN_MS = 330;
constexpr uint16_t PROFILE_SERIES_GAP_MAX_MS = 620;

inline uint32_t profileRand32(uint32_t& x);

inline uint16_t profileSeriesGapMs(uint32_t& rng) {
  uint32_t r=profileRand32(rng);
  return PROFILE_SERIES_GAP_MIN_MS +
         (uint16_t)(r % (PROFILE_SERIES_GAP_MAX_MS-PROFILE_SERIES_GAP_MIN_MS+1));
}

inline uint32_t profileRand32(uint32_t& x) {
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  if(x==0)x=0xA341316Cu;
  return x;
}

uint32_t profileSeedCounter=0x6D2B79F5u;

inline uint32_t profileNewSeed(uint32_t salt=0) {
  // Kein Hardware-RNG waehrend der ADC-Erfassung: wir brauchen keine
  // kryptographische Zufallsquelle, sondern nur immer neue Phasenlagen.
  profileSeedCounter += 0x9E3779B9u;
  uint32_t s=profileSeedCounter ^ (uint32_t)micros() ^
             ((uint32_t)millis()<<11) ^ salt;
  return profileRand32(s);
}

inline uint16_t profileJitterUs(uint32_t& rng) {
  uint32_t r=profileRand32(rng);
  return PROFILE_JITTER_MIN_US +
         (uint16_t)(r % (PROFILE_JITTER_MAX_US-PROFILE_JITTER_MIN_US+1));
}



struct FanState {
  uint8_t speed = 0;        // aktueller Zustand: 0=Aus, 1..3
  bool osc = false;
  uint8_t mode = 0;         // 0=Normal,1=Breeze,2=Night
  uint16_t sleepMin = 0;

  // Wird beim Ausschalten NICHT vergessen:
  uint8_t lastSpeed = 1;
  bool lastOsc = false;
  uint8_t lastMode = 0;
} state;

// Custom types bewusst sehr frueh deklarieren.
// Arduino erzeugt bei .ino-Dateien automatisch Funktionsprototypen; stehen
// solche Typen erst spaeter, kann daraus "'FbStats' does not name a type" werden.
struct FbStats {
  int minv;
  int maxv;
  int avg;
  int rangev;
  float abovePct;
  float belowPct;
  float cdfPct[RICH_CDF_LEVELS]; // Anteil >= RICH_CDF_ADC[q]
};

struct LearnedProfile {
  float avgPct[5];
  float avgMean[5];
  bool valid;
};

// -----------------------------------------------------------------
// ARDUINO-.INO-AUTOPROTOTYPE-FIREWALL
// Alle Funktionen mit eigenen Typen werden hier explizit deklariert.
// Dadurch muss der Arduino-Preprocessor fuer diese Signaturen KEINE
// automatischen Prototypen vor FbStats/LearnedProfile erzeugen.
// -----------------------------------------------------------------
inline void normalizedMeanFeature(const float meanv[5],float out[5]);
inline void normalizedMeanFeature(const FbStats cur[5],float out[5]);
inline float rawLevelFeature(const float meanv[5]);
inline float rawLevelFeature(const FbStats cur[5]);

bool addDiagnosticSampleAndAverage(const FbStats sample[5],
                                   FbStats avg[5],
                                   uint8_t minWindows);
void sampleProfileWindow(FbStats cur[5]);
void sampleProfileAverage(FbStats cur[5],uint8_t windows,uint16_t pauseMs);

bool learnProfile(LearnedProfile& p);
float currentNormalizedMean(const FbStats current[5],int ch);
float signatureDistanceWeighted(const LearnedProfile& sig,
                                const FbStats current[5],
                                const float weights[5]);
int detectProfilesWeightedFromCurrent(LearnedProfile profiles[3],
                                      const FbStats cur[5],
                                      const float weights[5],
                                      float* confidenceOut);
int detectProfilesFromCurrent(LearnedProfile profiles[3],
                              const FbStats cur[5],
                              float* confidenceOut);
int detectProfiles(LearnedProfile profiles[3],float* confidenceOut);
int detectCombo(const FbStats cur[5],
                int* speedOut,int* modeOut,
                float* speedConfOut,float* modeConfOut);
int detectComboOsc(const FbStats cur[5],
                   int* speedOut,int* modeOut,int* oscOut,
                   float* speedConfOut,float* modeConfOut,float* oscConfOut);
float comboDistanceToProfile(const LearnedProfile& p,const FbStats cur[5]);
float comboRichDistanceToState(int sp,int md,int os,const FbStats cur[5]);
float comboOscLinearScore(int sp,int md,const FbStats cur[5]);
int comboOscLinearDetect(int sp,int md,const FbStats cur[5],float* confidenceOut);
float comboPctShapeDistanceToProfile(const LearnedProfile& p,const FbStats cur[5]);
float comboPctShapeDistanceToSpeedMode(int sp,int md,const FbStats cur[5]);
int detectSpeedPctShapeForMode(int md,const FbStats cur[5],float* confidenceOut,float* fitOut);
int detectModePctShapeForSpeed(int sp,const FbStats cur[5],float* confidenceOut,float* fitOut);
void manualSyncAccumulateBatch(const FbStats cur[5],float pairDist[3][3],uint8_t speedVotes[3][3],uint8_t oscOnVotes[3][3],uint8_t oscOffVotes[3][3],float oscScoreSum[3][3]);
void detectContextual18(const FbStats cur[5],
                        int* sp,float* sc,int* md,float* mc,int* os,float* oc);
bool ledsLookOff(const FbStats cur[5]);
int anaClassifyStats(const FbStats cur[5],float* marginOut,float* fitOut);
bool anaSampleProfileAverageResponsive(FbStats cur[5],uint8_t windows,uint32_t salt);
void anaProfileRawMean(uint8_t c,float meanv[5]);

bool anaResponsiveDelay(unsigned long ms);
void anaServiceIo();
bool anaMoveToState(uint8_t sp,uint8_t md,bool os);

unsigned long timerDeadlineMs = 0;

uint32_t stateRevision=1;
uint32_t bootSessionId=0;

inline void bumpStateRevision() {
  stateRevision++;
  if(stateRevision==0)stateRevision=1;
}

uint32_t timerRemainingSec() {
  if(!timerDeadlineMs)return 0;
  long remain=(long)(timerDeadlineMs-millis());
  return remain>0?(uint32_t)((remain+999L)/1000L):0;
}


unsigned long lastWifiRetryMs = 0;

// Aktueller Power-Zustand getrennt von der gemerkten Geschwindigkeit.
bool fanIsOn = false;

// Direkt nach Power-EIN ist LED-Feedback besonders wichtig, um den vom
// Ventilator selbst gespeicherten Zustand wieder zu erkennen.
unsigned long startupSyncUntilMs = 0;
unsigned long startupSyncReadyMs = 0;
uint8_t startupSpeedCandidate = 0;
uint8_t startupSpeedHits = 0;
int8_t startupModeCandidate = -1;
uint8_t startupModeHits = 0;

bool manualLedSyncRequested = false;
bool suppressFanStateSave=false;   // nur waehrend finaler Analyse
bool suppressCalibrationSave=false; // beim atomaren Modellaufbau nur einmal am Ende schreiben

// LED-Erkennung ist reine Diagnose, ausser beim bewusst manuellen Sync.
// Nach Power-EIN warten wir, bis die Multiplexsignale stabil sind.
unsigned long ledDetectReadyMs = 0;
constexpr unsigned long LED_DETECT_SETTLE_MS = 5000UL;

int diagSpeedCandidate = 0;
uint8_t diagSpeedHits = 0;
int diagModeCandidate = -1;
uint8_t diagModeHits = 0;
int stableDiagSpeed = 0;
int stableDiagMode = -1;
float stableDiagSpeedConf = 0.0f;
float stableDiagModeConf = 0.0f;

int diagOscCandidate = -1;
uint8_t diagOscHits = 0;
int stableDiagOsc = -1;
float stableDiagOscConf = 0.0f;

constexpr uint8_t DIAG_ROLLING_WINDOWS = 10;
FbStats diagWindow[DIAG_ROLLING_WINDOWS][5];
uint8_t diagWindowCount=0;
uint8_t diagWindowPos=0;

bool diagCacheValid=false;
bool diagCachePctValid=false;
bool diagCacheOff=false;
int diagCacheSpeed=0;
int diagCacheMode=-1;
int diagCacheOsc=-1;
float diagCacheSpeedConf=0.0f;
float diagCacheModeConf=0.0f;
float diagCacheOscConf=0.0f;
float diagContextFit=0.0f;
float diagWebFit=0.0f;

// Globaler, kontextfreier 18P-Treffer.
int diagGlobalSpeed=0;
int diagGlobalMode=-1;
int diagGlobalOsc=-1;
float diagGlobalSpeedConf=0.0f;
float diagGlobalModeConf=0.0f;
float diagGlobalOscConf=0.0f;
float diagGlobalFit=0.0f;

float diagCachePct[5]={0,0,0,0,0};
unsigned long diagLastSampleMs=0;
unsigned long diagNextSampleMs=0;
uint32_t diagSeriesRng=0xD1A64C5Fu;
unsigned long diagCacheAtMs=0;
bool startupScanDone = false;

bool otaInProgress=false;
bool otaUploadSucceeded=false;


void saveFanState();

void loadFanState() {
  prefs.begin("fanstate", true);

  uint8_t savedSpeed=prefs.getUChar("lastSpeed",1);
  bool savedOsc=prefs.getBool("lastOsc",false);
  uint8_t savedMode=prefs.getUChar("lastMode",0);
  bool savedPower=prefs.getBool("powerOn",false);

  prefs.end();

  if(savedSpeed<1||savedSpeed>3)savedSpeed=1;
  if(savedMode>2)savedMode=0;

  esp_reset_reason_t reason=esp_reset_reason();
  bool realPowerCycle=(reason==ESP_RST_POWERON)||(reason==ESP_RST_BROWNOUT);

  state.sleepMin=0;
  timerDeadlineMs=0;

  if(realPowerCycle) {
    // Bei echter Netztrennung setzt auch die Ventilatorplatine zurueck:
    // physisch AUS; der naechste Start ist Stufe1 / Normal / Drehen AUS.
    fanIsOn=false;
    state.speed=0;
    state.osc=false;
    state.mode=0;
    state.lastSpeed=1;
    state.lastOsc=false;
    state.lastMode=0;
    saveFanState();
    Serial.println("[BOOT] echter Power-Cycle -> physisch AUS, Basis St1/Normal/Drehen AUS");
  } else {
    // Bei OTA-/Software-Neustart bleibt die Ventilatorplatine in Betrieb.
    // Deshalb muss nicht nur der letzte Modus, sondern auch EIN/AUS erhalten
    // bleiben; sonst wuerde die Weboberflaeche einen laufenden Fan als AUS zeigen.
    state.lastSpeed=savedSpeed;
    state.lastOsc=savedOsc;
    state.lastMode=savedMode;
    fanIsOn=savedPower;

    if(fanIsOn) {
      state.speed=savedSpeed;
      state.osc=savedOsc;
      state.mode=savedMode;
      Serial.printf("[BOOT] Software-Neustart -> Zustand erhalten: Stufe %u / Modus %u / Drehen %s\n",
                    state.speed,state.mode,state.osc?"AN":"AUS");
    } else {
      state.speed=0;
      state.osc=false;
      state.mode=0;
    }
  }
}

void saveFanState() {
  prefs.begin("fanstate",false);
  prefs.putUChar("lastSpeed",state.lastSpeed);
  prefs.putBool("lastOsc",state.lastOsc);
  prefs.putUChar("lastMode",state.lastMode);
  prefs.putBool("powerOn",fanIsOn);
  prefs.end();
}


#if ENABLE_LED_FEEDBACK
const uint8_t FB_USED_PINS[5] = {
  PIN_LED1_FB, PIN_LED2_FB, PIN_LED3_FB, PIN_LED8_FB, PIN_LED9_FB
};

LearnedProfile speedSig[3];  // Speed 1..3
LearnedProfile modeSig[3];   // Normal, Breeze, Night
// AUS wird gemessen statt geschaetzt. Bei gemultiplexten LEDs liegt der
// Aus-Pegel nicht zwingend weit genug unter den Ein-Profilen, deshalb
// reicht eine feste Prozentgrenze nicht aus.
LearnedProfile offSig;

// Neun Profile: eine eigene Signatur je Stufe/Modus-Kombination.
// Grund: Die Messung zeigt, dass sich das Muster eines Modus staerker
// zwischen den Stufen verschiebt (3,3) als Breeze von Nacht abweicht (0,3).
// Ein ueber die Stufen gemitteltes Modus-Profil passt damit zu keinem
// tatsaechlichen Zustand. comboSig[speed-1][mode] loest das.
LearnedProfile comboSig[3][3];

// Finale 18 Profile: Speed (3) x Modus (3) x Drehen (2).
// Der integrierte Drehen-Test hat gezeigt: 7/9 Zustaende wurden mit
// Drehen AN durch die alte 9-Profil-Erkennung falsch klassifiziert,
// groesster Driftfaktor 6.10. Deshalb ist Oscillation jetzt eine echte
// dritte Klassifikationsdimension und kein externer Diagnosetest mehr.
LearnedProfile comboOscSig[3][3][2];

bool calibrationSamplerCurrent=false; // nur FINAL_EFFICIENT_BALANCED_V6-Profile duerfen als aktuelle Generation gelten
bool compactComboLoaded = false;
bool compactComboLastSaveOk = false;

bool comboOscComplete() {
  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++)
      for(int os=0; os<2; os++)
        if(!comboOscSig[sp][md][os].valid) return false;
  return true;
}

uint8_t comboOscValidCount() {
  uint8_t n=0;
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++)
      for(int os=0;os<2;os++)
        if(comboOscSig[sp][md][os].valid)n++;
  return n;
}

uint8_t comboValidCount() {
  uint8_t n=0;
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++)
      if(comboSig[sp][md].valid)n++;
  return n;
}

// Diese beiden Flags muessen vor activeRecognitionModel() sichtbar sein.
bool comboRichValid=false;
bool comboGlobalSyncSafe=false;

const char* activeRecognitionModel() {
  if(!calibrationSamplerCurrent) return "NEUKALIBRIERUNG";
  if(comboOscComplete() && comboRichValid) return "FINAL_18P";
  if(comboValidCount()==9) return "FALLBACK_9P";
  return "EINZELPROFILE";
}

// Aus der Messung abgeleitete Kanalgewichte. Statt fester Zahlen bekommt
// jeder Kanal das Gewicht, das seinem Signal-Rausch-Verhaeltnis entspricht:
// Spannweite zwischen den Zustaenden geteilt durch die Streuung innerhalb
// eines Zustands. Kanaele, die nichts beitragen, fallen so von selbst weg.
float comboW[5] = {1.0f,1.0f,1.0f,1.0f,1.0f};
bool  comboWValid = false;

// Formanteil der Distanzberechnung (0 = nur Absolutpegel, 1 = nur Form).
// Hintergrund: Breeze und Nacht liegen im Pegel praktisch gleichauf, LED8
// ist im Nacht-Modus aber konsistent niedriger und LED9 hoeher. Diese
// Formdifferenz ueberlebt, wenn man den Mittelwert ueber alle Kanaele
// abzieht - und weil das Rauschen gemultiplexter LEDs weitgehend
// gleichtaktig ist, faellt es dabei mit heraus.
// Der Wert wird im Messlauf ermittelt, nicht geraten.
float comboShape = 0.0f;

// Zweites, explizit mitkalibriertes Merkmal: relative ADC-Mittelwerte.
// Diese Information ist besonders fuer die historisch enge Stufe-1/Stufe-3-
// Trennung nuetzlich. Anders als in alten 6.0.x-Versionen wird ihr Anteil
// NICHT geraten, sondern per Cross-Validation optimiert.
float comboMeanW[5]={1.0f,1.0f,1.0f,1.0f,1.0f};
float comboMeanAlpha=0.0f;

// 50 CDF-Merkmale (5 Kanaele x 10 Pegel), kompakt quantisierte 18P-Zentren.
// uint16_t speichert 0.01 Prozentpunkte (0..10000) ohne relevante Quantisierungsverluste.
float comboRichW[5][RICH_CDF_LEVELS] = {};
uint16_t comboRichPctQ[3][3][2][5][RICH_CDF_LEVELS] = {};
float comboLevelAlpha=0.0f;

// Dedizierter Drehen-Klassifikator V1.
// Der reale 6.4.3-Lauf zeigte: Speed und Modus sind mit dem Rich-Modell
// eindeutig, waehrend Drehen in Breeze/Nacht nicht durch eine gemeinsame
// 18P-Metrik stabil genug ist. Deshalb wird Drehen bei bekanntem Modus
// mode-uebergreifend ueber alle drei Stufen als lineares CDF-Modell erkannt.
// Features: 5 Kanaele x 10 CDF-Pegel + 3 Speed-One-Hot = 53.
constexpr uint8_t OSC_LINEAR_FEATURES=53;
int16_t comboOscLinW[3][OSC_LINEAR_FEATURES] = {};
float comboOscLinScale[3]={1.0f,1.0f,1.0f};
float comboOscLinBias[3]={0.0f,0.0f,0.0f};
bool comboOscLinValid=false;

// Normales Tracking ist bewusst kontextuell: Webzustand = autoritativ.
// Blindes 18P wird separat bewertet und nur fuer den manuellen 3/3-Sync
// freigeschaltet, wenn sein eigener Holdout stark genug ist.

inline uint16_t richEncodePct(float v) {
  return (uint16_t)constrain((int)lroundf(constrain(v,0.0f,100.0f)*100.0f),0,10000);
}
inline float richDecodePct(uint16_t q) {
  return ((float)q)*0.01f;
}
inline float rawLevelFeature(const float meanv[5]) {
  float s=0.0f;
  for(int ch=0;ch<5;ch++)s+=constrain(meanv[ch],0.0f,4095.0f);
  return s*(100.0f/(5.0f*4095.0f));
}
inline float rawLevelFeature(const FbStats cur[5]) {
  float m[5];
  for(int ch=0;ch<5;ch++)m[ch]=(float)cur[ch].avg;
  return rawLevelFeature(m);
}

// Typischer maximaler Abstand eines korrekten Holdout-Treffers.
// Laufzeitmessungen weit ausserhalb dieses Bereichs werden als
// "ausserhalb Kalibrierung" erkannt statt mit hoher Scheinsicherheit.
float comboFitLimit=12.0f;

// Merkmalsvektor: f = pct - shape * Mittelwert(pct). shape=0 laesst den
// Absolutpegel unveraendert, shape=1 zentriert vollstaendig auf die Form.
inline void comboFeature(const float pct[5], float shape, float out[5]) {
  float m = 0.0f;
  for(int i=0;i<5;i++) m += pct[i];
  m /= 5.0f;
  for(int i=0;i<5;i++) out[i] = pct[i] - shape * m;
}


inline void normalizedMeanFeature(const float meanv[5],float out[5]) {
  float sum=0.0f;
  for(int ch=0;ch<5;ch++)sum+=max(meanv[ch],0.0f);
  if(sum<1.0f)sum=1.0f;
  for(int ch=0;ch<5;ch++)out[ch]=meanv[ch]*100.0f/sum;
}

inline void normalizedMeanFeature(const FbStats cur[5],float out[5]) {
  float m[5];
  for(int ch=0;ch<5;ch++)m[ch]=(float)cur[ch].avg;
  normalizedMeanFeature(m,out);
}



// Vorwaertsdeklaration bewusst vor allen Kalibrierungs-Save-Helfern.
bool saveCompactComboCalibration(bool reclaimLegacy=true);

void loadFeedbackThresholds() {
  prefs.begin("fanfb", true);
  for(int i=0;i<5;i++) {
    String key = "th" + String(i);
    fbThresholds[i] = prefs.getInt(key.c_str(), FB_DEFAULT_THR[i]);
  }
  prefs.end();
}

void saveFeedbackThreshold(int index,int value) {
  if(index<0||index>=5)return;
  fbThresholds[index]=constrain(value,0,4095);

  if(!suppressCalibrationSave)
    saveCompactComboCalibration(false);
}
#endif

// Explizite Deklarationen verhindern Probleme mit der Arduino-.ino-Autoprototypisierung.
bool learnProfile(LearnedProfile& p);
bool learnSpeedSignature(int speedNo);

// true, solange ein manueller Lernvorgang laeuft. Verhindert, dass Analyse und
// Einzel-Lernen gleichzeitig den Ventilator steuern und messen.
bool learnBusy = false;
bool learnModeSignature(int modeNo);
bool loadCompactComboCalibration();
bool saveCompactComboCalibration(bool reclaimLegacy);
void clearCompactComboCalibration();
void resetStableDiagnostic();
void armLedDiagnostic(unsigned long settleMs);
void resetDiagnosticWindow();
bool addDiagnosticSampleAndAverage(const FbStats sample[5], FbStats avg[5], uint8_t minWindows);
void sampleProfileWindow(FbStats cur[5]);
void sampleProfileAverage(FbStats cur[5], uint8_t windows, uint16_t pauseMs);
void updateLedDiagnosticTask();
void invalidateRuntimeCalibrationForSampler();
bool anaResponsiveDelay(unsigned long ms);
bool anaNormalizePhysicalBaseline(String& err);
void detectContextual18(const FbStats cur[5], int* sp,float* sc,int* md,float* mc,int* os,float* oc);
float signatureDistance5(const LearnedProfile& sig, const FbStats current[5]);
int detectProfiles(LearnedProfile profiles[3], float* confidenceOut);
int detectProfilesFromCurrent(LearnedProfile profiles[3], const FbStats cur[5], float* confidenceOut);
float normalizedMean(const float means[5], int ch);
float currentNormalizedMean(const FbStats current[5], int ch);
float signatureDistanceWeighted(const LearnedProfile& sig, const FbStats current[5], const float weights[5]);
float comboDistanceToProfile(const LearnedProfile& p,const FbStats cur[5]);
float comboFitQualityPct(float distanceSq);
float comboPctShapeDistanceToProfile(const LearnedProfile& p,const FbStats cur[5]);
float comboPctShapeDistanceToSpeedMode(int sp,int md,const FbStats cur[5]);
int detectSpeedPctShapeForMode(int md,const FbStats cur[5],float* confidenceOut,float* fitOut);
int detectModePctShapeForSpeed(int sp,const FbStats cur[5],float* confidenceOut,float* fitOut);
void manualSyncAccumulateBatch(const FbStats cur[5],float pairDist[3][3],uint8_t speedVotes[3][3],uint8_t oscOnVotes[3][3],uint8_t oscOffVotes[3][3],float oscScoreSum[3][3]);
void resetDiagnosticWindow() {
  diagWindowCount=0;
  diagWindowPos=0;
}

bool addDiagnosticSampleAndAverage(const FbStats sample[5],
                                   FbStats avg[5],
                                   uint8_t minWindows) {
  minWindows=constrain(minWindows,(uint8_t)1,(uint8_t)DIAG_ROLLING_WINDOWS);

  for(int ch=0;ch<5;ch++)diagWindow[diagWindowPos][ch]=sample[ch];
  diagWindowPos=(diagWindowPos+1)%DIAG_ROLLING_WINDOWS;
  if(diagWindowCount<DIAG_ROLLING_WINDOWS)diagWindowCount++;

  if(diagWindowCount<minWindows)return false;

  // Exakt die letzten mode-/osc-abhaengigen Fenster mitteln.
  uint8_t start=(uint8_t)((diagWindowPos+DIAG_ROLLING_WINDOWS-minWindows)%DIAG_ROLLING_WINDOWS);

  for(int ch=0;ch<5;ch++) {
    double mean=0.0,pct=0.0;
    double cdf[RICH_CDF_LEVELS] = {};
    int minv=4095,maxv=0;

    for(uint8_t k=0;k<minWindows;k++) {
      uint8_t i=(uint8_t)((start+k)%DIAG_ROLLING_WINDOWS);
      mean+=diagWindow[i][ch].avg;
      pct+=diagWindow[i][ch].abovePct;
      for(int q=0;q<RICH_CDF_LEVELS;q++)cdf[q]+=diagWindow[i][ch].cdfPct[q];
      if(diagWindow[i][ch].minv<minv)minv=diagWindow[i][ch].minv;
      if(diagWindow[i][ch].maxv>maxv)maxv=diagWindow[i][ch].maxv;
    }

    avg[ch].avg=(int)(mean/minWindows+0.5);
    avg[ch].abovePct=(float)(pct/minWindows);
    avg[ch].belowPct=100.0f-avg[ch].abovePct;
    for(int q=0;q<RICH_CDF_LEVELS;q++)avg[ch].cdfPct[q]=(float)(cdf[q]/minWindows);
    avg[ch].minv=minv;
    avg[ch].maxv=maxv;
    avg[ch].rangev=maxv-minv;
  }
  return true;
}

// Laufzeitmessung passend zur FINAL_EFFICIENT_BALANCED_V6-Kalibrierung.
// Kalibrierung und Runtime nutzen dieselbe phasenentkoppelte Verteilung.
void sampleProfileWindow(FbStats cur[5]) {
  const uint8_t pins[5]={
    PIN_LED1_FB,PIN_LED2_FB,PIN_LED3_FB,PIN_LED8_FB,PIN_LED9_FB
  };

  double sum[5]={0,0,0,0,0};
  uint32_t above[5]={0,0,0,0,0};
  uint32_t richAbove[5][RICH_CDF_LEVELS] = {};
  int minv[5]={4095,4095,4095,4095,4095};
  int maxv[5]={0,0,0,0,0};

  uint32_t rng=profileNewSeed(0x13579BDFu);
  delayMicroseconds((uint16_t)(profileRand32(rng)%2001u)); // zufaellige Startphase

  for(int i=0;i<PROFILE_SAMPLE_CYCLES;i++) {
    uint32_t r=profileRand32(rng);
    int first=(int)(r%5u);
    int dir=(r&0x100u)?1:-1;

    for(int j=0;j<5;j++) {
      int ch=(first + dir*j + 10)%5;

      // Erste ADC-Lesung nach MUX-Wechsel bewusst verwerfen.
      (void)analogRead(pins[ch]);
      int v=constrain(analogRead(pins[ch]),0,4095);

      sum[ch]+=v;
      if(v>=fbThresholds[ch])above[ch]++;
      for(int q=0;q<RICH_CDF_LEVELS;q++)
        if(v>=RICH_CDF_ADC[q])richAbove[ch][q]++;
      if(v<minv[ch])minv[ch]=v;
      if(v>maxv[ch])maxv[ch]=v;
    }

    delayMicroseconds(profileJitterUs(rng));
  }

  for(int ch=0;ch<5;ch++) {
    cur[ch].avg=(int)(sum[ch]/PROFILE_SAMPLE_CYCLES+0.5);
    cur[ch].abovePct=above[ch]*100.0f/PROFILE_SAMPLE_CYCLES;
    cur[ch].belowPct=100.0f-cur[ch].abovePct;
    for(int q=0;q<RICH_CDF_LEVELS;q++)
      cur[ch].cdfPct[q]=richAbove[ch][q]*100.0f/PROFILE_SAMPLE_CYCLES;
    cur[ch].minv=minv[ch];
    cur[ch].maxv=maxv[ch];
    cur[ch].rangev=maxv[ch]-minv[ch];
  }
}

void sampleProfileAverage(FbStats cur[5], uint8_t windows, uint16_t pauseMs) {
  windows=constrain(windows,(uint8_t)1,(uint8_t)10);

  double sumMean[5]={0,0,0,0,0};
  double sumPct[5]={0,0,0,0,0};
  double sumCdf[5][RICH_CDF_LEVELS] = {};
  int minv[5]={4095,4095,4095,4095,4095};
  int maxv[5]={0,0,0,0,0};

  for(uint8_t w=0;w<windows;w++) {
    FbStats one[5];
    sampleProfileWindow(one);

    for(int ch=0;ch<5;ch++) {
      sumMean[ch]+=one[ch].avg;
      sumPct[ch]+=one[ch].abovePct;
      for(int q=0;q<RICH_CDF_LEVELS;q++)sumCdf[ch][q]+=one[ch].cdfPct[q];
      if(one[ch].minv<minv[ch])minv[ch]=one[ch].minv;
      if(one[ch].maxv>maxv[ch])maxv[ch]=one[ch].maxv;
    }

    if(w+1<windows) {
      delay(pauseMs);
      yield();
    }
  }

  for(int ch=0;ch<5;ch++) {
    cur[ch].avg=(int)(sumMean[ch]/windows+0.5);
    cur[ch].abovePct=sumPct[ch]/windows;
    cur[ch].belowPct=100.0f-cur[ch].abovePct;
    for(int q=0;q<RICH_CDF_LEVELS;q++)cur[ch].cdfPct[q]=sumCdf[ch][q]/windows;
    cur[ch].minv=minv[ch];
    cur[ch].maxv=maxv[ch];
    cur[ch].rangev=maxv[ch]-minv[ch];
  }
}


int detectLearnedSpeed(float* confidenceOut=nullptr);
int detectLearnedMode(float* confidenceOut=nullptr);
bool ledsLookOff(const FbStats cur[5]);

static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="de"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0d1116"><title>Ventilator</title>
<style>
:root{--bg:#0d1116;--card:#161c24;--line:#232c38;--txt:#e8edf3;--dim:#78889c;--air:#4fd1c5}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;min-height:100dvh;background:var(--bg);color:var(--txt);
 font:400 17px/1.4 ui-rounded,-apple-system,"Segoe UI",Roboto,sans-serif;
 padding:24px 22px}
.wrap{width:100%;max-width:380px;margin:0 auto;display:flex;flex-direction:column;gap:16px}
h1{margin:0;font-size:13px;letter-spacing:.22em;text-transform:uppercase;color:var(--dim);font-weight:600}

/* ---------- Ventilator-Grafik ---------- */
.stage{position:relative;display:grid;place-items:center;padding:4px 0 0}
#fanart{width:100%;max-width:230px;height:auto;overflow:visible}
#fanart .guard{fill:none;stroke:var(--line);stroke-width:2}
#fanart .g2{stroke-width:1.2;opacity:.7}
#fanart .g3{stroke-width:1;opacity:.45}
#fanart .cage{fill:none;stroke:var(--line);stroke-width:1;opacity:.35}
#fanart .blade{fill:var(--air);fill-opacity:.13;stroke:var(--air);stroke-width:1.6;
 stroke-linejoin:round;transition:fill-opacity .3s,stroke-opacity .3s}
#fanart .hub{fill:var(--card);stroke:var(--air);stroke-width:2}
#fanart .hubdot{fill:var(--air)}
#fanart .stand{stroke:var(--line);stroke-width:5;stroke-linecap:round;fill:none}
#fanart .base{fill:var(--card);stroke:var(--line);stroke-width:2}
#fanart .w{fill:none;stroke:var(--air);stroke-width:2.5;stroke-linecap:round;opacity:0}

/* Rotor */
#blades{transform-origin:100px 92px;animation:spin var(--spin,1s) linear infinite;animation-play-state:paused}
@keyframes spin{to{transform:rotate(360deg)}}
/* Schwenken */
#head{transform-origin:100px 156px;animation:sway 7s ease-in-out infinite;animation-play-state:paused}
@keyframes sway{0%,100%{transform:rotate(-17deg)}50%{transform:rotate(17deg)}}
/* Luftstrom */
@keyframes gust{0%{opacity:0;transform:translateX(-10px) scaleX(.7)}
 25%{opacity:var(--wo,.8)}100%{opacity:0;transform:translateX(22px) scaleX(1.15)}}

#fanart[data-speed="1"]{--spin:1.15s;--gust:1.5s;--wo:.45}
#fanart[data-speed="2"]{--spin:.62s;--gust:1s;--wo:.7}
#fanart[data-speed="3"]{--spin:.3s;--gust:.62s;--wo:1}
#fanart:not([data-speed="0"]) #blades{animation-play-state:running}
#fanart[data-osc="1"] #head{animation-play-state:running}
#fanart[data-speed="0"] .blade{fill-opacity:.05;stroke-opacity:.4}
#fanart:not([data-speed="0"]) .w{animation:gust var(--gust) ease-out infinite}
#fanart .w2{animation-delay:.18s}
#fanart .w3{animation-delay:.36s}
#fanart[data-speed="1"] .w3{display:none}
@media(prefers-reduced-motion:reduce){
 #blades,#head,#fanart .w{animation:none!important}
 #fanart:not([data-speed="0"]) .w{opacity:var(--wo)}
}

/* Modus-Effekte */
#fanart[data-mode="breeze"] g.wind{animation:breathe 4.5s ease-in-out infinite}
@keyframes breathe{0%,100%{opacity:.25}45%{opacity:1}}
#fanart[data-mode="night"]{filter:brightness(.68) saturate(.75)}

/* ---------- Bedienung ---------- */
.seg{margin-top:12px;display:grid;grid-template-columns:repeat(3,1fr);gap:6px;background:var(--card);
 border:1px solid var(--line);border-radius:20px;padding:6px;transition:opacity .2s}
.seg.off{opacity:.4}
.seg button{border:none;background:none;color:var(--dim);font:600 14px/1 inherit;
 padding:12px 4px;border-radius:15px;cursor:pointer;transition:.18s}
.seg button.on{background:var(--air);color:#06231f}
.seg button:disabled{cursor:default}
.dial{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-bottom:2px}
.dial button{aspect-ratio:1;border:1px solid var(--line);border-radius:20px;background:var(--card);
 color:var(--dim);font:600 26px/1 inherit;display:grid;place-content:center;gap:6px;transition:.18s;cursor:pointer}
.dial button span{font-size:10px;letter-spacing:.14em;text-transform:uppercase}
.dial button.on{background:var(--air);border-color:var(--air);color:#06231f;box-shadow:0 0 26px -6px var(--air)}
.dial button:active{transform:scale(.94)}
.dial button:disabled{opacity:.5;cursor:default}
.row{display:flex;align-items:center;justify-content:space-between;gap:16px;
 background:var(--card);border:1px solid var(--line);border-radius:20px;padding:14px 20px}
.row small{display:block;color:var(--dim);font-size:13px;margin-top:2px}
.sw{width:62px;height:34px;flex:0 0 auto;border:none;border-radius:17px;background:var(--line);position:relative;transition:.2s;cursor:pointer}
.sw::after{content:"";position:absolute;top:4px;left:4px;width:26px;height:26px;border-radius:50%;background:var(--txt);transition:.2s}
.sw.on{background:var(--air)}.sw.on::after{left:32px;background:#06231f}
.sw:disabled{opacity:.35;cursor:default}

.timer{background:var(--card);border:1px solid var(--line);border-radius:20px;padding:14px 20px 8px;transition:opacity .2s}
.timer.off{opacity:.4}
.thead{display:flex;align-items:baseline;justify-content:space-between;gap:12px}
.tval{font:600 30px/1 inherit;font-variant-numeric:tabular-nums;color:var(--dim);transition:color .2s;white-space:nowrap}
.tval.on{color:var(--air)}
.tval em{font-style:normal;font-size:14px;font-weight:400;margin-left:3px;color:var(--dim)}
.trest{color:var(--dim);font-size:13px;margin-top:2px}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;margin:8px 0 0;height:34px;background:transparent;cursor:pointer}
input[type=range]:disabled{cursor:default}
input[type=range]::-webkit-slider-runnable-track{height:8px;border-radius:4px;background:var(--trk,var(--line))}
input[type=range]::-moz-range-track{height:8px;border-radius:4px;background:var(--trk,var(--line))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;margin-top:-9px;border-radius:50%;
 background:var(--knob,var(--dim));border:4px solid var(--card);box-shadow:0 2px 10px #0009;transition:transform .12s}
input[type=range]::-moz-range-thumb{width:26px;height:26px;border-radius:50%;border:4px solid var(--card);
 background:var(--knob,var(--dim));box-shadow:0 2px 10px #0009}
input[type=range]:active::-webkit-slider-thumb{transform:scale(1.18)}
.ticks{display:flex;justify-content:space-between;font-size:11px;color:var(--dim);
 letter-spacing:.06em;padding:0 2px 4px;font-variant-numeric:tabular-nums}
.sync{border:none;background:none;color:var(--dim);font:inherit;font-size:13px;text-decoration:underline;padding:2px;cursor:pointer}
.demo{border-top:1px dashed var(--line);padding-top:12px;color:var(--dim);font-size:12px;line-height:1.7}
.demo b{color:var(--txt);font-weight:600}
.demo label{display:flex;align-items:center;gap:8px;margin-top:6px;cursor:pointer}
.demo input{accent-color:var(--air);width:16px;height:16px}

/* ---------- ESP32 / Netzwerk / OTA ---------- */
.statusbar{display:flex;align-items:center;justify-content:space-between;gap:10px;
 background:var(--card);border:1px solid var(--line);border-radius:16px;padding:10px 14px;
 color:var(--dim);font-size:12px}
.statusleft,.statusright{display:flex;align-items:center;gap:8px}
.dot{width:8px;height:8px;border-radius:50%;background:var(--dim);box-shadow:0 0 0 3px #0002}
.dot.ok{background:var(--air)} .dot.err{background:#ff7373}
.badge{border:1px solid var(--line);border-radius:999px;padding:3px 8px;font-size:11px}
nav.tabs{display:grid;grid-template-columns:1fr 1fr;gap:6px;background:var(--card);
 border:1px solid var(--line);border-radius:18px;padding:5px;margin:14px 0 4px;position:sticky;top:8px;z-index:5}
nav.tabs button{border:none;background:none;color:var(--dim);font:600 14px/1 inherit;
 padding:12px 4px;border-radius:13px;cursor:pointer;transition:.18s}
nav.tabs button.on{background:var(--air);color:#06231f}
section[hidden]{display:none}
#viewCfg:not([hidden]){display:flex;flex-direction:column;gap:14px}
#viewMain:not([hidden]){display:block}
a.panellink{display:flex;align-items:center;justify-content:space-between;gap:12px;
 background:var(--card);border:1px solid var(--line);border-radius:20px;padding:16px;
 color:var(--txt);text-decoration:none;font-weight:600;font-size:15px}
a.panellink span{color:var(--dim);font-weight:400;font-size:13px;display:block;margin-top:3px}
a.panellink i{color:var(--air);font-style:normal;font-size:20px}
.dial button.on:disabled,.seg button.on:disabled{opacity:1}
.dial button.pending,.seg button.pending,.sw.pending{animation:pend 1.1s ease-in-out infinite}
@keyframes pend{50%{opacity:.55}}
details.panel{background:var(--card);border:1px solid var(--line);border-radius:20px;padding:0 16px;
 overflow:hidden;flex:0 0 auto}
details.panel>summary{cursor:pointer;list-style:none;padding:14px 0;font-size:14px;font-weight:600}
details.panel>summary::-webkit-details-marker{display:none}
.panelbody{border-top:1px solid var(--line);padding:12px 0 15px;color:var(--dim);font-size:13px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.kv{background:#10161d;border:1px solid var(--line);border-radius:13px;padding:9px 10px}
.kv b{display:block;color:var(--txt);font-size:12px;margin-bottom:2px}
.mini{border:1px solid var(--line);background:#10161d;color:var(--txt);border-radius:10px;padding:7px 10px;font:inherit;font-size:12px;cursor:pointer}
.upload{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:10px}
.upload input[type=file]{max-width:100%;font-size:12px}
.note{font-size:11px;line-height:1.45;color:var(--dim);margin-top:8px}
.warn{border:1px solid #805f27;background:#2a2114;border-radius:14px;padding:10px 12px;color:#d8c18c;font-size:11px;line-height:1.45}

/* ---------- Manuelle Web-Status-Korrektur ---------- */
.manual-card{background:var(--card);border:1px solid var(--line);border-radius:20px;overflow:hidden}
.manual-details>summary{list-style:none;cursor:pointer;display:flex;align-items:flex-start;
 justify-content:space-between;gap:12px;padding:15px}
.manual-details>summary::-webkit-details-marker{display:none}
.manual-details>summary::after{content:"⌄";color:var(--dim);font-size:18px;line-height:1;
 margin-left:6px;transition:.18s}
.manual-details[open]>summary::after{transform:rotate(180deg)}
.manual-details>summary b{font-size:14px}
.manual-details>summary span{display:block;color:var(--dim);font-size:11px;margin-top:2px}
.manual-status{color:var(--air)!important;font-size:11px!important;white-space:nowrap;
 padding-top:2px;margin-left:auto}
.manual-body{border-top:1px solid var(--line);padding:12px 15px 15px}
.manual-buttons{display:grid;grid-template-columns:repeat(4,1fr);gap:7px}
.manual-buttons button,.manual-modes button{border:1px solid var(--line);background:#10161d;color:var(--txt);
 border-radius:12px;padding:11px 4px;font:600 12px/1 inherit;cursor:pointer}
.manual-buttons button.on,.manual-modes button.on{background:var(--air);border-color:var(--air);color:#06231f}
.manual-buttons button:disabled,.manual-modes button:disabled{opacity:.5;cursor:default}
.manual-buttons button:active,.manual-modes button:active{transform:scale(.97)}
.manual-mode-label{margin:12px 2px 7px;color:var(--dim);font-size:11px;text-transform:uppercase;
 letter-spacing:.12em}
.manual-modes{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}
.manual-osc-row{display:flex;align-items:center;justify-content:space-between;gap:14px;
 background:#10161d;border:1px solid var(--line);border-radius:12px;padding:10px 12px}
.manual-osc-row b{display:block;color:var(--txt);font-size:12px}
.manual-osc-row span{display:block;color:var(--dim);font-size:10px;margin-top:2px}
.sync-wide{width:100%;padding:12px 14px;border-radius:13px;font-weight:600}

/* ---------- Einstellungen kompakt gruppiert ---------- */
.grouped-tools{display:flex;flex-direction:column;gap:10px}
.tool-block{background:#10161d;border:1px solid var(--line);border-radius:14px;padding:11px}
.tool-head{display:flex;align-items:center;justify-content:space-between;gap:12px}
.tool-head b{display:block;color:var(--txt);font-size:13px}
.tool-head span{display:block;color:var(--dim);font-size:11px;margin-top:2px}
.tool-link{color:inherit;text-decoration:none}
.calibration-subpanel{padding:0;overflow:hidden}
.calibration-subpanel>summary{list-style:none;cursor:pointer;padding:11px}
.calibration-subpanel>summary::-webkit-details-marker{display:none}
.calibration-subpanel>summary::after{content:"⌄";color:var(--dim);font-size:18px;margin-left:8px}
.calibration-subpanel[open]>summary::after{transform:rotate(180deg)}
.calibration-subpanel>.note{padding:0 11px}
.calibration-subpanel>.panelbody{margin:0 11px;padding-bottom:11px}

</style></head><body>
<div class="wrap">
<h1 id="hdr">Aus</h1>
<div class="statusbar">
 <div class="statusleft"><span class="dot" id="connDot"></span><span id="connTxt">ESP32 wird gesucht…</span></div>
 <div class="statusright"><span class="badge" id="fwBadge">FW —</span></div>
</div>

<nav class="tabs">
 <button id="tabMain" class="on" type="button">Steuerung</button>
 <button id="tabCfg" type="button">Einstellungen</button>
</nav>

<section id="viewMain">
<div class="stage">
<svg id="fanart" viewBox="0 0 200 205" data-speed="0" data-osc="0" aria-hidden="true">
 <!-- Luftstrom -->
 <g class="wind">
  <path class="w w1" d="M168 66 q16 14 0 28"/>
  <path class="w w2" d="M168 92 q20 14 0 28" style="stroke-width:3"/>
  <path class="w w3" d="M168 120 q14 12 0 24"/>
 </g>
 <!-- Fuß -->
 <path class="stand" d="M100 150 L100 182"/>
 <ellipse class="base" cx="100" cy="186" rx="30" ry="8"/>
 <!-- Kopf: schwenkt -->
 <g id="head">
  <circle class="guard" cx="100" cy="92" r="62"/>
  <circle class="guard g2" cx="100" cy="92" r="47"/>
  <circle class="guard g3" cx="100" cy="92" r="31"/>
  <path class="cage" d="M100 30 V154 M38 92 H162 M56 48 L144 136 M144 48 L56 136"/>
  <g id="blades">
   <path class="blade" id="bl" d="M100 92 C92 66 100 44 120 38 C134 56 128 80 100 92 Z"/>
   <use href="#bl" class="blade" transform="rotate(120 100 92)"/>
   <use href="#bl" class="blade" transform="rotate(240 100 92)"/>
  </g>
  <circle class="hub" cx="100" cy="92" r="12"/>
  <circle class="hubdot" cx="100" cy="92" r="4"/>
 </g>
</svg>
</div>

<div class="dial">
 <button data-s="0">0<span>An / Aus</span></button>
 <button data-s="1">1<span>Leise</span></button>
 <button data-s="2">2<span>Mittel</span></button>
 <button data-s="3">3<span>Stark</span></button>
</div>
<div class="seg off" id="modes">
 <button data-m="0" class="on">Normal</button>
 <button data-m="1">Breeze</button>
 <button data-m="2">Nacht</button>
</div>
<div class="row" style="margin-top:10px"><div>Drehen<small>Schwenkbetrieb</small></div><button id="osc" class="sw"></button></div>
<div class="timer off" id="tcard" style="margin-top:14px">
 <div class="thead">
  <div>Sleep-Timer<div class="trest" id="trest">Ventilator ist aus</div></div>
  <div class="tval" id="tval">Aus</div>
 </div>
 <input type="range" id="slp" min="0" max="240" step="5" value="0" disabled
        aria-label="Sleep-Timer in Minuten">
 <div class="ticks"><span>Aus</span><span>1h</span><span>2h</span><span>3h</span><span>4h</span></div>
</div>

<div class="upload" style="margin-top:12px">
 <button class="mini sync-wide" id="ledSync" type="button">Status über LEDs abgleichen</button>
</div>

<details class="manual-card manual-details" id="manualStatePanel" style="margin-top:12px">
 <summary>
  <div>
   <b>Web-Status manuell setzen</b>
   <span>Falls der Ventilator direkt am Gerät bedient wurde</span>
  </div>
  <div class="manual-status" id="manualState">—</div>
 </summary>
 <div class="manual-body">
  <div class="manual-buttons" id="manualSpeedButtons">
   <button data-manual-speed="0" type="button">Aus</button>
   <button data-manual-speed="1" type="button">Stufe 1</button>
   <button data-manual-speed="2" type="button">Stufe 2</button>
   <button data-manual-speed="3" type="button">Stufe 3</button>
  </div>
  <div class="manual-mode-label">Modus</div>
  <div class="manual-modes" id="manualModeButtons">
   <button data-manual-mode="0" type="button">Normal</button>
   <button data-manual-mode="1" type="button">Breeze</button>
   <button data-manual-mode="2" type="button">Nacht</button>
  </div>
  <div class="manual-mode-label">Drehen</div>
  <div class="manual-osc-row">
   <div>
    <b>Schwenkbetrieb</b>
    <span>Nur den Web-Status korrigieren</span>
   </div>
   <button id="manualOsc" class="sw" type="button" aria-label="Drehen-Status manuell setzen"></button>
  </div>
 </div>
</details>


</section>

<section id="viewCfg" hidden>

<details class="panel">
<summary>ESP32 & OTA</summary>
<div class="panelbody">
 <div class="grid2">
  <div class="kv"><b>IP-Adresse</b><span id="ip">—</span></div>
  <div class="kv"><b>WLAN</b><span id="rssi">—</span></div>
  <div class="kv"><b>Uptime</b><span id="uptime">—</span></div>
  <div class="kv"><b>Build</b><span id="build">—</span></div>
  <div class="kv"><b>Gemerkter Zustand</b><span id="remembered">—</span></div>
  <div class="kv"><b>Statusquelle</b><span id="stateSource">—</span></div>
 </div>

 <div class="note" style="margin-top:14px"><b style="color:var(--txt)">Kalibrierungsstatus</b><br>
 Hier siehst du nach Neustart oder Update sofort, ob die finale Kalibrierung geladen ist und welches Erkennungsmodell tatsächlich benutzt wird.</div>
 <div class="grid2" style="margin-top:10px">
  <div class="kv"><b>Finale Profile</b><span id="otaCal18">—</span></div>
  <div class="kv"><b>Aktives Modell</b><span id="otaModel">—</span></div>
  <div class="kv"><b>AUS-Profil</b><span id="otaOffProfile">—</span></div>
  <div class="kv"><b>9P-Fallback</b><span id="otaCal9">—</span></div>
  <div class="kv"><b>Profilspeicher</b><span id="otaStorage">—</span></div>
  <div class="kv"><b>Kalibrierungsstand</b><span id="otaCalSeq">—</span></div>
 </div>

 <form class="upload" id="otaForm">
  <input type="file" id="otaFile" accept=".bin,application/octet-stream">
  <button class="mini" type="submit">OTA installieren</button>
 </form>
 <div class="note" id="otaMsg">Spätere Firmware-Updates direkt über WLAN. Verbindungsanzeige läuft getrennt von den langsameren LED-Messungen.</div>
 <div class="warn" style="margin-top:12px">GPIO-Belegung &mdash; Tasten: T1/Power = GP6, T2/Speed = GP5, T3/Drehen = GP4, T5/Modus = GP3. LED-Feedback: Speed 1 = GP7, Speed 2 = GP8, Speed 3 = GP9, Breeze = GP10, Nacht = GP1.</div>
</div>
</details>

<details class="panel">
<summary>Kalibrierung &amp; Daten</summary>
<div class="panelbody grouped-tools">

 <div class="tool-block">
  <div class="tool-head">
   <a class="tool-link" href="/analyze">
    <b>Trennschärfe-Analyse</b>
    <span>Messprofile, Diagnose und Übernahme</span>
   </a>
   <a class="mini tool-link" href="/analyze">Öffnen</a>
  </div>
 </div>

 <details class="tool-block calibration-subpanel">
  <summary class="tool-head">
   <div>
    <b>Speed-Kalibrierung</b>
    <span>Referenzprofile, Diagnose und Fallback</span>
   </div>
  </summary>
<div class="note">Aktive Schwellwerte: <span id="thrNow">—</span><br>
Werden sie geändert, verlieren alle gelernten Profile ihre Gültigkeit und müssen
neu aufgenommen werden — egal ob einzeln oder über die Trennschärfe-Analyse.</div>
<div class="panelbody">
 <div class="note">Diese Einzelprofile sind nur Diagnose/Fallback. Das produktive FINAL-Tracking nutzt die 18 Profile. Im Hauptbetrieb <b>Normal</b> wird zusaetzlich die offset-/skalierungsfreie relative Kanalform verwendet; der Webzustand bleibt autoritativ.</div>
 
 <div class="grid2" style="margin-top:10px">
  <div class="kv"><b>Speed 1</b><span id="cal1">nicht gelernt</span></div>
  <div class="kv"><b>Speed 2</b><span id="cal2">nicht gelernt</span></div>
  <div class="kv"><b>Speed 3</b><span id="cal3">nicht gelernt</span></div>
  <div class="kv"><b>Aus-Zustand</b><span id="calOff">nicht gelernt</span></div>
  <div class="kv"><b>Kombiprofile</b><span id="calCombo">nicht gelernt</span></div>
  <div class="kv"><b>Web-Soll</b><span id="detWebState">—</span></div>
  <div class="kv"><b>LED-Pruefung Speed</b><span id="detSpeed">—</span></div>
  <div class="kv"><b>Speed Sicherheit</b><span id="detConf">—</span></div>
  <div class="kv"><b>LED-Pruefung Drehen</b><span id="detOsc">—</span></div>
  <div class="kv"><b>Drehen Sicherheit</b><span id="detOscConf">—</span></div>
  <div class="kv"><b>Tracking</b><span id="detTracking">—</span></div>
  <div class="kv"><b>Modell-Fit</b><span id="detFit">—</span></div>
  <div class="kv"><b>18P Treffer ohne Web-Kontext</b><span id="detProfile">—</span></div>
  <div class="kv"><b>Messalter</b><span id="detAge">—</span></div>
  <div class="kv"><b>Live LED-%</b><span id="detLivePct">—</span></div>
  <div class="kv"><b>Web-Soll-Profil</b><span id="detExpectedPct">—</span></div>
 </div>
 <div class="upload">
  <button class="mini" id="forgetSpeedCal" type="button">Speed-Kalibrierung löschen</button>
 </div>
 <div class="upload">
  <button class="mini" id="learn1" type="button">Speed 1 lernen</button>
  <button class="mini" id="learn2" type="button">Speed 2 lernen</button>
  <button class="mini" id="learn3" type="button">Speed 3 lernen</button>
 </div>
 <div class="note" style="margin-top:10px">Der Aus-Zustand wird ebenfalls gemessen statt geschätzt.
 Ventilator wirklich ausschalten, dann lernen — sonst erkennt „Status über LEDs abgleichen" den
 Aus-Zustand nicht und landet auf Stufe 1.</div>
 <div class="upload">
  <button class="mini" id="learnOff" type="button">Aus-Zustand lernen</button>
 </div>
 
 
 <div class="note" style="margin-top:14px"><b style="color:var(--txt)">Modus-Fallback lernen</b> — optional fuer Diagnose; das FINAL-18P-Modell bleibt unveraendert.</div>
 
 <div class="grid2" style="margin-top:10px">
  <div class="kv"><b>Normal</b><span id="calNormal">nicht gelernt</span></div>
  <div class="kv"><b>Breeze</b><span id="calBreeze">nicht gelernt</span></div>
  <div class="kv"><b>Night</b><span id="calNight">nicht gelernt</span></div>
  <div class="kv"><b>LED-Pruefung Modus</b><span id="detMode">—</span></div>
  <div class="kv"><b>Modus Sicherheit</b><span id="detModeConf">—</span></div>
 </div>
 <div class="upload">
  <button class="mini" id="forgetModeCal" type="button">Modus-Kalibrierung löschen</button>
 </div>
 <div class="upload">
  <button class="mini" id="learnNormal" type="button">Normal lernen</button>
  <button class="mini" id="learnBreeze" type="button">Breeze lernen</button>
  <button class="mini" id="learnNight" type="button">Night lernen</button>
 </div>
 
 <div class="note"><b style="color:var(--txt)">Zustandsprioritaet:</b> Webbefehle, manuelle Stufen-/Modus-/Drehen-Bedienung und der Sleep-Timer sind verbindlich. Die LED-Diagnose veraendert die Hauptanzeige nie automatisch. Die finale LED-Erkennung nutzt FINAL_EFFICIENT_BALANCED_V6: zufaellige Startphase, Kanalreihenfolge und Abtastabstaende verhindern eine Synchronisation mit dem LED-Multiplex. Die Felder „LED-Pruefung Speed/Modus/Drehen“ pruefen gezielt gegen den bewusst gesetzten Webzustand; „18P Treffer“ bleibt die komplett kontextfreie LED-Erkennung. „Status über LEDs abgleichen“ und „Web-Status manuell setzen“ dürfen den Zustand bewusst korrigieren; beide erzeugen dabei keine automatischen LED-basierten Hintergrundänderungen.</div><div class="note"><b style="color:var(--txt)">FINAL_EFFICIENT_BALANCED_V6:</b> Diese Generation kombiniert phasenentkoppelte Jitter-Abtastung, CDF-Prozentmerkmale und relative ADC-Mittelwertmuster. Alte 6.0–6.4.2-Profile/Backups und fruehere Analyse-Checkpoints sind absichtlich inkompatibel; fuer EFFICIENT-BALANCED V6 ist genau einmal der neue komplette Messlauf erforderlich.</div><div class="note"><b style="color:var(--txt)">Finale Messlogik:</b> AUS wird separat erkannt. Das FINAL-Modell klassifiziert 18 vollständige Speed × Modus × Drehen-Zustände; 9P und Einzelprofile sind nur Rückfall/Diagnose. Die LED-Diagnose überschreibt den Webzustand nie automatisch.</div><div class="note"><b style="color:var(--txt)">Manueller LED-Sync:</b> Drei unabhängige Messbatches verwenden exakt dieselbe Modus-/Drehen-abhängige Fenstertiefe wie die Hintergrunddiagnose (Normal AUS 6, Normal AN 8, dynamisch AUS 8, dynamisch AN 10). Ein frischer stabiler Diagnose-Speed wird bevorzugt; sonst entscheidet die Batch-Mehrheit und erst danach die gemittelte Formdistanz. NORMAL bleibt der Ausgangspunkt. Ein Moduswechsel weg von NORMAL oder ein widersprechender Drehen-Wechsel braucht stärkere Evidenz.</div><div class="note"><b style="color:var(--txt)">Tracking:</b> Web-Soll, Timer und Befehlszustand werden direkt nach dem Tastendruck bzw. der Serverbestätigung aktualisiert. Die LED-Diagnose läuft separat im Hintergrund; /api/status ist ADC-frei und liefert fertige Cachewerte. Dadurch bleibt die Oberfläche schnell, während die Erkennung exakt 6/8 Fenster in Normal bzw. 8/10 Fenster in Breeze/Nacht, zeitlich gespreizt mittelt.</div><div class="note" id="calMsg">Noch nichts gelernt.</div>
</div>
 </details>

 <div class="tool-block">
  <div class="tool-head">
   <div>
    <b>Backup &amp; Wiederherstellung</b>
    <span>Kalibrierung sichern oder wiederherstellen</span>
   </div>
  </div>
<div class="panelbody">
 <div class="note">Exportiert interne Messparameter, gelernte Speed-/Mode-Profile und die zuletzt gemerkten Einstellungen als JSON-Datei. Export ist erst bei einem vollstaendigen FINAL-18P-Modell freigegeben. Das JSON enthaelt Schwellen, 18P/9P-Profile, CDF- und Mean-Gewichte sowie Fit-Grenze. Nur FINAL_EFFICIENT_BALANCED_V6-Backups werden importiert.</div>
 <div class="upload">
  <button class="mini" id="backupExport" type="button">Backup exportieren</button>
  <input type="file" id="backupFile" accept=".json,application/json">
  <button class="mini" id="backupImport" type="button">Backup importieren</button>
 </div>
 <div class="note" id="backupMsg">Noch kein Backup importiert/exportiert.</div>
</div>
 </div>

</div>
</details>

</section>


</div>
<script>
let S={speed:0,osc:false,sleep:0,sleepSec:0,mode:0},busy=false,tick=null,restMin=0,online=false,calibrating=false;
let remembered={speed:1,mode:0,osc:false};
let confirmedSleepSec=0,confirmedSleepAt=Date.now();
let lastStateRevision=0,lastBootId=0,sleepSyncAt=Date.now();
const MODE=['Normal','Breeze','Nacht'],MKEY=['normal','breeze','night'];
const $=q=>document.querySelector(q);
const slp=$('#slp'), art=$('#fanart');

 const fmt=(m)=>{
 if(m<=0)return'Aus';
 if(m<60)return m+'<em>min</em>';
 return Math.floor(m/60)+':'+String(m%60).padStart(2,'0')+'<em>h</em>';
}
 const currentSleepSec=()=>{
  if(!S.sleepSec)return 0;
  return Math.max(0,Math.ceil(S.sleepSec-(Date.now()-sleepSyncAt)/1000));
}
 const fmtClock=(sec)=>{
  sec=Math.max(0,Math.round(sec)||0);
  const h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),s=sec%60;
  return h?`${h}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`:`${m}:${String(s).padStart(2,'0')}`;
}
 const applyServerState=(st)=>{
  if(!st)return false;

  const boot=Number(st.bootId||0);
  if(boot && boot!==lastBootId){
    lastBootId=boot;
    lastStateRevision=0;
  }

  const rev=Number(st.revision||0);
  if(rev && rev<lastStateRevision)return false;
  if(rev)lastStateRevision=rev;

  const powerOn=!!st.powerOn;
  S.speed=powerOn?(Number(st.speed)||1):0;
  S.osc=powerOn?!!st.osc:false;
  S.mode=powerOn?(Number(st.mode)||0):0;
  S.sleep=Number(st.sleep)||0;
  S.sleepSec=Number(st.sleepSec)||0;
  sleepSyncAt=Date.now();
  confirmedSleepSec=S.sleepSec;
  confirmedSleepAt=sleepSyncAt;

  remembered.speed=Math.min(3,Math.max(1,Number(st.lastSpeed)||1));
  remembered.mode=Math.min(2,Math.max(0,Number(st.lastMode)||0));
  remembered.osc=!!st.lastOsc;

  const rem=$('#remembered');
  if(rem)rem.textContent='Speed '+remembered.speed+' · '+MODE[remembered.mode]+' · Drehen '+(remembered.osc?'AN':'AUS');
  return true;
}
 const fmtUp=(s)=>{
 s=Math.max(0,Number(s)||0);const d=Math.floor(s/86400);s%=86400;
 const h=Math.floor(s/3600),m=Math.floor((s%3600)/60);
 return(d?d+'d ':'')+h+'h '+m+'m';
}
 const paintSlider=()=>{
 const v=+slp.value,pct=v/240*100;
 slp.style.setProperty('--trk',`linear-gradient(90deg,${v>0?'var(--air)':'var(--dim)'} ${pct}%,var(--line) ${pct}%)`);
 slp.style.setProperty('--knob',v>0?'var(--air)':'var(--dim)');
}
 const paintTimer=()=>{
 const live=S.speed>0,sec=currentSleepSec();
 $('#tcard').classList.toggle('off',!live);
 slp.disabled=!live||busy;
 $('#tval').classList.toggle('on',sec>0);
 $('#tval').innerHTML=sec>0?fmtClock(sec):'Aus';
 $('#trest').textContent=!live?'Ventilator ist aus':(sec>0?'Abschaltung in '+fmtClock(sec):'Läuft ohne Abschaltung');
 paintSlider();
}
 const paintManualPanel=()=>{
 const ms=$('#manualState');
 const displayMode=S.speed?S.mode:remembered.mode;
 const displayOsc=S.speed?S.osc:remembered.osc;
 if(ms)ms.textContent=S.speed
   ?('Stufe '+S.speed+' · '+MODE[displayMode]+' · Drehen '+(displayOsc?'AN':'AUS'))
   :('Aus · '+MODE[displayMode]+' · Drehen '+(displayOsc?'AN':'AUS'));

 document.querySelectorAll('#manualSpeedButtons button').forEach(b=>{
  b.classList.toggle('on',+b.dataset.manualSpeed===S.speed);
  b.disabled=busy||calibrating;
 });
 document.querySelectorAll('#manualModeButtons button').forEach(b=>{
  b.classList.toggle('on',+b.dataset.manualMode===displayMode);
  b.disabled=busy||calibrating;
 });
 const mo=$('#manualOsc');
 if(mo){
  mo.classList.toggle('on',displayOsc);
  mo.classList.toggle('pending',busy);
  mo.disabled=busy||calibrating;
 }
}
 const draw=()=>{
 paintManualPanel();
 const web=$('#detWebState');
 if(web)web.textContent=S.speed?('St'+S.speed+' / '+MODE[S.mode]+' / '+(S.osc?'AN':'AUS')):'AUS';
 document.querySelectorAll('.dial button').forEach(b=>{
  const t=+b.dataset.s;
  const active=(t===S.speed);
  b.classList.toggle('on',active);
  b.classList.toggle('pending',busy&&active);
  b.disabled=busy||(S.speed===0&&t>0);
 });
 $('#osc').classList.toggle('on',S.osc);
 $('#osc').classList.toggle('pending',busy);
 $('#osc').disabled=busy||!S.speed;
 $('#hdr').textContent=(S.speed?'Stufe '+S.speed+(S.mode?' · '+MODE[S.mode]:'')+(S.osc?' · dreht':''):'Aus');
 art.dataset.speed=S.speed;art.dataset.osc=S.osc?'1':'0';art.dataset.mode=S.speed?MKEY[S.mode]:'normal';
 document.querySelectorAll('#modes button').forEach(b=>{
  const act=(+b.dataset.m===S.mode);
  b.classList.toggle('on',act);
  b.classList.toggle('pending',busy&&act);
  b.disabled=busy||!S.speed;
 });
 $('#modes').classList.toggle('off',!S.speed);
 slp.value=S.sleep||0;
 paintTimer();
}
 const api=async (path,opts={})=>{
 const ctl=new AbortController(),to=setTimeout(()=>ctl.abort(),8000);
 try{
  const r=await fetch(path,{...opts,signal:ctl.signal,cache:'no-store'});
  clearTimeout(to);
  if(!r.ok)throw new Error('HTTP '+r.status);
  const ct=r.headers.get('content-type')||'';
  return ct.includes('application/json')?await r.json():await r.text();
 }catch(e){
  clearTimeout(to);
  throw e;
 }
}
let lastHealthOk=0;
// Statusabrufe strikt nacheinander; keine alte Antwort darf eine neuere
// Diagnoseanzeige wieder überschreiben.
let statusInFlight=false;
let statusPending=false;
let statusAbort=null;
let statusPollNotBefore=0;
let statusRequestSerial=0;
let statusAppliedSerial=0;

const markDiagSettling=()=>{
  if($('#detSpeed')) $('#detSpeed').textContent='stabilisiert…';
  if($('#detConf')) $('#detConf').textContent='—';
  if($('#detMode')) $('#detMode').textContent='stabilisiert…';
  if($('#detModeConf')) $('#detModeConf').textContent='—';
  if($('#detOsc')) $('#detOsc').textContent='stabilisiert…';
  if($('#detOscConf')) $('#detOscConf').textContent='—';
  if($('#detProfile')) $('#detProfile').textContent='—';
  if($('#detLivePct')) $('#detLivePct').textContent='—';
  if($('#detExpectedPct')) $('#detExpectedPct').textContent='wird aktualisiert…';
  if($('#detTracking')) $('#detTracking').textContent='stabilisiert…';
  if($('#detFit')) $('#detFit').textContent='—';
  if($('#detAge')) $('#detAge').textContent='—';
};

 const pollHealth=async ()=>{
 if(calibrating){
  $('#connDot').className='dot ok';
  $('#connTxt').textContent='Kalibrierung läuft…';
  return;
 }
 const ctl=new AbortController();
 const to=setTimeout(()=>ctl.abort(),2500);
 try{
  const r=await fetch('/api/health',{signal:ctl.signal,cache:'no-store'});
  clearTimeout(to);
  if(!r.ok)throw new Error('HTTP '+r.status);
  const x=await r.json();

  lastHealthOk=Date.now();
  online=true;
  $('#connDot').className='dot ok';
  $('#connTxt').textContent='ESP32 verbunden';

  $('#ip').textContent=x.ip||'—';
  $('#rssi').textContent=x.rssi!=null?x.rssi+' dBm':'—';
  $('#uptime').textContent=fmtUp(x.uptime);
  $('#build').textContent=x.build||'—';
  $('#fwBadge').textContent='FW '+(x.version||'—');
 }catch(e){
  clearTimeout(to);

  // Erst nach mehreren Sekunden ohne EINEN erfolgreichen Health-Check offline anzeigen.
  // Ein langsamer LED-Scan oder Lernvorgang darf die Anzeige nicht sofort rot machen.
  if(Date.now()-lastHealthOk > 9000){
   online=false;
   $('#connDot').className='dot err';
   $('#connTxt').textContent='ESP32 nicht erreichbar';
  }else{
   $('#connTxt').textContent='ESP32 beschäftigt…';
  }
 }
}

 const pollStatus=async ()=>{
 if(calibrating)return;
 if(Date.now()<statusPollNotBefore)return;
 if(statusInFlight){statusPending=true;return;}

 statusInFlight=true;
 const req=++statusRequestSerial;
 statusAbort=new AbortController();
 const ctl=statusAbort;
 const to=setTimeout(()=>ctl.abort(),5000);

 try{
  const r=await fetch('/api/status',{signal:ctl.signal,cache:'no-store'});
  clearTimeout(to);
  if(!r.ok)throw new Error('HTTP '+r.status);
  const x=await r.json();

  if(req<statusAppliedSerial)return;
  statusAppliedSerial=req;
  if(x.state){
    applyServerState(x.state);
    const ss=$('#stateSource');
    if(ss)ss.textContent='Web-/Sollzustand · Rev '+(x.state.revision||lastStateRevision);
  }
  if(x.thresholds && $('#thrNow')){
    $('#thrNow').textContent=x.thresholds.join(' · ');
  }
  if(x.offLearned!==undefined && $('#calOff')){
    $('#calOff').textContent = x.offLearned ? '✓ gelernt' : 'nicht gelernt';
  }
  if($('#calCombo')){
    $('#calCombo').textContent = x.recognitionModel==='FINAL_18P'&&x.combo18Learned ? ('✓ 18 von 18 · NORMAL-PRIORITY'+(x.oscLinearValid?' + OSC-LINEAR':'')) :
      (x.combo18Count ? (x.combo18Count+'/18 · nicht FINAL') : 'unvollständig');
  }
  // Kleine dauerhafte Übersicht unter "ESP32 & OTA".
  const c18=Number(x.combo18Count||0);
  const c9=Number(x.combo9Count||0);
  if($('#otaCal18')){
    $('#otaCal18').textContent = c18===18 ? '✓ 18/18 geladen' : ('⚠ '+c18+'/18 geladen');
  }
  if($('#otaCal9')){
    $('#otaCal9').textContent = c9===9 ? '✓ 9/9 geladen' : (c9+'/9');
  }
  if($('#otaOffProfile')){
    $('#otaOffProfile').textContent = x.offLearned ? '✓ geladen' : '⚠ fehlt';
  }
  if($('#otaModel')){
    const model=x.recognitionModel||'';
    $('#otaModel').textContent =
      model==='FINAL_18P' ? ('✓ FINAL 18P · NORMAL-PRIORITY'+(x.oscLinearValid?' + OSC-LINEAR':'')+' aktiv') :
      model==='FALLBACK_9P' ? '⚠ 9P-Fallback aktiv' :
      model==='NEUKALIBRIERUNG' ? '⚠ Finale Analyse erforderlich' :
      '⚠ Einzelprofile aktiv';
  }
  if($('#otaStorage')){
    const slots=Number(x.calSlots||0);
    $('#otaStorage').textContent = x.comboStorage!=='AB_CRC_NVS' ? '⚠ neue Kalibrierung nötig' :
      (slots>=2?'✓ A/B + CRC · 2/2':'⚠ CRC gültig · nur 1/2 Slots');
  }
  if($('#otaCalSeq')){const crc=Number(x.calCrc||0);$('#otaCalSeq').textContent=x.calSequence?('Gen '+x.calSequence+' · CRC '+crc.toString(16).toUpperCase().padStart(8,'0')):'—';}
  if($('#ledSync')){
    const modelOk=x.recognitionModel==='FINAL_18P';
    const syncOk=modelOk;
    $('#ledSync').disabled=busy||!syncOk;
    $('#ledSync').title=syncOk
      ?'Manueller 3-Batch-Abgleich · NORMAL priorisiert · globaler 18P-Score ist nur Diagnose'
      :'Erst finales Kalibrierungsmodell laden';
  }
  if(x.calibration){
    $('#cal1').textContent=x.calibration.s1?'✓ gelernt':'nicht gelernt';
    $('#cal2').textContent=x.calibration.s2?'✓ gelernt':'nicht gelernt';
    $('#cal3').textContent=x.calibration.s3?'✓ gelernt':'nicht gelernt';
    $('#detSpeed').textContent=x.calibration.settling?'stabilisiert…':(x.calibration.detected?('Speed '+x.calibration.detected+(S.speed===0?' (bei AUS ignoriert)':'')):'—');
    $('#detConf').textContent=x.calibration.detected?(Number(x.calibration.confidence).toFixed(0)+' %'):'—';
    if(x.calibration.detectedOff){
      $('#detSpeed').textContent='Aus';
      $('#detConf').textContent='LEDs sehr niedrig';
    }
    $('#calNormal').textContent=x.calibration.normal?'✓ gelernt':'nicht gelernt';
    $('#calBreeze').textContent=x.calibration.breeze?'✓ gelernt':'nicht gelernt';
    $('#calNight').textContent=x.calibration.night?'✓ gelernt':'nicht gelernt';
    const mn=['Normal','Breeze','Night'];
    if($('#detWebState')) $('#detWebState').textContent=S.speed?('St'+S.speed+' / '+mn[S.mode]+' / '+(S.osc?'AN':'AUS')):'AUS';
    if($('#detTracking')){
      $('#detTracking').textContent=x.recognitionModel!=='FINAL_18P'?'nicht verfügbar':
        (x.calibration.settling?'stabilisiert…':(x.calibration.trackingMatch?'✓ passt zum Web-Soll':'⚠ Abweichung'));
    }
    if($('#detFit')){
      const wf=Number(x.calibration.webFit||0),gf=Number(x.calibration.globalFit||0);
      $('#detFit').textContent=x.calibration.settling?'—':('Web-Soll '+wf.toFixed(0)+' % · 18P '+gf.toFixed(0)+' %');
    }
    if($('#detAge')){
      const age=Number(x.calibration.ageMs||0);
      $('#detAge').textContent=x.calibration.settling?'—':(age<1000?age+' ms':(age/1000).toFixed(1)+' s');
    }
    $('#detMode').textContent=x.calibration.settling?'stabilisiert…':(x.calibration.detectedMode>=0?mn[x.calibration.detectedMode]:'—');
    $('#detModeConf').textContent=x.calibration.detectedMode>=0?(Number(x.calibration.modeConfidence).toFixed(0)+' %'):'—';
    if($('#detOsc')) $('#detOsc').textContent=x.calibration.settling?'stabilisiert…':
      (x.calibration.detectedOsc>=0?(x.calibration.detectedOsc?'AN':'AUS'):'—');
    if($('#detOscConf')) $('#detOscConf').textContent=x.calibration.detectedOsc>=0?
      (Number(x.calibration.oscConfidence).toFixed(0)+' %'):'—';
    if($('#detProfile')){
      const ds=Number(x.calibration.globalSpeed||0);
      const dm=Number(x.calibration.globalMode);
      const doo=Number(x.calibration.globalOsc);
      $('#detProfile').textContent=(ds>=1&&ds<=3&&dm>=0&&dm<=2&&doo>=0)?
        ('St'+ds+' / '+mn[dm]+' / '+(doo?'AN':'AUS')):'—';
    }
    if($('#detLivePct')){
      const p=x.calibration.livePct||[];
      $('#detLivePct').textContent=x.calibration.settling?'—':(p.length===5?p.map(v=>Number(v).toFixed(1)).join(' · '):'—');
    }
    if($('#detExpectedPct')){
      const p=x.calibration.expectedPct||[];
      $('#detExpectedPct').textContent=x.calibration.expectedValid&&p.length===5?
        p.map(v=>Number(v).toFixed(1)).join(' · '):'—';
    }
  }
  draw();

  // Wenn der ESP noch Hintergrundfenster sammelt, zeitnah erneut fragen,
  // statt auf den normalen 1,6-s-Takt zu warten.
  if(x.calibration && x.calibration.settling && S.speed>0)
    setTimeout(pollStatus,1100);
 }catch(e){
  clearTimeout(to);
 }finally{
  if(statusAbort===ctl)statusAbort=null;
  statusInFlight=false;
  if(statusPending){
    statusPending=false;
    setTimeout(pollStatus,60);
  }
 }
}
 const post=async (path,obj)=>{
 return api(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj||{})});
}
let queued=null;
 const run=async (action,optimistic=null,diagSettleMs=0)=>{
 if(busy){ queued={action,optimistic,diagSettleMs}; return; }

 const before={...S},beforeSleepSyncAt=sleepSyncAt;
 if(statusAbort){
   try{statusAbort.abort();}catch(e){}
   statusAbort=null;
 }
 statusPending=false;

 if(optimistic){
  try{optimistic();}catch(e){}
 }

 if(diagSettleMs>0){
   statusPollNotBefore=Date.now()+250;
   markDiagSettling();
 }else{
   statusPollNotBefore=Date.now()+120;
 }

 busy=true;
 draw();

 try{
  const result=await action();
  if(result&&result.state)applyServerState(result.state);
 }catch(e){
  console.log(e);
  S={...before};sleepSyncAt=beforeSleepSyncAt;
  statusPollNotBefore=0;
  setTimeout(pollStatus,80);
 }finally{
  busy=false;
  draw();

  const q=queued; queued=null;
  if(q){
    run(q.action,q.optimistic,q.diagSettleMs||0);
  }else{
    const wait=Math.max(80,statusPollNotBefore-Date.now()+30);
    setTimeout(pollStatus,wait);
  }
 }
}

document.querySelectorAll('.dial button').forEach(b=>b.onclick=()=>{
 const t=+b.dataset.s;

 // Im AUS-Zustand ist ausschließlich An/Aus bedienbar.
 // Erst einschalten, danach Speed 1/2/3 auswählen.
 if(S.speed===0 && t>0)return;

 if(t===0){
   const wasOn=S.speed>0;
   run(
    ()=>post('/api/power',{}),
    ()=>{
      if(wasOn){
        S.speed=0;
        S.osc=false;
        S.mode=0;
        S.sleep=0;
        S.sleepSec=0;sleepSyncAt=Date.now();
      }else{
        // Die Fan-Platine behaelt beim normalen AUS->AN Speed/Modus/Drehen.
        // Deshalb alle drei gemerkten Dimensionen sofort korrekt darstellen;
        // die Serverantwort bestaetigt denselben autoritativen Zustand danach.
        S.speed=remembered.speed;
        S.mode=remembered.mode;
        S.osc=remembered.osc;
        S.sleep=0;
        S.sleepSec=0;
        sleepSyncAt=Date.now();
      }
    },
    5400
   );
   return;
 }

 if(t===S.speed)return;

 run(
   ()=>post('/api/speed',{target:t}),
   ()=>{S.speed=t;},
   3400
 );
});
document.querySelectorAll('#modes button').forEach(b=>b.onclick=()=>{
 const t=+b.dataset.m;
 if(!S.speed||t===S.mode)return;

 run(
   ()=>post('/api/mode',{target:t}),
   ()=>{S.mode=t;},
   4400
 );
});
$('#osc').onclick=()=>{
 if(!S.speed)return;
 const turningOn=!S.osc;
 run(
   ()=>post('/api/osc',{}),
   ()=>{S.osc=!S.osc;},
   turningOn?8600:5000
 );
};

// Nur den internen/Web-Zustand korrigieren. KEIN physischer Tastendruck.
// Das ist fuer Bedienung direkt am Ventilator gedacht.
document.querySelectorAll('#manualSpeedButtons button').forEach(b=>b.onclick=()=>{
 const target=+b.dataset.manualSpeed;
 const mode=S.speed?S.mode:remembered.mode;
 const osc=S.speed?S.osc:remembered.osc;

 run(
   ()=>post('/api/manual-state',{speed:target,mode,osc:osc?1:0}),
   ()=>{
    if(target===0){
      remembered.mode=mode;
      remembered.osc=osc;
      S.speed=0;S.osc=false;S.mode=0;S.sleep=0;S.sleepSec=0;sleepSyncAt=Date.now();
    }else{
      S.speed=target;
      S.mode=mode;
      S.osc=osc;
      remembered.speed=target;
      remembered.mode=mode;
      remembered.osc=osc;
    }
   },
   900
 );
});

document.querySelectorAll('#manualModeButtons button').forEach(b=>b.onclick=()=>{
 const target=+b.dataset.manualMode;
 const speed=S.speed;
 const osc=S.speed?S.osc:remembered.osc;
 run(
   ()=>post('/api/manual-state',{speed,mode:target,osc:osc?1:0}),
   ()=>{
    if(speed){
      S.mode=target;
      remembered.speed=speed;
      remembered.mode=target;
      remembered.osc=osc;
    }else{
      remembered.mode=target;
      remembered.osc=osc;
    }
   },
   900
 );
});

$('#manualOsc').onclick=()=>{
 const speed=S.speed;
 const mode=S.speed?S.mode:remembered.mode;
 const currentOsc=S.speed?S.osc:remembered.osc;
 const targetOsc=!currentOsc;

 run(
   ()=>post('/api/manual-state',{speed,mode,osc:targetOsc?1:0}),
   ()=>{
    if(speed){
      S.osc=targetOsc;
      remembered.speed=speed;
      remembered.mode=mode;
      remembered.osc=targetOsc;
    }else{
      remembered.mode=mode;
      remembered.osc=targetOsc;
    }
   },
   900
 );
};

slp.oninput=()=>{
 const preview=+slp.value;
 S.sleep=preview;
 S.sleepSec=preview*60;
 sleepSyncAt=Date.now();
 paintTimer();
};
slp.onchange=()=>{
 const target=+slp.value;
 // oninput hat bereits eine Vorschau in S geschrieben. Fuer die generische
 // Rollback-Logik zuerst den letzten SERVER-bestaetigten Timer rekonstruieren.
 const elapsed=Math.max(0,(Date.now()-confirmedSleepAt)/1000);
 const prevSec=Math.max(0,Math.ceil(confirmedSleepSec-elapsed));
 S.sleep=prevSec?Math.ceil(prevSec/60):0;
 S.sleepSec=prevSec;
 sleepSyncAt=Date.now();
 run(
   ()=>post('/api/timer',{minutes:target}),
   ()=>{S.sleep=target;S.sleepSec=target*60;sleepSyncAt=Date.now();}
 );
};

$('#ledSync').onclick=async()=>{
 const btn=$('#ledSync');
 const old=btn.textContent;
 if(btn.disabled)return;
 btn.disabled=true;btn.textContent='3× Normal-Prioritäts-Abgleich…';
 try{
  const ctl=new AbortController();
  const to=setTimeout(()=>ctl.abort(),45000);
  const rr=await fetch('/api/sync-from-leds',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:'{}',
    signal:ctl.signal,
    cache:'no-store'
  });
  clearTimeout(to);
  const txt=await rr.text();
  if(!rr.ok)throw new Error(txt||('HTTP '+rr.status));
  const x=JSON.parse(txt);
  if(x&&x.state)applyServerState(x.state);
  draw();
  btn.textContent=x.detectedOff?'Erkannt: Ventilator AUS':('Status übernommen · Speed '+(x.speed||'?')+' · '+(x.modeName||'Modus')+' · Drehen '+(x.osc?'AN':'AUS'));
  setTimeout(pollStatus,180);
 }catch(e){
  btn.textContent='Abgleich fehlgeschlagen';
  setTimeout(pollStatus,250);
 }
 setTimeout(()=>{btn.textContent=old;btn.disabled=false;},2200);
};





 const learnSpeed=async (n)=>{
 calibrating=true;
 const msg=$('#calMsg');
 msg.textContent='Lerne Speed '+n+'… bitte einige Sekunden nichts umschalten.';
 const ctl=new AbortController();
 const timeout=setTimeout(()=>ctl.abort(),70000);
 try{
  const r=await fetch('/api/learn-speed',{
   method:'POST',
   headers:{'Content-Type':'application/json'},
   body:JSON.stringify({speed:n}),
   signal:ctl.signal,
   cache:'no-store'
  });
  clearTimeout(timeout);
  const t=await r.text();
  if(!r.ok)throw new Error(t||('HTTP '+r.status));
  msg.textContent='Speed '+n+' wurde erfolgreich gelernt und gespeichert.';
  setTimeout(pollStatus,200);
 }catch(e){
  clearTimeout(timeout);
  msg.textContent=e.name==='AbortError'
   ?'Lernen dauert zu lange / Verbindung unterbrochen.'
   :'Lernen fehlgeschlagen: '+e.message;
  setTimeout(pollStatus,500);
 }finally{
  calibrating=false;
  setTimeout(pollHealth,100);
 }
}
$('#learn1').onclick=()=>learnSpeed(1);
$('#learn2').onclick=()=>learnSpeed(2);
$('#learn3').onclick=()=>learnSpeed(3);
$('#learnOff').onclick=async()=>{
 const b=$('#learnOff'); b.disabled=true; const alt=b.textContent;
 b.textContent='Messe…'; calibrating=true;
 try{
  const r=await post('/api/learn-off',{});
  if(r&&r.state)applyServerState(r.state);
  b.textContent = r&&r.ok ? 'Aus-Zustand gelernt' : 'Fehlgeschlagen';
  draw();
 }catch(e){ b.textContent='Fehler · Ventilator wirklich AUS?'; }
 calibrating=false;
 setTimeout(()=>{b.textContent=alt;b.disabled=false;pollStatus();},1800);
};

 const learnMode=async (n,label)=>{
 calibrating=true;
 const msg=$('#calMsg');
 msg.textContent='Lerne '+label+'… bitte einige Sekunden nichts umschalten.';
 const ctl=new AbortController();
 const timeout=setTimeout(()=>ctl.abort(),70000);
 try{
  const r=await fetch('/api/learn-mode',{
   method:'POST',
   headers:{'Content-Type':'application/json'},
   body:JSON.stringify({mode:n}),
   signal:ctl.signal,
   cache:'no-store'
  });
  clearTimeout(timeout);
  const t=await r.text();
  if(!r.ok)throw new Error(t||('HTTP '+r.status));
  msg.textContent=label+' wurde erfolgreich gelernt und gespeichert.';
  setTimeout(pollStatus,200);
 }catch(e){
  clearTimeout(timeout);
  msg.textContent=e.name==='AbortError'?'Lernen dauert zu lange / Verbindung unterbrochen.':'Lernen fehlgeschlagen: '+e.message;
  setTimeout(pollStatus,500);
 }finally{
  calibrating=false;
  setTimeout(pollHealth,100);
 }
}
$('#learnNormal').onclick=()=>learnMode(0,'Normal');
$('#learnBreeze').onclick=()=>learnMode(1,'Breeze');
$('#learnNight').onclick=()=>learnMode(2,'Night');





$('#forgetSpeedCal').onclick=async()=>{
 const msg=$('#calMsg');
 if(!confirm('Speed-Kalibrierung für Stufe 1, 2 und 3 wirklich löschen?')) return;
 msg.textContent='Speed-Kalibrierung wird gelöscht…';
 try{
  await post('/api/forget-speed-calibration',{});
  msg.textContent='Speed-Kalibrierung gelöscht. Speed 1–3 können neu angelernt werden.';
  setTimeout(pollStatus,200);
 }catch(e){
  msg.textContent='Löschen der Speed-Kalibrierung fehlgeschlagen.';
 }
};

$('#forgetModeCal').onclick=async()=>{
 const msg=$('#calMsg');
 if(!confirm('Modus-Kalibrierung für Normal, Breeze und Night wirklich löschen?')) return;
 msg.textContent='Modus-Kalibrierung wird gelöscht…';
 try{
  await post('/api/forget-mode-calibration',{});
  msg.textContent='Modus-Kalibrierung gelöscht. Normal, Breeze und Night können neu angelernt werden.';
  setTimeout(pollStatus,200);
 }catch(e){
  msg.textContent='Löschen der Modus-Kalibrierung fehlgeschlagen.';
 }
};

$('#backupExport').onclick=async()=>{
 const msg=$('#backupMsg');
 msg.textContent='Backup wird erstellt…';
 try{
  const r=await fetch('/api/backup/export',{cache:'no-store'});
  if(!r.ok)throw new Error('HTTP '+r.status);
  const blob=await r.blob();
  const a=document.createElement('a');
  const url=URL.createObjectURL(blob);
  a.href=url;
  a.download='ventilator-efficient-balanced-v6-backup.json';
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
  msg.textContent='Backup exportiert.';
 }catch(e){
  msg.textContent='Export fehlgeschlagen: '+e.message;
 }
};

$('#backupImport').onclick=async()=>{
 const msg=$('#backupMsg');
 const f=$('#backupFile').files[0];
 if(!f){msg.textContent='Bitte zuerst eine Backup-JSON auswählen.';return;}
 msg.textContent='Backup wird importiert…';
 try{
  const txt=await f.text();
  const r=await fetch('/api/backup/import',{
   method:'POST',
   headers:{'Content-Type':'application/json'},
   body:txt
  });
  const t=await r.text();
  if(!r.ok)throw new Error(t||('HTTP '+r.status));
  const x=JSON.parse(t);
  if(x&&x.state)applyServerState(x.state);
  draw();
  msg.textContent='Backup erfolgreich importiert und CRC-verifiziert.';
  setTimeout(pollStatus,180);
 }catch(e){
  msg.textContent='Import fehlgeschlagen: '+e.message;
 }
};

$('#otaForm').onsubmit=async e=>{
 e.preventDefault();const f=$('#otaFile').files[0];if(!f)return;
 $('#otaMsg').textContent='Upload läuft…';
 try{
  const fd=new FormData();fd.append('firmware',f,f.name);
  const r=await fetch('/update',{method:'POST',body:fd});
  const t=await r.text();if(!r.ok)throw new Error(t);
  $('#otaMsg').textContent='Update erfolgreich. ESP32 startet neu…';
  setTimeout(()=>location.reload(),6500);
 }catch(err){$('#otaMsg').textContent='OTA fehlgeschlagen: '+err.message;}
};

 const showTab=(cfg)=>{
 document.getElementById('viewMain').hidden = cfg;
 document.getElementById('viewCfg').hidden = !cfg;
 if(cfg){
  // Alle Einstellungs-Panels zugeklappt zeigen - jedes Mal.
  document.querySelectorAll('#viewCfg details.panel').forEach(d=>d.open=false);
 }else{
  // Manuelles Setzen auf der Steuerungsseite immer zugeklappt starten.
  const mp=document.getElementById('manualStatePanel');
  if(mp)mp.open=false;
 }
 document.getElementById('tabMain').classList.toggle('on',!cfg);
 document.getElementById('tabCfg').classList.toggle('on',cfg);
 window.scrollTo(0,0);
 try{location.hash = cfg?'#einstellungen':'';}catch(e){}
}
document.getElementById('tabMain').onclick=()=>showTab(false);
document.getElementById('tabCfg').onclick=()=>showTab(true);
if(location.hash==='#einstellungen') showTab(true);

setInterval(pollHealth,3000);
setInterval(pollStatus,1600);
setTimeout(pollHealth,100);
setTimeout(pollStatus,350);
setInterval(paintTimer,1000);
draw();
</script></body></html>

)HTML";

const char* pinName(uint8_t pin) {
  if(pin==PIN_POWER) return "POWER";
  if(pin==PIN_SPEED) return "SPEED";
  if(pin==PIN_OSC)   return "OSC";
  if(pin==PIN_MODE)  return "MODE";
  return "?";
}

void pulsePin(uint8_t pin, uint8_t count=1) {
  count = constrain(count, 1, 8);
  Serial.printf("[TASTE] %s  x%u  (GPIO%u)\n", pinName(pin), count, pin);
  for(uint8_t i=0;i<count;i++) {
    digitalWrite(pin,HIGH);
    delay(PRESS_MS);
    digitalWrite(pin,LOW);
    if(i+1<count) delay(GAP_MS);
  }
}

int jsonInt(const String& body,const char* key,int fallback=-1) {
  // substring() wuerde bei jedem Aufruf den Rest des Bodys auf den Heap kopieren.
  // strtol arbeitet direkt auf dem Puffer und kann zusaetzlich melden,
  // ob ueberhaupt eine Zahl dastand.
  String needle=String("\"")+key+"\":";
  int p=body.indexOf(needle);
  if(p<0) return fallback;
  p+=needle.length();
  const char* start=body.c_str()+p;
  char* end=nullptr;
  long v=strtol(start,&end,10);
  if(end==start) return fallback;          // keine Ziffer gefunden
  return (int)v;
}

void cancelTimer() {
  timerDeadlineMs=0;
  state.sleepMin=0;
}

void powerOff() {
  if(!fanIsOn)return;

#if ENABLE_LED_FEEDBACK
  // Falls der Ventilator am Originaltaster bereits ausgeschaltet wurde,
  // keinen zweiten Power-Impuls senden. Die Messung ist absichtlich robust.
  FbStats cur[5];
  sampleProfileAverage(cur,4,30);
  if(ledsLookOff(cur)) {
    Serial.println("[POWER] physisch bereits AUS -> nur Softwarezustand korrigiert");

    fanIsOn=false;
    state.speed=0;
    state.osc=false;
    state.mode=0;
    cancelTimer();

    ledDetectReadyMs=0;
    resetStableDiagnostic();

    startupSyncUntilMs=0;
    startupSyncReadyMs=0;
    startupSpeedCandidate=0;
    startupSpeedHits=0;
    startupModeCandidate=-1;
    startupModeHits=0;
    manualLedSyncRequested=false;
    startupScanDone=false;

    if(!suppressFanStateSave) {
      bumpStateRevision();
      saveFanState();
    }
    return;
  }
#endif

  if(state.speed>=1&&state.speed<=3)state.lastSpeed=state.speed;
  state.lastOsc=state.osc;
  state.lastMode=state.mode;

  Serial.printf("[POWER] AUS  (gemerkt: Stufe %u, Drehen %s, Modus %u)\n",
                state.lastSpeed,state.lastOsc?"an":"aus",state.lastMode);

  pulsePin(PIN_POWER);

  fanIsOn=false;
  state.speed=0;
  state.osc=false;
  state.mode=0;
  cancelTimer();

  ledDetectReadyMs=0;
  resetStableDiagnostic();

  startupSyncUntilMs=0;
  startupSyncReadyMs=0;
  startupSpeedCandidate=0;
  startupSpeedHits=0;
  startupModeCandidate=-1;
  startupModeHits=0;
  manualLedSyncRequested=false;
  startupScanDone=false;

  if(!suppressFanStateSave) {
    bumpStateRevision();
    saveFanState(); // powerOn=false wird jetzt ebenfalls persistent gespeichert
  }
}


void powerOffFromTimer() {
  if(!fanIsOn)return;

  // Timer nutzt ausschliesslich den verbindlichen Web-/Befehlszustand.
  if(state.speed>=1&&state.speed<=3)state.lastSpeed=state.speed;
  state.lastOsc=state.osc;
  state.lastMode=state.mode;

  Serial.printf("[TIMER] AUS merke: Stufe %u, Drehen %s, Modus %u\n",
                state.lastSpeed,state.lastOsc?"an":"aus",state.lastMode);

  pulsePin(PIN_POWER);

  fanIsOn=false;
  state.speed=0;
  state.osc=false;
  state.mode=0;
  cancelTimer();

  ledDetectReadyMs=0;
  resetStableDiagnostic();
  startupSyncUntilMs=0;
  startupSyncReadyMs=0;
  startupScanDone=true;
  manualLedSyncRequested=false;

  bumpStateRevision();
  saveFanState();
}

void togglePower() {
  if(!fanIsOn) {
    Serial.printf("[POWER] EIN  (bekannter Zustand: Stufe %u, Drehen %s, Modus %u)\n",
                  state.lastSpeed,state.lastOsc?"an":"aus",state.lastMode);

    pulsePin(PIN_POWER);
    fanIsOn=true;

    // Bei normalem AUS->AN bewahrt die Fan-Platine ihren Zustand.
    state.speed=state.lastSpeed;
    state.osc=state.lastOsc;
    state.mode=state.lastMode;

    startupSyncReadyMs=0;
    startupSyncUntilMs=0;
    startupScanDone=true;
    startupSpeedCandidate=0;
    startupSpeedHits=0;
    startupModeCandidate=-1;
    startupModeHits=0;

    // Wenn Drehen bereits als letzter Zustand AN war, braucht der Motor
    // dieselbe lange Beruhigungszeit wie nach einem expliziten OSC-Befehl.
    armLedDiagnostic(state.osc?8000UL:LED_DETECT_SETTLE_MS);

    if(!suppressFanStateSave) {
      bumpStateRevision();
      saveFanState(); // wichtig fuer OTA-/Software-Neustart bei laufendem Fan
    }

    Serial.println("[STATE] Web-/Befehlszustand verbindlich; LED = Diagnose/Manuell-Sync");
  } else {
    powerOff();
  }
}


// Eine bewusste Bedienung ueber das Webinterface ist maßgeblich.
// Sie beendet einen noch laufenden Startscan, damit die LED-Erkennung
// den gerade gewaehlten Zustand nicht wieder ueberschreibt. Der Abgleich
// aus den LEDs bleibt ueber den Knopf "Status jetzt aus LED uebernehmen"
// jederzeit moeglich.
void endStartupScan(const char* grund) {
  if(startupScanDone && startupSyncUntilMs == 0) return;
  startupScanDone       = true;
  startupSyncUntilMs    = 0;
  startupSyncReadyMs    = 0;
  startupSpeedCandidate = 0;
  startupSpeedHits      = 0;
  startupModeCandidate  = -1;
  startupModeHits       = 0;
  Serial.printf("[SCAN] Startscan beendet (%s)\n", grund);
}

void setSpeed(uint8_t target) {
  target=constrain(target,(uint8_t)0,(uint8_t)3);

  if(target==0) {
    powerOff();
    return;
  }
  if(!fanIsOn)return;
  if(target==state.speed)return;

  endStartupScan("Speed ueber Web gewaehlt");

  uint8_t presses=(target+3-state.speed)%3;
  Serial.printf("[SPEED] %u -> %u  (%u Impulse)\n",state.speed,target,presses);
  if(presses)pulsePin(PIN_SPEED,presses);

  state.speed=target;
  state.lastSpeed=target;

  // Physische Stufenumschaltung braucht kurz Ruhe; danach neue Fenster.
  armLedDiagnostic(3000UL);

  if(!suppressFanStateSave) {
    bumpStateRevision();
    saveFanState();
  }
}

void setMode(uint8_t target) {
  target=constrain(target,(uint8_t)0,(uint8_t)2);
  if(!fanIsOn || target==state.mode)return;

  endStartupScan("Modus ueber Web gewaehlt");

  uint8_t presses=(target+3-state.mode)%3;
  Serial.printf("[MODE] %u -> %u  (%u Impulse)\n",state.mode,target,presses);
  if(presses)pulsePin(PIN_MODE,presses);

  state.mode=target;
  state.lastMode=target;

  armLedDiagnostic(4000UL);

  if(!suppressFanStateSave) {
    bumpStateRevision();
    saveFanState();
  }
}

void toggleOsc() {
  if(!fanIsOn)return;

  endStartupScan("Drehen ueber Web geschaltet");

  bool target=!state.osc;
  Serial.printf("[OSC] %s -> %s\n",state.osc?"AN":"AUS",target?"AN":"AUS");

  pulsePin(PIN_OSC);
  state.osc=target;
  state.lastOsc=target;

  // Motor/Mechanik: AN deutlich laenger als AUS.
  armLedDiagnostic(target?8000UL:4500UL);

  if(!suppressFanStateSave) {
    bumpStateRevision();
    saveFanState();
  }
}







#if ENABLE_LED_FEEDBACK

// Alte fancal2/fancal3/fancal4-Lade-/Savepfade sind absichtlich entfernt.
// FINAL_EFFICIENT_BALANCED_V6 + OSC-LINEAR nutzt den kompakten fancal10 A/B+CRC-Speicher.

void invalidateCombinedProfiles() {
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++) {
      comboSig[sp][md].valid=false;
      for(int os=0;os<2;os++)comboOscSig[sp][md][os].valid=false;
    }

  comboWValid=false;
  comboRichValid=false;
  comboGlobalSyncSafe=false;
  comboOscLinValid=false;
  comboLevelAlpha=0.0f;
  memset(comboRichPctQ,0,sizeof(comboRichPctQ));
  memset(comboOscLinW,0,sizeof(comboOscLinW));
  for(int md=0;md<3;md++){comboOscLinScale[md]=1.0f;comboOscLinBias[md]=0.0f;}
  for(int ch=0;ch<5;ch++)
    for(int q=0;q<RICH_CDF_LEVELS;q++)
      comboRichW[ch][q]=1.0f/(5.0f*RICH_CDF_LEVELS);
  comboShape=0.0f;
  comboMeanAlpha=0.0f;
  comboFitLimit=12.0f;
  for(int ch=0;ch<5;ch++) {
    comboW[ch]=1.0f;
    comboMeanW[ch]=1.0f;
  }

  if(!suppressCalibrationSave && !saveCompactComboCalibration(false))
    Serial.println("[CAL-NVS] WARNUNG: invalidierte Kalibrierung nicht dauerhaft gespeichert");
}


bool comboComplete() {
  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++)
      if(!comboSig[sp][md].valid) return false;
  return true;
}

// -------------------- Robuster Kalibrierungsspeicher A/B + CRC --------------------
// Zwei kompakte Slots statt hunderter einzelner NVS-Keys:
// - jeder Slot enthaelt die komplette Modellgeneration
// - CRC32 erkennt stille Korruption
// - Sequenznummer waehlt beim Boot den neuesten gueltigen Slot
// - beim Speichern bleibt der vorige Slot als Rueckfall erhalten
constexpr uint32_t COMPACT_COMBO_MAGIC=0x46363330UL; // "F630"
constexpr uint16_t COMPACT_COMBO_VERSION=9;         // FINAL_EFFICIENT_BALANCED_V6 + OSC_LINEAR_V1
constexpr const char* CAL_NVS_NS="fancal10";
constexpr const char* CAL_SLOT_A="A";
constexpr const char* CAL_SLOT_B="B";

uint32_t calStoreSequence=0;
uint32_t calStoreCrc=0;
uint8_t calStoreValidSlots=0;

struct CompactComboStore {
  uint32_t magic;
  uint16_t version;
  uint16_t bytes;
  uint32_t sequence;
  uint32_t crc32;

  uint8_t samplerVersion;
  uint8_t weightsValid;
  uint8_t reserved[2];

  int32_t thresholds[5];

  LearnedProfile speed[3];
  LearnedProfile mode[3];
  LearnedProfile off;
  LearnedProfile combo9[3][3];
  LearnedProfile combo18[3][3][2];

  float weights[5];
  float shape;

  float meanWeights[5];
  float meanAlpha;
  float levelAlpha;

  float richWeights[5][RICH_CDF_LEVELS];
  uint16_t richPctQ[3][3][2][5][RICH_CDF_LEVELS];
  uint8_t richValid;
  uint8_t globalSyncSafe;
  uint8_t richReserved[2];

  int16_t oscLinW[3][OSC_LINEAR_FEATURES];
  float oscLinScale[3];
  float oscLinBias[3];
  uint8_t oscLinValid;
  uint8_t oscLinReserved[3];

  float fitLimit;
};

// STACK-SAFE v6.4.5:
// CompactComboStore liegt knapp unter 4 KiB. Zwei oder mehr lokale Instanzen
// auf dem Arduino-loopTask-Stack koennen den ESP32-S3 bereits beim Booten
// zuruecksetzen, noch bevor WLAN/Webserver gestartet werden.
// Deshalb liegen die drei Arbeitsbuffer dauerhaft im BSS/RAM statt auf dem Stack.
static CompactComboStore calStoreBufA;
static CompactComboStore calStoreBufB;
static CompactComboStore calStoreVerifyBuf;

// Explizite Prototypen auch fuer den kompakten NVS-Datentyp.
uint32_t compactStoreCrc(const CompactComboStore& src);
void fillCompactComboStore(CompactComboStore& b);
bool validateCompactStore(CompactComboStore& b);
bool readCompactSlot(const char* key,CompactComboStore& out);
bool writeCompactSlot(const char* key,const CompactComboStore& b);

static_assert(sizeof(CompactComboStore)<4096,"CompactComboStore unexpectedly large");

uint32_t calibrationCrc32(const uint8_t* data,size_t len) {
  uint32_t crc=0xFFFFFFFFu;
  for(size_t i=0;i<len;i++) {
    crc^=data[i];
    for(uint8_t b=0;b<8;b++)
      crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
  }
  return crc^0xFFFFFFFFu;
}

uint32_t compactStoreCrc(const CompactComboStore& src) {
  // Kein 4-KiB-Temp-Objekt auf dem Stack: CRC direkt ueber die Quellstruktur,
  // wobei das crc32-Feld logisch als vier Nullbytes behandelt wird.
  const uint8_t* p=(const uint8_t*)&src;
  const size_t crcOff=(const uint8_t*)&src.crc32-p;
  uint32_t crc=0xFFFFFFFFu;

  for(size_t i=0;i<sizeof(src);i++) {
    uint8_t v=(i>=crcOff && i<crcOff+sizeof(src.crc32))?0u:p[i];
    crc^=v;
    for(uint8_t b=0;b<8;b++)
      crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
  }
  return crc^0xFFFFFFFFu;
}

void fillCompactComboStore(CompactComboStore& b) {
  memset(&b,0,sizeof(b));
  b.magic=COMPACT_COMBO_MAGIC;
  b.version=COMPACT_COMBO_VERSION;
  b.bytes=sizeof(CompactComboStore);
  b.sequence=calStoreSequence+1;
  b.samplerVersion=CAL_SAMPLER_VERSION;
  b.weightsValid=comboWValid?1:0;

  for(int ch=0;ch<5;ch++)b.thresholds[ch]=fbThresholds[ch];

  memcpy(b.speed,speedSig,sizeof(speedSig));
  memcpy(b.mode,modeSig,sizeof(modeSig));
  memcpy(&b.off,&offSig,sizeof(offSig));
  memcpy(b.combo9,comboSig,sizeof(comboSig));
  memcpy(b.combo18,comboOscSig,sizeof(comboOscSig));

  for(int ch=0;ch<5;ch++) {
    b.weights[ch]=comboW[ch];
    b.meanWeights[ch]=comboMeanW[ch];
  }
  b.shape=comboShape;
  b.meanAlpha=comboMeanAlpha;
  b.levelAlpha=comboLevelAlpha;
  memcpy(b.richWeights,comboRichW,sizeof(comboRichW));
  memcpy(b.richPctQ,comboRichPctQ,sizeof(comboRichPctQ));
  b.richValid=comboRichValid?1:0;
  b.globalSyncSafe=comboGlobalSyncSafe?1:0;
  memcpy(b.oscLinW,comboOscLinW,sizeof(comboOscLinW));
  memcpy(b.oscLinScale,comboOscLinScale,sizeof(comboOscLinScale));
  memcpy(b.oscLinBias,comboOscLinBias,sizeof(comboOscLinBias));
  b.oscLinValid=comboOscLinValid?1:0;
  b.fitLimit=comboFitLimit;

  b.crc32=compactStoreCrc(b);
}

bool validateCompactStore(CompactComboStore& b) {
  if(b.magic!=COMPACT_COMBO_MAGIC ||
     b.version!=COMPACT_COMBO_VERSION ||
     b.bytes!=sizeof(CompactComboStore) ||
     b.samplerVersion!=CAL_SAMPLER_VERSION)return false;

  uint32_t stored=b.crc32;
  return stored!=0 && stored==compactStoreCrc(b);
}

bool readCompactSlot(const char* key,CompactComboStore& out) {
  memset(&out,0,sizeof(out));

  prefs.begin(CAL_NVS_NS,true);
  size_t len=prefs.getBytesLength(key);
  size_t got=(len==sizeof(out))?prefs.getBytes(key,&out,sizeof(out)):0;
  prefs.end();

  return got==sizeof(out)&&validateCompactStore(out);
}

bool writeCompactSlot(const char* key,const CompactComboStore& b) {
  prefs.begin(CAL_NVS_NS,false);
  size_t n=prefs.putBytes(key,&b,sizeof(b));
  prefs.end();

  if(n!=sizeof(b)) {
    Serial.printf("[CAL-NVS] Slot %s write failed: %u/%u bytes\n",
                  key,(unsigned)n,(unsigned)sizeof(b));
    return false;
  }

  // Verifikation ebenfalls ueber globalen BSS-Buffer statt 4-KiB-Stackobjekt.
  if(!readCompactSlot(key,calStoreVerifyBuf) ||
     calStoreVerifyBuf.sequence!=b.sequence ||
     calStoreVerifyBuf.crc32!=b.crc32) {
    Serial.printf("[CAL-NVS] Slot %s verify FEHLER\n",key);
    return false;
  }

  Serial.printf("[CAL-NVS] Slot %s verifiziert · Seq %lu · %u Bytes\n",
                key,(unsigned long)b.sequence,(unsigned)sizeof(b));
  return true;
}

bool sequenceNewer(uint32_t a,uint32_t b) {
  return (int32_t)(a-b)>0;
}

void reclaimLegacyCalibrationNvs() {
  const char* oldNs[]={"fancal2","fancal3","fancal4","fancal5","fancal6","fancal7","fancal8","fancal9","fanfb"};
  for(const char* ns:oldNs) {
    prefs.begin(ns,false);
    prefs.clear();
    prefs.end();
  }
}

bool saveCompactComboCalibration(bool reclaimLegacy) {
  // Nur globale Arbeitsbuffer verwenden: keine mehreren ~4-KiB-Strukturen
  // auf dem loopTask-Stack.
  bool va=readCompactSlot(CAL_SLOT_A,calStoreBufA);
  bool vb=readCompactSlot(CAL_SLOT_B,calStoreBufB);
  bool hadCurrentGeneration=va||vb;

  const char* target=CAL_SLOT_A;
  uint32_t newest=calStoreSequence;

  if(va && (!vb || sequenceNewer(calStoreBufA.sequence,calStoreBufB.sequence))) {
    newest=calStoreBufA.sequence;
    target=CAL_SLOT_B;
  } else if(vb) {
    newest=calStoreBufB.sequence;
    target=CAL_SLOT_A;
  }

  calStoreSequence=newest;

  CompactComboStore& out=(strcmp(target,CAL_SLOT_A)==0)?calStoreBufA:calStoreBufB;
  fillCompactComboStore(out);

  bool ok=writeCompactSlot(target,out);

  if(!ok && reclaimLegacy) {
    Serial.println("[CAL-NVS] Speicher knapp -> alte Kalibrierungs-Namespaces werden freigegeben");
    reclaimLegacyCalibrationNvs();
    ok=writeCompactSlot(target,out);
  }

  compactComboLastSaveOk=ok;
  if(!ok)return false;

  calStoreSequence=out.sequence;
  calStoreCrc=out.crc32;
  compactComboLoaded=true;
  calibrationSamplerCurrent=true;

  // Bei der ersten Speicherung eine zweite CRC-verifizierte Kopie erzeugen.
  // Der jeweils andere globale Slotbuffer kann dafuer wiederverwendet werden.
  if(!hadCurrentGeneration) {
    const char* mirror=(strcmp(target,CAL_SLOT_A)==0)?CAL_SLOT_B:CAL_SLOT_A;
    CompactComboStore& second=(strcmp(mirror,CAL_SLOT_A)==0)?calStoreBufA:calStoreBufB;
    fillCompactComboStore(second);
    bool mirrorOk=writeCompactSlot(mirror,second);
    if(!mirrorOk && reclaimLegacy) {
      reclaimLegacyCalibrationNvs();
      mirrorOk=writeCompactSlot(mirror,second);
    }
    if(mirrorOk) {
      calStoreSequence=second.sequence;
      calStoreCrc=second.crc32;
    } else {
      Serial.println("[CAL-NVS] WARNUNG: erster Slot OK, A/B-Spiegelung fehlgeschlagen");
    }
  }

  // Redundanzzustand neu lesen/validieren, weiterhin ohne Stack-Grossobjekte.
  bool nva=readCompactSlot(CAL_SLOT_A,calStoreBufA);
  bool nvb=readCompactSlot(CAL_SLOT_B,calStoreBufB);
  calStoreValidSlots=(uint8_t)(nva?1:0)+(uint8_t)(nvb?1:0);

  CompactComboStore* best=nullptr;
  if(nva&&nvb)best=sequenceNewer(calStoreBufA.sequence,calStoreBufB.sequence)?&calStoreBufA:&calStoreBufB;
  else if(nva)best=&calStoreBufA;
  else if(nvb)best=&calStoreBufB;

  if(best) {
    calStoreSequence=best->sequence;
    calStoreCrc=best->crc32;
  }

  if(reclaimLegacy)reclaimLegacyCalibrationNvs();
  return calStoreValidSlots>=1;
}

bool loadCompactComboCalibration() {
  // Bootpfad stack-sicher: beide knapp 4-KiB-Slots liegen im globalen BSS.
  bool va=readCompactSlot(CAL_SLOT_A,calStoreBufA);
  bool vb=readCompactSlot(CAL_SLOT_B,calStoreBufB);

  if(!va && !vb) {
    compactComboLoaded=false;
    compactComboLastSaveOk=false;
    calibrationSamplerCurrent=false;
    calStoreSequence=0;
    calStoreCrc=0;
    calStoreValidSlots=0;
    return false;
  }

  CompactComboStore* best=nullptr;
  if(va && vb)best=sequenceNewer(calStoreBufA.sequence,calStoreBufB.sequence)?&calStoreBufA:&calStoreBufB;
  else best=va?&calStoreBufA:&calStoreBufB;

  for(int ch=0;ch<5;ch++)fbThresholds[ch]=constrain((int)best->thresholds[ch],0,4095);

  memcpy(speedSig,best->speed,sizeof(speedSig));
  memcpy(modeSig,best->mode,sizeof(modeSig));
  memcpy(&offSig,&best->off,sizeof(offSig));
  memcpy(comboSig,best->combo9,sizeof(comboSig));
  memcpy(comboOscSig,best->combo18,sizeof(comboOscSig));

  for(int ch=0;ch<5;ch++) {
    comboW[ch]=best->weights[ch];
    comboMeanW[ch]=best->meanWeights[ch];
  }
  comboShape=constrain(best->shape,0.0f,1.0f);
  // Cross-Boot-Robustheit: der 6.4.3-Rohdatensatz war mit 16/4 im
  // unabhaengigen Holdout mindestens so gut wie 64/16, ist aber deutlich
  // weniger empfindlich gegen sessionweite ADC-Pegelverschiebungen.
  comboMeanAlpha=constrain(best->meanAlpha,0.0f,16.0f);
  comboLevelAlpha=constrain(best->levelAlpha,0.0f,4.0f);
  memcpy(comboRichW,best->richWeights,sizeof(comboRichW));
  memcpy(comboRichPctQ,best->richPctQ,sizeof(comboRichPctQ));
  comboRichValid=best->richValid!=0;
  comboGlobalSyncSafe=best->globalSyncSafe!=0;
  memcpy(comboOscLinW,best->oscLinW,sizeof(comboOscLinW));
  memcpy(comboOscLinScale,best->oscLinScale,sizeof(comboOscLinScale));
  memcpy(comboOscLinBias,best->oscLinBias,sizeof(comboOscLinBias));
  comboOscLinValid=best->oscLinValid!=0;
  comboFitLimit=max(best->fitLimit,0.25f);
  comboWValid=best->weightsValid!=0;
  if(!comboRichValid) {
    compactComboLoaded=false;
    calibrationSamplerCurrent=false;
    return false;
  }

  calStoreSequence=best->sequence;
  calStoreCrc=best->crc32;
  calStoreValidSlots=(uint8_t)(va?1:0)+(uint8_t)(vb?1:0);
  compactComboLoaded=true;
  compactComboLastSaveOk=true;
  calibrationSamplerCurrent=true;

  Serial.printf("[CAL-NVS] FINAL geladen · Seq %lu · Speed=%u/%u/%u · Mode=%u/%u/%u · AUS=%u · 9P=%u/9 · 18P=%u/18\n",
                (unsigned long)calStoreSequence,
                speedSig[0].valid?1:0,speedSig[1].valid?1:0,speedSig[2].valid?1:0,
                modeSig[0].valid?1:0,modeSig[1].valid?1:0,modeSig[2].valid?1:0,
                offSig.valid?1:0,comboValidCount(),comboOscValidCount());
  return true;
}

void clearCompactComboCalibration() {
  prefs.begin(CAL_NVS_NS,false);
  prefs.clear();
  prefs.end();

  compactComboLoaded=false;
  compactComboLastSaveOk=false;
  calibrationSamplerCurrent=false;
  calStoreSequence=0;
  calStoreCrc=0;
  calStoreValidSlots=0;
  comboRichValid=false;
  comboGlobalSyncSafe=false;
  comboOscLinValid=false;
  comboLevelAlpha=0.0f;
  memset(comboRichPctQ,0,sizeof(comboRichPctQ));
  memset(comboOscLinW,0,sizeof(comboOscLinW));
  for(int md=0;md<3;md++){comboOscLinScale[md]=1.0f;comboOscLinBias[md]=0.0f;}
}


void invalidateRuntimeCalibrationForSampler() {
  // Alte Modellgenerationen werden im RAM bewusst NICHT weiterverwendet.
  for(int s=0;s<3;s++){speedSig[s].valid=false;modeSig[s].valid=false;}
  offSig.valid=false;
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++) {
      comboSig[sp][md].valid=false;
      for(int os=0;os<2;os++)comboOscSig[sp][md][os].valid=false;
    }

  comboWValid=false;
  comboRichValid=false;
  comboGlobalSyncSafe=false;
  comboLevelAlpha=0.0f;
  memset(comboRichPctQ,0,sizeof(comboRichPctQ));
  for(int ch=0;ch<5;ch++)
    for(int q=0;q<RICH_CDF_LEVELS;q++)comboRichW[ch][q]=1.0f/(5.0f*RICH_CDF_LEVELS);
  comboShape=0.0f;
  comboMeanAlpha=0.0f;
  comboFitLimit=12.0f;
  for(int ch=0;ch<5;ch++) {
    comboW[ch]=1.0f;
    comboMeanW[ch]=1.0f;
    fbThresholds[ch]=FB_DEFAULT_THR[ch];
  }

  compactComboLoaded=false;
  calibrationSamplerCurrent=false;
  calStoreSequence=0;
  calStoreCrc=0;
  calStoreValidSlots=0;
  resetStableDiagnostic();

  Serial.println("[CAL] Keine kompatible FINAL-RICH-CDF-Kalibrierung -> kompletter Messlauf erforderlich");
}

void saveOffCalibration() {
  if(!suppressCalibrationSave)
    saveCompactComboCalibration(false);
}

void loadOffCalibration() {
  prefs.begin("fancal2", true);
  offSig.valid = prefs.getBool("ov", false);
  for(int ch=0; ch<5; ch++) {
    String pk="op"+String(ch);
    String mk="om"+String(ch);
    offSig.avgPct[ch]  = prefs.getFloat(pk.c_str(), 0.0f);
    offSig.avgMean[ch] = prefs.getFloat(mk.c_str(), 0.0f);
  }
  prefs.end();
}

// Misst den aktuellen Zustand und legt ihn als AUS-Referenz ab.
bool learnOffSignature() {
#if ENABLE_LED_FEEDBACK
  FbStats cur[5];
  sampleProfileAverage(cur,10,55);

  for(int ch=0;ch<5;ch++) {
    offSig.avgPct[ch]=cur[ch].abovePct;
    offSig.avgMean[ch]=cur[ch].avg;
  }

  offSig.valid=true;
  if(!saveCompactComboCalibration(false)) {
    Serial.println("[CAL-NVS] WARNUNG: AUS-Profil nicht dauerhaft gespeichert");
    return false;
  }

  Serial.printf("[AUS] FINAL_EFFICIENT_BALANCED_V6-Profil: %.1f %.1f %.1f %.1f %.1f\n",
                offSig.avgPct[0],offSig.avgPct[1],offSig.avgPct[2],
                offSig.avgPct[3],offSig.avgPct[4]);
  return true;
#else
  return false;
#endif
}


void saveSpeedCalibration(int idx) {
  if(idx<0||idx>2)return;
  if(!suppressCalibrationSave)
    saveCompactComboCalibration(false);
}

void saveModeCalibration(int idx) {
  if(idx<0||idx>2)return;
  if(!suppressCalibrationSave)
    saveCompactComboCalibration(false);
}


float robustMedianOfMeans(float* v, int n) {
  // 6 Gruppen; Median der Gruppenmittelwerte unterdrueckt einzelne
  // Stoerfenster wesentlich besser als ein einfacher Gesamtmittelwert.
  const int G = 6;
  float gm[G] = {0,0,0,0,0,0};
  int gc[G] = {0,0,0,0,0,0};

  for(int i=0;i<n;i++) {
    int g = i % G;
    gm[g] += v[i];
    gc[g]++;
  }
  for(int g=0;g<G;g++) if(gc[g]) gm[g] /= gc[g];

  // kleine Insertion-Sortierung
  for(int i=1;i<G;i++) {
    float x=gm[i]; int j=i-1;
    while(j>=0 && gm[j] > x) { gm[j+1]=gm[j]; j--; }
    gm[j+1]=x;
  }
  return (gm[2] + gm[3]) * 0.5f;
}

bool learnProfile(LearnedProfile& p) {
  // Einzel-Lernen nutzt denselben FINAL_EFFICIENT_BALANCED_V6-Sampler wie die finale 18P-Analyse.
  constexpr int WINDOWS=12;
  double pct[5]={0,0,0,0,0};
  double meanv[5]={0,0,0,0,0};

  for(int w=0;w<WINDOWS;w++) {
    FbStats cur[5];
    sampleProfileWindow(cur);
    for(int ch=0;ch<5;ch++) {
      pct[ch]+=cur[ch].abovePct;
      meanv[ch]+=cur[ch].avg;
    }
    delay(45);
    yield();
  }

  for(int ch=0;ch<5;ch++) {
    p.avgPct[ch]=pct[ch]/WINDOWS;
    p.avgMean[ch]=meanv[ch]/WINDOWS;
  }
  p.valid=true;
  return true;
}

bool learnSpeedSignature(int speedNo) {
  if(speedNo<1 || speedNo>3)return false;
  int idx=speedNo-1;
  bool ok=learnProfile(speedSig[idx]);
  if(ok)saveSpeedCalibration(idx);
  return ok;
}

bool learnModeSignature(int modeNo) {
  if(modeNo<0 || modeNo>2)return false;
  bool ok=learnProfile(modeSig[modeNo]);
  if(ok)saveModeCalibration(modeNo);
  return ok;
}

float normalizedMean(const float means[5], int ch) {
  float sum = 0.0f;
  for(int i=0;i<5;i++) sum += means[i];
  if(sum < 1.0f) return 0.0f;
  return (means[ch] / sum) * 100.0f;
}

float currentNormalizedMean(const FbStats current[5], int ch) {
  float sum = 0.0f;
  for(int i=0;i<5;i++) sum += current[i].avg;
  if(sum < 1.0f) return 0.0f;
  return ((float)current[ch].avg / sum) * 100.0f;
}

float signatureDistanceWeighted(const LearnedProfile& sig,
                                const FbStats current[5],
                                const float weights[5]) {
  float d = 0.0f;

  for(int ch=0;ch<5;ch++) {
    // 1) Pulsanteil: relativ robust gegen globale Pegelverschiebungen.
    float dp = (float)current[ch].abovePct - sig.avgPct[ch];

    // 2) Normalisierter Mittelwert: betrachtet das Verhältnis der Kanäle
    //    zueinander statt absolute ADC-Pegel. Das macht die Erkennung
    //    unempfindlicher gegen Versorgungsschwankungen / gemeinsame Drift.
    float pMean = normalizedMean(sig.avgMean, ch);
    float cMean = currentNormalizedMean(current, ch);
    float dn = (cMean - pMean) * 2.0f;

    d += weights[ch] * (dp*dp + dn*dn);
  }

  return d;
}

int detectProfilesWeightedFromCurrent(LearnedProfile profiles[3],
                                      const FbStats cur[5],
                                      const float weights[5],
                                      float* confidenceOut) {
  float best = 1e30f;
  float second = 1e30f;
  int bestIdx = -1;

  for(int s=0;s<3;s++) {
    if(!profiles[s].valid) continue;

    float d = signatureDistanceWeighted(profiles[s], cur, weights);

    if(d < best) {
      second = best;
      best = d;
      bestIdx = s;
    } else if(d < second) {
      second = d;
    }
  }

  if(confidenceOut) {
    if(bestIdx < 0 || second >= 1e29f) {
      *confidenceOut = 0.0f;
    } else {
      // Kombination aus Abstand zum Zweitbesten und absoluter Match-Qualität.
      float margin = 1.0f - (best / (second + 0.0001f));
      if(margin < 0) margin = 0;
      if(margin > 1) margin = 1;

      float quality = 1.0f / (1.0f + sqrtf(best) / 90.0f);
      float c = margin * quality;
      if(c < 0) c = 0;
      if(c > 1) c = 1;
      *confidenceOut = c * 100.0f;
    }
  }

  return bestIdx;
}

// Kompatibilitätsfunktion für vorhandenen Code.
int detectProfilesFromCurrent(LearnedProfile profiles[3],
                              const FbStats cur[5],
                              float* confidenceOut) {
  const float equalWeights[5] = {1,1,1,1,1};
  return detectProfilesWeightedFromCurrent(profiles,cur,equalWeights,confidenceOut);
}

int detectProfiles(LearnedProfile profiles[3], float* confidenceOut) {
  FbStats cur[5];
  sampleProfileWindow(cur);
  const float equalWeights[5] = {1,1,1,1,1};
  return detectProfilesWeightedFromCurrent(profiles,cur,equalWeights,confidenceOut);
}

// Klassifiziert Stufe UND Modus in einem Schritt gegen alle neun Profile.
// Liefert -1, wenn nicht alle neun gelernt sind.
int detectCombo(const FbStats cur[5],int* speedOut,int* modeOut,
                float* speedConfOut,float* modeConfOut) {
  if(speedOut)*speedOut=0;
  if(modeOut)*modeOut=-1;
  if(speedConfOut)*speedConfOut=0.0f;
  if(modeConfOut)*modeConfOut=0.0f;
  if(!comboComplete())return -1;

  float dist[3][3];
  float best=1e30f;
  int bs=-1,bm=-1;

  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++) {
      float d=comboDistanceToProfile(comboSig[sp][md],cur);
      dist[sp][md]=d;
      if(d<best){best=d;bs=sp;bm=md;}
    }

  if(bs<0)return -1;

  float altSpeed=1e30f,altMode=1e30f;
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++) {
      if(sp!=bs)altSpeed=min(altSpeed,dist[sp][md]);
      if(md!=bm)altMode=min(altMode,dist[sp][md]);
    }

  auto conf=[&](float other)->float{
    if(other>1e29f)return 0.0f;
    float rb=sqrtf(max(best,0.0f));
    float ro=sqrtf(max(other,0.0f));
    float margin=(ro>0.0001f)?((ro-rb)/ro):0.0f;
    margin=constrain(margin,0.0f,1.0f);
    return margin*(comboFitQualityPct(best)/100.0f)*100.0f;
  };

  if(speedOut)*speedOut=bs+1;
  if(modeOut)*modeOut=bm;
  if(speedConfOut)*speedConfOut=conf(altSpeed);
  if(modeConfOut)*modeConfOut=conf(altMode);
  return bs+1;
}


// Finale Klassifikation gegen alle 18 Profile.
// WICHTIG:
// Die Trennschaerfe-Analyse bewertet den Abstand zwischen VOLLSTAENDIGEN
// 18P-Zustaenden. Deshalb muss auch die Laufzeit-Erkennung EINEN globalen
// Gewinner aus den 18 Profilen waehlen.
//
// Eine getrennte Wahl von Speed/Modus/Drehen ist falsch: dabei koennen z.B.
// "Speed 3" aus einem Profil und "Drehen AUS" aus einem anderen Profil
// zusammengebaut werden, obwohl kein einziges 18P-Profil diese Kombination
// als besten Treffer hatte.
int detectComboOsc(const FbStats cur[5],
                   int* speedOut,int* modeOut,int* oscOut,
                   float* speedConfOut,float* modeConfOut,float* oscConfOut) {
  if(speedOut)*speedOut=0;
  if(modeOut)*modeOut=-1;
  if(oscOut)*oscOut=-1;
  if(speedConfOut)*speedConfOut=0.0f;
  if(modeConfOut)*modeConfOut=0.0f;
  if(oscConfOut)*oscConfOut=0.0f;
  if(!comboOscComplete() || !comboRichValid)return -1;

  float dist[3][3][2];
  float best=1e30f;
  int bs=-1,bm=-1,bo=-1;

  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++)
      for(int os=0;os<2;os++) {
        float d=comboRichDistanceToState(sp,md,os,cur);
        dist[sp][md][os]=d;
        if(d<best){best=d;bs=sp;bm=md;bo=os;}
      }

  if(bs<0)return -1;

  float altSpeed=1e30f,altMode=1e30f,altOsc=1e30f;
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++)
      for(int os=0;os<2;os++) {
        float d=dist[sp][md][os];
        if(sp!=bs&&d<altSpeed)altSpeed=d;
        if(md!=bm&&d<altMode)altMode=d;
        if(os!=bo&&d<altOsc)altOsc=d;
      }

  auto conf=[&](float other)->float{
    if(other>1e29f)return 0.0f;
    float rb=sqrtf(max(best,0.0f));
    float ro=sqrtf(max(other,0.0f));
    float margin=(ro>0.0001f)?((ro-rb)/ro):0.0f;
    margin=constrain(margin,0.0f,1.0f);
    return margin*(comboFitQualityPct(best)/100.0f)*100.0f;
  };

  if(speedOut)*speedOut=bs+1;
  if(modeOut)*modeOut=bm;
  if(oscOut)*oscOut=bo;
  if(speedConfOut)*speedConfOut=conf(altSpeed);
  if(modeConfOut)*modeConfOut=conf(altMode);
  if(oscConfOut)*oscConfOut=conf(altOsc);
  return bs+1;
}


float comboRichDistanceToState(int sp,int md,int os,const FbStats cur[5]) {
  if(!comboRichValid || sp<0||sp>2||md<0||md>2||os<0||os>1)
    return 1e30f;

  float d=0.0f;

  for(int q=0;q<RICH_CDF_LEVELS;q++) {
    float tm=0.0f,rm=0.0f;
    for(int ch=0;ch<5;ch++) {
      tm+=cur[ch].cdfPct[q];
      rm+=richDecodePct(comboRichPctQ[sp][md][os][ch][q]);
    }
    tm/=5.0f;rm/=5.0f;

    for(int ch=0;ch<5;ch++) {
      float t=cur[ch].cdfPct[q]-comboShape*tm;
      float r=richDecodePct(comboRichPctQ[sp][md][os][ch][q])-comboShape*rm;
      float e=t-r;
      d+=comboRichW[ch][q]*e*e;
    }
  }

  float pMean[5],cMean[5];
  normalizedMeanFeature(comboOscSig[sp][md][os].avgMean,pMean);
  normalizedMeanFeature(cur,cMean);
  for(int ch=0;ch<5;ch++) {
    float e=cMean[ch]-pMean[ch];
    d+=comboMeanAlpha*comboMeanW[ch]*e*e;
  }

  float pLevel=rawLevelFeature(comboOscSig[sp][md][os].avgMean);
  float cLevel=rawLevelFeature(cur);
  float el=cLevel-pLevel;
  d+=comboLevelAlpha*el*el;

  return d;
}

float comboOscLinearScore(int sp,int md,const FbStats cur[5]) {
  if(!comboOscLinValid || sp<0||sp>2||md<0||md>2)return 0.0f;

  double acc=comboOscLinBias[md];
  int fi=0;
  const float scale=comboOscLinScale[md];

  for(int ch=0;ch<5;ch++)
    for(int q=0;q<RICH_CDF_LEVELS;q++,fi++)
      acc+=(double)scale*(double)comboOscLinW[md][fi]*(double)cur[ch].cdfPct[q];

  for(int s=0;s<3;s++,fi++)
    if(s==sp)acc+=(double)scale*(double)comboOscLinW[md][fi];

  return (float)acc;
}

int comboOscLinearDetect(int sp,int md,const FbStats cur[5],float* confidenceOut) {
  if(confidenceOut)*confidenceOut=0.0f;
  if(!comboOscLinValid || sp<0||sp>2||md<0||md>2)return -1;

  float score=comboOscLinearScore(sp,md,cur);
  // Logistische Distanz zur Entscheidungsgrenze. Ein kleiner Wert ist bewusst
  // nur geringe Diagnose-Sicherheit, auch wenn die Klasse selbst eindeutig ist.
  float z=constrain(score,-12.0f,12.0f);
  float p=1.0f/(1.0f+expf(-z));
  if(confidenceOut)*confidenceOut=fabsf(p-0.5f)*200.0f;
  return score>=0.0f?1:0;
}

// Cross-Boot-robuste Formmetrik fuer die fuenf Live-LED-%.
// Gemeinsamer Offset und gemeinsame positive Skalierung verschwinden durch
// Zentrieren + Normieren. Die gespeicherten Rich-CDF-Profile bleiben erhalten.
float comboPctShapeDistanceToProfile(const LearnedProfile& p,const FbStats cur[5]) {
  if(!p.valid)return 1e30f;

  float mp=0.0f,mc=0.0f;
  for(int ch=0;ch<5;ch++){mp+=p.avgPct[ch];mc+=cur[ch].abovePct;}
  mp*=0.2f;mc*=0.2f;

  float dot=0.0f,np=0.0f,nc=0.0f;
  for(int ch=0;ch<5;ch++) {
    float a=p.avgPct[ch]-mp;
    float b=cur[ch].abovePct-mc;
    dot+=a*b;np+=a*a;nc+=b*b;
  }

  if(np<0.0001f || nc<0.0001f)return 2.0f;
  float corr=dot/sqrtf(np*nc);
  corr=constrain(corr,-1.0f,1.0f);
  return 1.0f-corr;
}

// Speed-/Modus-Formvergleich ignoriert absichtlich Drehen. Dadurch kann eine
// unstabile AUS/AN-Amplitude Stufe oder Modus nicht alleine umwerfen.
float comboPctShapeDistanceToSpeedMode(int sp,int md,const FbStats cur[5]) {
  if(sp<0||sp>2||md<0||md>2)return 1e30f;
  float d0=comboPctShapeDistanceToProfile(comboOscSig[sp][md][0],cur);
  float d1=comboPctShapeDistanceToProfile(comboOscSig[sp][md][1],cur);
  return min(d0,d1);
}

static float pctShapeConfidence(float best,float second) {
  if(second>1e29f)return 0.0f;
  return constrain((second-best)/(second+0.02f),0.0f,1.0f)*100.0f;
}

static float pctShapeFit(float d) {
  // Absichtlich tolerant gegen die beobachtete Cross-Boot-Verschiebung.
  return constrain(1.0f-d/0.25f,0.0f,1.0f)*100.0f;
}

int detectSpeedPctShapeForMode(int md,const FbStats cur[5],float* confidenceOut,float* fitOut) {
  if(confidenceOut)*confidenceOut=0.0f;
  if(fitOut)*fitOut=0.0f;
  if(md<0||md>2 || !comboOscComplete())return -1;

  float d[3];
  for(int sp=0;sp<3;sp++)d[sp]=comboPctShapeDistanceToSpeedMode(sp,md,cur);
  int best=0;if(d[1]<d[best])best=1;if(d[2]<d[best])best=2;
  float second=1e30f;for(int sp=0;sp<3;sp++)if(sp!=best)second=min(second,d[sp]);

  if(confidenceOut)*confidenceOut=pctShapeConfidence(d[best],second);
  if(fitOut)*fitOut=pctShapeFit(d[best]);
  return best;
}

int detectModePctShapeForSpeed(int sp,const FbStats cur[5],float* confidenceOut,float* fitOut) {
  if(confidenceOut)*confidenceOut=0.0f;
  if(fitOut)*fitOut=0.0f;
  if(sp<0||sp>2 || !comboOscComplete())return -1;

  float d[3];
  for(int md=0;md<3;md++)d[md]=comboPctShapeDistanceToSpeedMode(sp,md,cur);
  int best=0;if(d[1]<d[best])best=1;if(d[2]<d[best])best=2;
  float second=1e30f;for(int md=0;md<3;md++)if(md!=best)second=min(second,d[md]);

  if(confidenceOut)*confidenceOut=pctShapeConfidence(d[best],second);
  if(fitOut)*fitOut=pctShapeFit(d[best]);
  return best;
}


// Manueller Zustandsabgleich: sammelt nur cross-boot-robuste Merkmale.
// Speed/Modus basieren auf der relativen 5-Kanal-Form. Drehen verwendet
// bevorzugt OSC-LINEAR. Der globale 18P-Fit wird hier absichtlich NICHT
// als Freigabebedingung benutzt.
void manualSyncAccumulateBatch(const FbStats cur[5],
                               float pairDist[3][3],
                               uint8_t speedVotes[3][3],
                               uint8_t oscOnVotes[3][3],
                               uint8_t oscOffVotes[3][3],
                               float oscScoreSum[3][3]) {
  float d[3][3];

  for(int sp=0;sp<3;sp++) {
    for(int md=0;md<3;md++) {
      d[sp][md]=comboPctShapeDistanceToSpeedMode(sp,md,cur);
      pairDist[sp][md]+=d[sp][md];

      float oscScore=0.0f;
      if(comboOscLinValid) {
        oscScore=comboOscLinearScore(sp,md,cur);
      } else {
        float d0=comboRichDistanceToState(sp,md,0,cur);
        float d1=comboRichDistanceToState(sp,md,1,cur);
        // Positiv bedeutet AN, negativ AUS.
        oscScore=sqrtf(max(d0,0.0f))-sqrtf(max(d1,0.0f));
      }
      oscScoreSum[sp][md]+=oscScore;
      if(oscScore>=0.0f)oscOnVotes[sp][md]++;
      else oscOffVotes[sp][md]++;
    }
  }

  // Pro Modus eine Speed-Stimme. Dadurch koennen wir spaeter den bekannten
  // Web-Modus als Prior verwenden, ohne eine globale 18P-Entscheidung zu
  // erzwingen.
  for(int md=0;md<3;md++) {
    int bs=0;
    if(d[1][md]<d[bs][md])bs=1;
    if(d[2][md]<d[bs][md])bs=2;
    speedVotes[md][bs]++;
  }
}


float comboDistanceToProfile(const LearnedProfile& p,const FbStats cur[5]) {
  float pp[5],cc[5],pf[5],cf[5];
  for(int ch=0;ch<5;ch++){pp[ch]=p.avgPct[ch];cc[ch]=cur[ch].abovePct;}
  comboFeature(pp,comboShape,pf);
  comboFeature(cc,comboShape,cf);

  float pMean[5],cMean[5];
  normalizedMeanFeature(p.avgMean,pMean);
  normalizedMeanFeature(cur,cMean);

  float d=0.0f;
  for(int ch=0;ch<5;ch++) {
    float ePct=cf[ch]-pf[ch];
    float eMean=cMean[ch]-pMean[ch];

    float wp=comboWValid?comboW[ch]:1.0f;
    float wm=comboWValid?comboMeanW[ch]:1.0f;

    d += wp*ePct*ePct;
    d += comboMeanAlpha*wm*eMean*eMean;
  }
  return d;
}

float comboFitQualityPct(float distanceSq) {
  float root=sqrtf(max(distanceSq,0.0f));
  float lim=max(comboFitLimit,0.25f);

  // Innerhalb des per Holdout bestimmten Trainingsbereichs keine Strafe.
  if(root<=lim)return 100.0f;

  // Zwischen 1x und 2x Trainingsgrenze linear auf 0.
  float q=1.0f-(root-lim)/lim;
  return constrain(q,0.0f,1.0f)*100.0f;
}

float contextualConfidence(float best,float second) {
  if(second>1e29f)return 0.0f;

  float rb=sqrtf(max(best,0.0f));
  float rs=sqrtf(max(second,0.0f));
  float margin=(rs>0.0001f)?((rs-rb)/rs):0.0f;
  margin=constrain(margin,0.0f,1.0f);

  float fit=comboFitQualityPct(best)/100.0f;
  return margin*fit*100.0f;
}

// Plausibilitaetspruefung gegen den vom Web bewusst gesetzten Zustand.
// Beispiel Stufe1/Normal: "Drehen erkannt" vergleicht NUR
// St1/Normal/AUS gegen St1/Normal/AN. Genau deshalb ist Drehen hier viel
// eindeutiger als bei einer komplett kontextfreien Suche ueber alle 18 Profile.
//
// Wichtig: Das ist nur Diagnose. "Status aus LEDs uebernehmen" benutzt
// weiterhin den globalen 18P-Klassifikator und kann alle drei Werte korrigieren.
void detectContextual18(const FbStats cur[5],
                        int* sp,float* sc,int* md,float* mc,int* os,float* oc) {
  if(sp)*sp=0;
  if(md)*md=-1;
  if(os)*os=-1;
  if(sc)*sc=0;
  if(mc)*mc=0;
  if(oc)*oc=0;
  if(!comboOscComplete() || !comboRichValid || !fanIsOn || state.speed<1 || state.speed>3 || state.mode>2)return;

  int knownSp=state.speed-1,knownMd=state.mode,knownOs=state.osc?1:0;

  float ds[3];
  for(int s=0;s<3;s++)ds[s]=comboRichDistanceToState(s,knownMd,knownOs,cur);
  int bs=0;if(ds[1]<ds[bs])bs=1;if(ds[2]<ds[bs])bs=2;
  float ss=1e30f;for(int s=0;s<3;s++)if(s!=bs)ss=min(ss,ds[s]);
  float richSpeedConf=contextualConfidence(ds[bs],ss);

  float dm[3];
  for(int m=0;m<3;m++)dm[m]=comboRichDistanceToState(knownSp,m,knownOs,cur);
  int bm=0;if(dm[1]<dm[bm])bm=1;if(dm[2]<dm[bm])bm=2;
  float sm=1e30f;for(int m=0;m<3;m++)if(m!=bm)sm=min(sm,dm[m]);
  float richModeConf=contextualConfidence(dm[bm],sm);

  float shapeSpeedConf=0.0f,shapeSpeedFit=0.0f;
  float shapeModeConf=0.0f,shapeModeFit=0.0f;
  int shapeSp=detectSpeedPctShapeForMode(knownMd,cur,&shapeSpeedConf,&shapeSpeedFit);
  int shapeMd=detectModePctShapeForSpeed(knownSp,cur,&shapeModeConf,&shapeModeFit);

  int outSp=bs;
  int outMd=bm;
  float outSc=richSpeedConf;
  float outMc=richModeConf;

  if(knownMd==0) {
    // NORMAL ist der Hauptbetrieb. Die relative Kanalform ist hier primaer,
    // weil sie den beobachteten sessionweiten LED-%-Offset entfernt.
    if(shapeSp>=0) {
      outSp=shapeSp;
      outSc=shapeSpeedConf;
      if(outSp!=knownSp && shapeSpeedConf<55.0f) {
        outSp=knownSp;
        outSc=0.0f;
      }
    }

    // Webzustand bleibt autoritativ. Eine unsichere LED-Modusabweichung soll
    // im fast ausschliesslich genutzten Normalbetrieb keine Fehlwarnung sein.
    outMd=knownMd;
    outMc=(shapeMd==knownMd)?shapeModeConf:0.0f;
  } else {
    // Breeze/Nacht: Rich bleibt primaer; die relative Form darf einen
    // offensichtlichen Cross-Boot-Absolutpegel-Fehler zugunsten des Web-Solls
    // korrigieren.
    if(outSp!=knownSp && shapeSp==knownSp && shapeSpeedFit>=50.0f) {
      outSp=knownSp;
      outSc=shapeSpeedConf;
    }
    if(outMd!=knownMd && shapeMd==knownMd && shapeModeFit>=50.0f) {
      outMd=knownMd;
      outMc=shapeModeConf;
    }
  }

  float d0=comboRichDistanceToState(knownSp,knownMd,0,cur);
  float d1=comboRichDistanceToState(knownSp,knownMd,1,cur);
  float oscLinearConf=0.0f;
  int rawOsc=comboOscLinValid?comboOscLinearDetect(knownSp,knownMd,cur,&oscLinearConf):(d1<d0?1:0);
  if(rawOsc<0)rawOsc=d1<d0?1:0;

  int outOsc=rawOsc;
  float outOc=comboOscLinValid?oscLinearConf:contextualConfidence(rawOsc?d1:d0,rawOsc?d0:d1);

  // Der Webzustand bleibt fuer Drehen autoritativ. OSC-LINEAR darf ihn
  // bestaetigen; ein Widerspruch wird als 0 % Sicherheit markiert, aber nicht
  // mehr als harter Zustandsfehler. Das vermeidet genau die beobachteten
  // Cross-Boot-Fehlwarnungen bei physikalisch kaum trennbaren AUS/AN-Paaren.
  if(rawOsc!=knownOs) {
    outOsc=knownOs;
    outOc=0.0f;
  }

  if(sp)*sp=outSp+1;
  if(md)*md=outMd;
  if(os)*os=outOsc;
  if(sc)*sc=outSc;
  if(mc)*mc=outMc;
  if(oc)*oc=outOc;
}

// Ergebnis der letzten Kombi-Klassifikation, damit Stufe und Modus aus
// DERSELBEN Messung stammen - und die 600 ms Messzeit nicht doppelt anfallen.
struct ComboCache { unsigned long at=0; int speed=0; int mode=-1; float sc=0, mc=0; bool ok=false; };
ComboCache comboCache;

bool detectComboCached(int* sp,int* md,float* sc,float* mc) {
  if(!comboOscComplete() && !comboComplete()) return false;
  if(comboCache.ok && (millis()-comboCache.at)<400UL) {
    if(sp)*sp=comboCache.speed;
    if(md)*md=comboCache.mode;
    if(sc)*sc=comboCache.sc;
    if(mc)*mc=comboCache.mc;
    return true;
  }

  FbStats cur[5];sampleProfileWindow(cur);
  int s2=0,m2=-1;float c1=0,c2=0;
  if(comboOscComplete()) {
    int o=-1;float oc=0;
    detectComboOsc(cur,&s2,&m2,&o,&c1,&c2,&oc);
  } else {
    if(detectCombo(cur,&s2,&m2,&c1,&c2)<=0){comboCache.ok=false;return false;}
  }

  if(s2<=0){comboCache.ok=false;return false;}
  comboCache={millis(),s2,m2,c1,c2,true};
  if(sp)*sp=s2;
  if(md)*md=m2;
  if(sc)*sc=c1;
  if(mc)*mc=c2;
  return true;
}

int detectLearnedSpeed(float* confidenceOut) {
  FbStats cur[5];
  sampleProfileWindow(cur);

  // Speed wird fast ausschließlich über LED1..3 beurteilt.
  // LED8/9 dürfen die Speed-Erkennung nur minimal beeinflussen.
  const float speedWeights[5] = {1.8f,1.8f,1.8f,0.08f,0.08f};
  // Wenn alle neun Kombiprofile vorliegen, ist deren Klassifikation
  // genauer - sie kennt den Modus-Einfluss auf das Stufenmuster.
  {
    int sp=0, md=-1; float sc=0, mc=0;
    if(detectComboCached(&sp,&md,&sc,&mc)) {
      if(confidenceOut) *confidenceOut = sc;
      return sp;
    }
  }
  int idx = detectProfilesWeightedFromCurrent(speedSig,cur,speedWeights,confidenceOut);
  return idx < 0 ? 0 : idx + 1;
}

int detectLearnedMode(float* confidenceOut) {
  FbStats cur[5];
  sampleProfileWindow(cur);

  // Modus wird primär über LED8/9 beurteilt.
  const float modeWeights[5] = {0.08f,0.08f,0.08f,2.2f,2.2f};
  {
    int sp=0, md=-1; float sc=0, mc=0;
    if(detectComboCached(&sp,&md,&sc,&mc)) {
      if(confidenceOut) *confidenceOut = mc;
      return md;
    }
  }
  int idx = detectProfilesWeightedFromCurrent(modeSig,cur,modeWeights,confidenceOut);
  return idx < 0 ? -1 : idx;
}
#endif




bool smartTimeActive(unsigned long untilMs) {
  return untilMs != 0 && (long)(untilMs - millis()) > 0;
}


bool ledsLookOff(const FbStats cur[5]) {
#if ENABLE_LED_FEEDBACK
  float currentTotal=0.0f;
  for(int ch=0;ch<5;ch++)currentTotal+=cur[ch].avg;

  float minLearnedOnTotal=1e30f;
  float nearestOnMeanErr=1e30f;
  bool haveOn=false;

  auto considerOn=[&](const LearnedProfile& p){
    if(!p.valid)return;
    float total=0.0f;
    for(int ch=0;ch<5;ch++)total+=p.avgMean[ch];
    minLearnedOnTotal=min(minLearnedOnTotal,total);
    nearestOnMeanErr=min(nearestOnMeanErr,fabsf(currentTotal-total));
    haveOn=true;
  };

  // Finale 18P-Profile zuerst; Fallbackprofile nur wenn FINAL noch fehlt.
  if(comboOscComplete()) {
    for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++)for(int os=0;os<2;os++)
      considerOn(comboOscSig[sp][md][os]);
  } else if(comboComplete()) {
    for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++)considerOn(comboSig[sp][md]);
  } else {
    for(int s=0;s<3;s++)considerOn(speedSig[s]);
  }

  if(offSig.valid) {
    float w[5];
    for(int ch=0;ch<5;ch++)w[ch]=comboWValid?comboW[ch]:1.0f;

    float dOff=0.0f;
    for(int ch=0;ch<5;ch++) {
      float e=cur[ch].abovePct-offSig.avgPct[ch];
      dOff+=w[ch]*e*e;
    }

    float dOn=1e30f;
    auto considerOnPct=[&](const LearnedProfile& p){
      if(!p.valid)return;
      float d=0.0f;
      for(int ch=0;ch<5;ch++) {
        float e=cur[ch].abovePct-p.avgPct[ch];
        d+=w[ch]*e*e;
      }
      dOn=min(dOn,d);
    };

    if(comboOscComplete()) {
      for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++)for(int os=0;os<2;os++)
        considerOnPct(comboOscSig[sp][md][os]);
    } else if(comboComplete()) {
      for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++)considerOnPct(comboSig[sp][md]);
    } else {
      for(int s=0;s<3;s++)considerOnPct(speedSig[s]);
    }

    float offTotal=0.0f;
    for(int ch=0;ch<5;ch++)offTotal+=offSig.avgMean[ch];
    float offMeanErr=fabsf(currentTotal-offTotal);

    // Bei diesem Ventilator ist der absolute ADC-Pegel AUS gegenueber EIN sehr
    // stark getrennt. Wir nutzen diese zweite, unabhaengige Evidenz bewusst,
    // damit ein seltenes Prozent-/Multiplex-Ausreisserfenster nicht "AUS"
    // behaupten kann, waehrend der Ventilator physisch laeuft.
    if(haveOn && nearestOnMeanErr<1e29f && minLearnedOnTotal>offTotal+500.0f) {
      bool pctOff=(dOn>1e29f)?false:(dOff<=dOn);
      bool meanOff=(offMeanErr<=nearestOnMeanErr);
      float lowCut=offTotal+0.55f*(minLearnedOnTotal-offTotal);
      bool clearlyLow=currentTotal<=lowCut;
      bool veryNearOff=currentTotal<=offTotal*1.30f;

      return meanOff && (pctOff || clearlyLow || veryNearOff);
    }

    // Kein brauchbares EIN-Meanprofil: nur bei echter Naehe zum AUS-Profil.
    float sumW=0.0f;for(int ch=0;ch<5;ch++)sumW+=w[ch];
    return dOff < 25.0f*sumW;
  }

  // Vor der ersten finalen Kalibrierung nur eine sehr konservative
  // absolute Fallbackgrenze. Reale Projektdaten: AUS ~7,5k, EIN ~13k+.
  if(haveOn && minLearnedOnTotal<1e29f)
    return currentTotal < minLearnedOnTotal*0.55f;

  return currentTotal<10000.0f;
#else
  return false;
#endif
}

void applySmartDetectedState(int learnedSpeed, float speedConf,
                             int learnedMode, float modeConf,
                             int learnedOsc, float oscConf) {
#if ENABLE_LED_FEEDBACK
  if(!fanIsOn || !manualLedSyncRequested) return;

  if(learnedSpeed>=1 && learnedSpeed<=3 && speedConf>=8.0f) {
    state.speed=learnedSpeed;
    state.lastSpeed=learnedSpeed;
  }
  if(learnedMode>=0 && learnedMode<=2 && modeConf>=18.0f) {
    state.mode=learnedMode;
    state.lastMode=learnedMode;
  }
  if(learnedOsc>=0 && learnedOsc<=1 && oscConf>=18.0f) {
    state.osc=(learnedOsc==1);
    state.lastOsc=state.osc;
  }

  saveFanState();
  manualLedSyncRequested=false;
#endif
}


void resetStableDiagnostic() {
  diagSpeedCandidate=0;diagSpeedHits=0;
  diagModeCandidate=-1;diagModeHits=0;
  diagOscCandidate=-1;diagOscHits=0;
  stableDiagSpeed=0;stableDiagMode=-1;stableDiagOsc=-1;
  stableDiagSpeedConf=0.0f;stableDiagModeConf=0.0f;stableDiagOscConf=0.0f;
  diagWindowCount=0;diagWindowPos=0;
  diagCacheValid=false;
  diagCachePctValid=false;
  diagCacheOff=false;
  diagCacheSpeed=0;diagCacheMode=-1;diagCacheOsc=-1;
  diagCacheSpeedConf=0.0f;diagCacheModeConf=0.0f;diagCacheOscConf=0.0f;
  diagContextFit=0.0f;
  diagWebFit=0.0f;
  diagGlobalSpeed=0;diagGlobalMode=-1;diagGlobalOsc=-1;
  diagGlobalSpeedConf=0.0f;diagGlobalModeConf=0.0f;diagGlobalOscConf=0.0f;
  diagGlobalFit=0.0f;
  diagLastSampleMs=0;
  diagNextSampleMs=0;
  diagCacheAtMs=0;
}

void armLedDiagnostic(unsigned long settleMs) {
  resetStableDiagnostic();
  resetDiagnosticWindow();
  ledDetectReadyMs=fanIsOn?(millis()+settleMs):0;
}

void updateStableDiagnostic(int speed,float speedConf,
                            int mode,float modeConf,
                            int osc,float oscConf) {
  if(!fanIsOn || ledDetectReadyMs==0 || (long)(millis()-ledDetectReadyMs)<0) return;

  if(speed>=1 && speed<=3 && speedConf>=8.0f) {
    if(diagSpeedCandidate==speed){if(diagSpeedHits<5)diagSpeedHits++;}
    else {diagSpeedCandidate=speed;diagSpeedHits=1;}
    if(diagSpeedHits>=2){stableDiagSpeed=speed;stableDiagSpeedConf=speedConf;}
  } else {diagSpeedCandidate=0;diagSpeedHits=0;}

  if(mode>=0 && mode<=2 && modeConf>=18.0f) {
    if(diagModeCandidate==mode){if(diagModeHits<5)diagModeHits++;}
    else {diagModeCandidate=mode;diagModeHits=1;}
    if(diagModeHits>=2){stableDiagMode=mode;stableDiagModeConf=modeConf;}
  } else {diagModeCandidate=-1;diagModeHits=0;}

  if(osc>=0 && osc<=1 && oscConf>=18.0f) {
    if(diagOscCandidate==osc){if(diagOscHits<5)diagOscHits++;}
    else {diagOscCandidate=osc;diagOscHits=1;}
    if(diagOscHits>=2){stableDiagOsc=osc;stableDiagOscConf=oscConf;}
  } else {diagOscCandidate=-1;diagOscHits=0;}
}

String buildStateObjectJson() {
  String j="{";
  j+="\"bootId\":"+String((unsigned long)bootSessionId)+",";
  j+="\"revision\":"+String((unsigned long)stateRevision)+",";
  j+="\"powerOn\":"+String(fanIsOn?"true":"false")+",";
  j+="\"speed\":"+String(state.speed)+",";
  j+="\"osc\":"+String(state.osc?"true":"false")+",";
  j+="\"mode\":"+String(state.mode)+",";
  j+="\"sleep\":"+String(state.sleepMin)+",";
  j+="\"sleepSec\":"+String((unsigned long)timerRemainingSec())+",";
  j+="\"lastSpeed\":"+String(state.lastSpeed)+",";
  j+="\"lastOsc\":"+String(state.lastOsc?"true":"false")+",";
  j+="\"lastMode\":"+String(state.lastMode);
  j+="}";
  return j;
}

void sendOkState() {
  String j="{\"ok\":true,\"state\":";
  j+=buildStateObjectJson();
  j+="}";
  server.send(200,"application/json",j);
}

void sendStatus() {
  String ip = WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  int rssi = WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : 0;

  int learnedSpeed=diagCacheValid?diagCacheSpeed:0;
  int learnedMode=diagCacheValid?diagCacheMode:-1;
  int learnedOsc=diagCacheValid?diagCacheOsc:-1;
  float speedConf=diagCacheValid?diagCacheSpeedConf:0.0f;
  float modeConf=diagCacheValid?diagCacheModeConf:0.0f;
  float oscConf=diagCacheValid?diagCacheOscConf:0.0f;
  bool measuredOff=diagCachePctValid&&diagCacheOff;
  bool haveDetectWindow=diagCacheValid;


  String j="{";
  j+="\"version\":\""+String(FW_VERSION)+"\",";
  j+="\"build\":\""+String(__DATE__)+" "+String(__TIME__)+"\",";
  j+="\"ip\":\""+ip+"\",";
  j+="\"rssi\":"+String(rssi)+",";
  j+="\"uptime\":"+String(millis()/1000UL)+",";
  j+="\"offLearned\":"+String(offSig.valid?"true":"false")+",";
  j+="\"comboLearned\":"+String(comboComplete()?"true":"false")+",";
  j+="\"combo18Learned\":"+String(comboOscComplete()?"true":"false")+",";
  j+="\"combo9Count\":"+String(comboValidCount())+",";
  j+="\"combo18Count\":"+String(comboOscValidCount())+",";
  j+="\"globalSyncSafe\":"+String(comboGlobalSyncSafe?"true":"false")+",";
  j+="\"oscLinearValid\":"+String(comboOscLinValid?"true":"false")+",";
  j+="\"recognitionModel\":\""+String(activeRecognitionModel())+"\",";
  j+="\"comboStorage\":\""+String(calibrationSamplerCurrent&&compactComboLoaded?"AB_CRC_NVS":"NEUKALIBRIERUNG")+"\",";
  j+="\"calSequence\":"+String((unsigned long)calStoreSequence)+",";
  j+="\"calCrc\":"+String((unsigned long)calStoreCrc)+",";
  j+="\"calSlots\":"+String(calStoreValidSlots)+",";

  j+="\"state\":{";
  j+="\"bootId\":"+String((unsigned long)bootSessionId)+",";
  j+="\"revision\":"+String((unsigned long)stateRevision)+",";
  j+="\"powerOn\":"+String(fanIsOn?"true":"false")+",";
  j+="\"speed\":"+String(state.speed)+",";
  j+="\"osc\":"+String(state.osc?"true":"false")+",";
  j+="\"mode\":"+String(state.mode)+",";
  j+="\"sleep\":"+String(state.sleepMin)+",";
  j+="\"sleepSec\":"+String((unsigned long)timerRemainingSec())+",";
  j+="\"lastSpeed\":"+String(state.lastSpeed)+",";
  j+="\"lastOsc\":"+String(state.lastOsc?"true":"false")+",";
  j+="\"lastMode\":"+String(state.lastMode)+",";
  j+="\"startupSync\":"+String(smartTimeActive(startupSyncUntilMs)?"true":"false")+",";
  j+="\"startupReady\":"+String((startupSyncReadyMs!=0 && (long)(millis()-startupSyncReadyMs)>=0)?"true":"false")+",";
  j+="\"startupSpeedHits\":"+String(startupSpeedHits)+",";
  j+="\"startupModeHits\":"+String(startupModeHits);
  j+="},";

#if ENABLE_LED_FEEDBACK
  bool diagCanShow=fanIsOn && !measuredOff && ledDetectReadyMs!=0 &&
                   (long)(millis()-ledDetectReadyMs)>=0 && haveDetectWindow;
  // Diagnosefelder zeigen den aktuellsten fertigen Cache, nicht einen
  // eventuell Sekunden alten "stable"-Treffer. Das macht die UI zeitnah.
  int shownSpeed=diagCanShow?learnedSpeed:0;
  int shownMode=diagCanShow?learnedMode:-1;
  int shownOsc=diagCanShow?learnedOsc:-1;
  float shownSpeedConf=diagCanShow?speedConf:0.0f;
  float shownModeConf=diagCanShow?modeConf:0.0f;
  float shownOscConf=diagCanShow?oscConf:0.0f;

  j+="\"calibration\":{";
  j+="\"s1\":"+String(speedSig[0].valid?"true":"false")+",";
  j+="\"s2\":"+String(speedSig[1].valid?"true":"false")+",";
  j+="\"s3\":"+String(speedSig[2].valid?"true":"false")+",";
  j+="\"normal\":"+String(modeSig[0].valid?"true":"false")+",";
  j+="\"breeze\":"+String(modeSig[1].valid?"true":"false")+",";
  j+="\"night\":"+String(modeSig[2].valid?"true":"false")+",";
  j+="\"detected\":"+String(shownSpeed)+",";
  j+="\"confidence\":"+String(shownSpeedConf,1)+",";
  j+="\"detectedMode\":"+String(shownMode)+",";
  j+="\"modeConfidence\":"+String(shownModeConf,1)+",";
  j+="\"detectedOsc\":"+String(shownOsc)+",";
  j+="\"oscConfidence\":"+String(shownOscConf,1)+",";
  j+="\"globalSpeed\":"+String(diagCanShow?diagGlobalSpeed:0)+",";
  j+="\"globalMode\":"+String(diagCanShow?diagGlobalMode:-1)+",";
  j+="\"globalOsc\":"+String(diagCanShow?diagGlobalOsc:-1)+",";
  j+="\"globalSpeedConfidence\":"+String(diagCanShow?diagGlobalSpeedConf:0.0f,1)+",";
  j+="\"globalModeConfidence\":"+String(diagCanShow?diagGlobalModeConf:0.0f,1)+",";
  j+="\"globalOscConfidence\":"+String(diagCanShow?diagGlobalOscConf:0.0f,1)+",";
  j+="\"contextFit\":"+String(diagCanShow?diagContextFit:0.0f,1)+",";
  j+="\"webFit\":"+String(diagCanShow?diagWebFit:0.0f,1)+",";
  j+="\"globalFit\":"+String(diagCanShow?diagGlobalFit:0.0f,1)+",";
  unsigned long diagAge=diagCacheAtMs?millis()-diagCacheAtMs:0;
  j+="\"ageMs\":"+String((unsigned long)diagAge)+",";
  // Der Fit bleibt Diagnose. Cross-Boot-Pegelverschiebungen duerfen nicht
  // alleine eine harte Abweichung erzeugen, wenn die Dimensionen passen.
  bool trackingMatch=diagCanShow && diagAge<=5000UL &&
                     shownSpeed==state.speed &&
                     shownMode==state.mode &&
                     shownOsc==(state.osc?1:0);
  j+="\"trackingMatch\":"+String(trackingMatch?"true":"false")+",";
  j+="\"detectedOff\":"+String(measuredOff?"true":"false")+",";
  j+="\"livePct\":[";
  for(int ch=0;ch<5;ch++) {
    if(ch)j+=",";
    j+=String(diagCachePctValid?diagCachePct[ch]:0.0f,1);
  }
  j+="],";

  bool expectedValid=fanIsOn && state.speed>=1 && state.speed<=3 &&
                     state.mode<=2 && comboOscSig[state.speed-1][state.mode][state.osc?1:0].valid;
  j+="\"expectedValid\":"+String(expectedValid?"true":"false")+",";
  j+="\"expectedPct\":[";
  for(int ch=0;ch<5;ch++) {
    if(ch)j+=",";
    float v=expectedValid?comboOscSig[state.speed-1][state.mode][state.osc?1:0].avgPct[ch]:0.0f;
    j+=String(v,1);
  }
  j+="],";
  bool diagSettling=fanIsOn && !measuredOff &&
                    (ledDetectReadyMs==0 || (long)(millis()-ledDetectReadyMs)<0 || !diagCacheValid);
  j+="\"settling\":"+String(diagSettling?"true":"false");
  j+="}";
#else
  j+="\"calibration\":{\"s1\":false,\"s2\":false,\"s3\":false,\"normal\":false,\"breeze\":false,\"night\":false,\"detected\":0,\"confidence\":0,\"detectedMode\":-1,\"modeConfidence\":0,\"detectedOsc\":-1,\"oscConfidence\":0,\"detectedOff\":false}";
#endif

  j+="}";
  server.send(200,"application/json",j);
}

String buildBackupJson() {
  String j;
  j.reserve(30000);
  j="{";
  j+="\"format\":8,";
  j+="\"sampler\":\"FINAL_EFFICIENT_BALANCED_V6_OSCLIN1\",";
  j+="\"fw\":\""+String(FW_VERSION)+"\",";

  j+="\"fan\":{";
  j+="\"lastSpeed\":"+String(state.lastSpeed)+",";
  j+="\"lastOsc\":"+String(state.lastOsc?"true":"false")+",";
  j+="\"lastMode\":"+String(state.lastMode);
  j+="},";

#if ENABLE_LED_FEEDBACK
  j+="\"thresholds\":[";
  for(int i=0;i<5;i++) {
    if(i) j+=",";
    j+=String(fbThresholds[i]);
  }
  j+="],";

  j+="\"speedProfiles\":[";
  for(int s=0;s<3;s++) {
    if(s) j+=",";
    j+="{\"valid\":"+String(speedSig[s].valid?"true":"false")+",\"pct\":[";
    for(int ch=0;ch<5;ch++) {
      if(ch) j+=",";
      j+=String(speedSig[s].avgPct[ch],6);
    }
    j+="],\"mean\":[";
    for(int ch=0;ch<5;ch++) {
      if(ch) j+=",";
      j+=String(speedSig[s].avgMean[ch],6);
    }
    j+="]}";
  }
  j+="],";

  j+="\"modeProfiles\":[";
  for(int s=0;s<3;s++) {
    if(s) j+=",";
    j+="{\"valid\":"+String(modeSig[s].valid?"true":"false")+",\"pct\":[";
    for(int ch=0;ch<5;ch++) {
      if(ch) j+=",";
      j+=String(modeSig[s].avgPct[ch],6);
    }
    j+="],\"mean\":[";
    for(int ch=0;ch<5;ch++) {
      if(ch) j+=",";
      j+=String(modeSig[s].avgMean[ch],6);
    }
    j+="]}";
  }
  j+="],";

  // AUS-Profil - fehlte bisher komplett im Backup
  j+="\"offProfile\":{\"valid\":"+String(offSig.valid?"true":"false")+",\"pct\":[";
  for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(offSig.avgPct[ch],6); }
  j+="],\"mean\":[";
  for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(offSig.avgMean[ch],6); }
  j+="]},";

  j+="\"comboShape\":"+String(comboShape,4)+",";
  j+="\"comboWeights\":{\"valid\":"+String(comboWValid?"true":"false")+",\"w\":[";
  for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(comboW[ch],6); }
  j+="]},";
  j+="\"comboMeanWeights\":[";
  for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(comboMeanW[ch],6); }
  j+="],";
  j+="\"comboMeanAlpha\":"+String(comboMeanAlpha,6)+",";
  j+="\"comboLevelAlpha\":"+String(comboLevelAlpha,6)+",";
  j+="\"globalSyncSafe\":"+String(comboGlobalSyncSafe?"true":"false")+",";
  j+="\"comboRichWeights\":[";
  bool firstRW=true;
  for(int ch=0;ch<5;ch++)for(int q=0;q<RICH_CDF_LEVELS;q++){
    if(!firstRW)j+=",";
    firstRW=false;
    j+=String(comboRichW[ch][q],7);
  }
  j+="],";
  j+="\"comboRichPctQ\":[";
  bool firstRQ=true;
  for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++)for(int os=0;os<2;os++)
    for(int ch=0;ch<5;ch++)for(int q=0;q<RICH_CDF_LEVELS;q++){
      if(!firstRQ)j+=",";
      firstRQ=false;
      j+=String((unsigned)comboRichPctQ[sp][md][os][ch][q]);
    }
  j+="],";
  j+="\"comboFitLimit\":"+String(comboFitLimit,6)+",";
  j+="\"oscLinearValid\":"+String(comboOscLinValid?"true":"false")+",";
  j+="\"oscLinearScale\":[";
  for(int md=0;md<3;md++){if(md)j+=",";j+=String(comboOscLinScale[md],10);}j+="],";
  j+="\"oscLinearBias\":[";
  for(int md=0;md<3;md++){if(md)j+=",";j+=String(comboOscLinBias[md],8);}j+="],";
  j+="\"oscLinearWeights\":[";
  bool firstOL=true;
  for(int md=0;md<3;md++)for(int f=0;f<OSC_LINEAR_FEATURES;f++){
    if(!firstOL)j+=",";
    firstOL=false;
    j+=String((int)comboOscLinW[md][f]);
  }
  j+="],";

  // Neun Kombiprofile, Reihenfolge Stufe 1..3 x Normal/Breeze/Nacht
  j+="\"comboProfiles\":[";
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++) {
      if(sp||md) j+=",";
      j+="{\"valid\":"+String(comboSig[sp][md].valid?"true":"false")+",\"pct\":[";
      for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(comboSig[sp][md].avgPct[ch],6); }
      j+="],\"mean\":[";
      for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(comboSig[sp][md].avgMean[ch],6); }
      j+="]}";
    }
  j+="]";

  j+=",";

  // Finale 18 Profile: Stufe 1..3 x Normal/Breeze/Nacht x Drehen AUS/AN.
  j+="\"comboOscProfiles\":[";
  bool first18=true;
  for(int sp=0;sp<3;sp++)
    for(int md=0;md<3;md++)
      for(int os=0;os<2;os++) {
        if(!first18)j+=",";
        first18=false;
        j+="{\"valid\":"+String(comboOscSig[sp][md][os].valid?"true":"false")+",\"pct\":[";
        for(int ch=0;ch<5;ch++){if(ch)j+=",";j+=String(comboOscSig[sp][md][os].avgPct[ch],6);}
        j+="],\"mean\":[";
        for(int ch=0;ch<5;ch++){if(ch)j+=",";j+=String(comboOscSig[sp][md][os].avgMean[ch],6);}
        j+="]}";
      }
  j+="]";
#else
  j+="\"thresholds\":[],\"speedProfiles\":[],\"modeProfiles\":[],\"offProfile\":{},\"comboProfiles\":[],\"comboOscProfiles\":[]";
#endif

  j+="}";
  return j;
}

bool extractJsonInt(const String& src, const String& key, int& out) {
  String needle="\""+key+"\":";
  int p=src.indexOf(needle);
  if(p<0) return false;
  p += needle.length();
  out = src.substring(p).toInt();
  return true;
}

bool extractJsonBool(const String& src, const String& key, bool& out) {
  String needle="\""+key+"\":";
  int p=src.indexOf(needle);
  if(p<0) return false;
  p += needle.length();
  while(p<src.length() && isspace((unsigned char)src[p])) p++;
  if(src.startsWith("true",p)) { out=true; return true; }
  if(src.startsWith("false",p)) { out=false; return true; }
  return false;
}

bool extractNumberArray(const String& src, const String& key, float* out, int count) {
  String needle="\""+key+"\":[";
  int p=src.indexOf(needle);
  if(p<0) return false;
  p += needle.length();

  for(int i=0;i<count;i++) {
    while(p<src.length() && (isspace((unsigned char)src[p]) || src[p]==',')) p++;
    int e=p;
    while(e<src.length() && src[e]!=',' && src[e]!=']') e++;
    if(e<=p) return false;
    out[i]=src.substring(p,e).toFloat();
    p=e;
  }
  return true;
}

bool extractIntArray(const String& src, const String& key, int* out, int count) {
  float tmp[8];
  if(count>8) return false;
  if(!extractNumberArray(src,key,tmp,count)) return false;
  for(int i=0;i<count;i++) out[i]=(int)tmp[i];
  return true;
}

bool extractFlatFloatArray(const String& src,const String& key,float* out,int count) {
  String needle="\""+key+"\":[";
  int p=src.indexOf(needle);
  if(p<0)return false;
  p+=needle.length();

  for(int i=0;i<count;i++) {
    while(p<src.length() && isspace((unsigned char)src[p]))p++;
    int e=p;
    while(e<src.length() && src[e]!=',' && src[e]!=']')e++;
    if(e<=p)return false;
    out[i]=src.substring(p,e).toFloat();
    p=e;
    if(i<count-1) {
      if(p>=src.length() || src[p]!=',')return false;
      p++;
    }
  }
  return true;
}

bool extractFlatUInt16Array(const String& src,const String& key,uint16_t* out,int count) {
  String needle="\""+key+"\":[";
  int p=src.indexOf(needle);
  if(p<0)return false;
  p+=needle.length();

  for(int i=0;i<count;i++) {
    while(p<src.length() && isspace((unsigned char)src[p]))p++;
    int e=p;
    while(e<src.length() && src[e]!=',' && src[e]!=']')e++;
    if(e<=p)return false;
    int v=src.substring(p,e).toInt();
    if(v<0||v>10000)return false;
    out[i]=(uint16_t)v;
    p=e;
    if(i<count-1) {
      if(p>=src.length() || src[p]!=',')return false;
      p++;
    }
  }
  return true;
}

bool extractObjectArrayItem(const String& src, const String& key, int index, String& outObj) {
  String needle="\""+key+"\":[";
  int p=src.indexOf(needle);
  if(p<0) return false;
  p += needle.length();

  int current=-1;
  int depth=0;
  int start=-1;

  for(int i=p;i<src.length();i++) {
    char c=src[i];
    if(c=='{') {
      if(depth==0) {
        current++;
        if(current==index) start=i;
      }
      depth++;
    } else if(c=='}') {
      depth--;
      if(depth==0 && current==index && start>=0) {
        outObj=src.substring(start,i+1);
        return true;
      }
    } else if(c==']' && depth==0) {
      break;
    }
  }
  return false;
}


bool extractJsonString(const String& src,const String& key,String& out) {
  String needle="\""+key+"\":\"";
  int p=src.indexOf(needle);
  if(p<0)return false;
  p+=needle.length();
  int e=src.indexOf('"',p);
  if(e<0)return false;
  out=src.substring(p,e);
  return true;
}

bool applyBackupJson(const String& body, String& error) {
  int format=0;
  String sampler;
  if(!extractJsonInt(body,"format",format) || format!=8 ||
     !extractJsonString(body,"sampler",sampler) || sampler!="FINAL_EFFICIENT_BALANCED_V6_OSCLIN1") {
    error="Backup nicht kompatibel: OSC-LINEAR V1 benoetigt das 6.4.4/6.4.5-Backupformat";
    return false;
  }

#if ENABLE_LED_FEEDBACK
  int tmpThr[5];
  static LearnedProfile tmpSpeed[3],tmpMode[3],tmpOff;
  static LearnedProfile tmpCombo9[3][3],tmpCombo18[3][3][2];
  float tmpW[5],tmpMeanW[5];
  static float tmpRichW[5][RICH_CDF_LEVELS];
  static uint16_t tmpRichQ[3][3][2][5][RICH_CDF_LEVELS];
  float tmpShape=0.0f,tmpMeanAlpha=0.0f,tmpLevelAlpha=0.0f,tmpFitLimit=0.0f;
  bool tmpWValid=false;
  bool tmpGlobalSyncSafe=false;
  bool tmpOscLinValid=false;
  float tmpOscLinScale[3]={1.0f,1.0f,1.0f};
  float tmpOscLinBias[3]={0.0f,0.0f,0.0f};
  static float tmpOscLinWf[3][OSC_LINEAR_FEATURES] = {};
  static int16_t tmpOscLinW[3][OSC_LINEAR_FEATURES] = {};

  memset(tmpSpeed,0,sizeof(tmpSpeed));memset(tmpMode,0,sizeof(tmpMode));
  memset(&tmpOff,0,sizeof(tmpOff));memset(tmpCombo9,0,sizeof(tmpCombo9));
  memset(tmpCombo18,0,sizeof(tmpCombo18));

  if(!extractIntArray(body,"thresholds",tmpThr,5)) {
    error="Backup: thresholds fehlen";return false;
  }
  for(int ch=0;ch<5;ch++) {
    if(tmpThr[ch]<0||tmpThr[ch]>4095){error="Backup: Schwellwert ausserhalb ADC-Bereich";return false;}
  }

  for(int s=0;s<3;s++) {
    String obj;bool valid=false;float pct[5],mean[5];
    if(!extractObjectArrayItem(body,"speedProfiles",s,obj) ||
       !extractJsonBool(obj,"valid",valid) ||
       !extractNumberArray(obj,"pct",pct,5) || !extractNumberArray(obj,"mean",mean,5)) {
      error="Backup: Speedprofile unvollstaendig";return false;
    }
    for(int ch=0;ch<5;ch++){tmpSpeed[s].avgPct[ch]=pct[ch];tmpSpeed[s].avgMean[ch]=mean[ch];}
    tmpSpeed[s].valid=valid;
  }

  for(int s=0;s<3;s++) {
    String obj;bool valid=false;float pct[5],mean[5];
    if(!extractObjectArrayItem(body,"modeProfiles",s,obj) ||
       !extractJsonBool(obj,"valid",valid) ||
       !extractNumberArray(obj,"pct",pct,5) || !extractNumberArray(obj,"mean",mean,5)) {
      error="Backup: Modusprofile unvollstaendig";return false;
    }
    for(int ch=0;ch<5;ch++){tmpMode[s].avgPct[ch]=pct[ch];tmpMode[s].avgMean[ch]=mean[ch];}
    tmpMode[s].valid=valid;
  }

  {
    int p=body.indexOf("\"offProfile\"");
    if(p<0){error="Backup: AUS-Profil fehlt";return false;}
    int a=body.indexOf('{',p),b=body.indexOf('}',a);
    if(a<0||b<=a){error="Backup: AUS-Profil ungueltig";return false;}
    String obj=body.substring(a,b+1);bool valid=false;float pct[5],mean[5];
    if(!extractJsonBool(obj,"valid",valid) || !valid ||
       !extractNumberArray(obj,"pct",pct,5) || !extractNumberArray(obj,"mean",mean,5)) {
      error="Backup: AUS-Profil nicht FINAL";return false;
    }
    for(int ch=0;ch<5;ch++){tmpOff.avgPct[ch]=pct[ch];tmpOff.avgMean[ch]=mean[ch];}
    tmpOff.valid=true;
  }

  {
    int p=body.indexOf("\"comboWeights\"");
    if(p<0){error="Backup: comboWeights fehlen";return false;}
    int a=body.indexOf('{',p),b=body.indexOf('}',a);
    if(a<0||b<=a){error="Backup: comboWeights ungueltig";return false;}
    String obj=body.substring(a,b+1);
    if(!extractJsonBool(obj,"valid",tmpWValid) || !tmpWValid ||
       !extractNumberArray(obj,"w",tmpW,5)) {
      error="Backup: comboWeights nicht FINAL";return false;
    }
  }

  int q=body.indexOf("\"comboShape\"");
  if(q<0){error="Backup: comboShape fehlt";return false;}
  tmpShape=body.substring(body.indexOf(':',q)+1).toFloat();
  if(tmpShape<0.0f||tmpShape>1.0f){error="Backup: comboShape ungueltig";return false;}

  if(!extractNumberArray(body,"comboMeanWeights",tmpMeanW,5)) {
    error="Backup: Mean-Gewichte fehlen";return false;
  }
  int ma=body.indexOf("\"comboMeanAlpha\""),la=body.indexOf("\"comboLevelAlpha\""),fl=body.indexOf("\"comboFitLimit\"");
  if(ma<0||la<0||fl<0){error="Backup: Rich-Modell unvollstaendig";return false;}
  tmpMeanAlpha=body.substring(body.indexOf(':',ma)+1).toFloat();
  tmpLevelAlpha=body.substring(body.indexOf(':',la)+1).toFloat();
  tmpFitLimit=body.substring(body.indexOf(':',fl)+1).toFloat();
  if(tmpMeanAlpha<0.0f||tmpMeanAlpha>128.0f||tmpLevelAlpha<0.0f||tmpLevelAlpha>128.0f||
     tmpFitLimit<0.25f||tmpFitLimit>1000.0f) {
    error="Backup: Rich-Modellwerte ausserhalb Bereich";return false;
  }
  if(!extractJsonBool(body,"globalSyncSafe",tmpGlobalSyncSafe)) {
    error="Backup: Global-Sync-Status fehlt";return false;
  }
  if(!extractFlatFloatArray(body,"comboRichWeights",&tmpRichW[0][0],5*RICH_CDF_LEVELS)) {
    error="Backup: Rich-CDF-Gewichte fehlen";return false;
  }
  for(int ch=0;ch<5;ch++)for(int q=0;q<RICH_CDF_LEVELS;q++) {
    if(tmpRichW[ch][q]!=tmpRichW[ch][q] || tmpRichW[ch][q]<0.0f || tmpRichW[ch][q]>10.0f) {
      error="Backup: Rich-CDF-Gewicht ausserhalb Bereich";return false;
    }
  }
  if(!extractFlatUInt16Array(body,"comboRichPctQ",&tmpRichQ[0][0][0][0][0],
                           3*3*2*5*RICH_CDF_LEVELS)) {
    error="Backup: Rich-CDF-Profile fehlen";return false;
  }

  if(!extractJsonBool(body,"oscLinearValid",tmpOscLinValid) || !tmpOscLinValid ||
     !extractNumberArray(body,"oscLinearScale",tmpOscLinScale,3) ||
     !extractNumberArray(body,"oscLinearBias",tmpOscLinBias,3) ||
     !extractFlatFloatArray(body,"oscLinearWeights",&tmpOscLinWf[0][0],3*OSC_LINEAR_FEATURES)) {
    error="Backup: OSC-LINEAR-Modell fehlt";return false;
  }
  for(int md=0;md<3;md++) {
    if(!(tmpOscLinScale[md]>0.0f) || tmpOscLinScale[md]>1.0f ||
       tmpOscLinBias[md]!=tmpOscLinBias[md] || fabsf(tmpOscLinBias[md])>10000.0f) {
      error="Backup: OSC-LINEAR Skalierung/Bias ungueltig";return false;
    }
    for(int f=0;f<OSC_LINEAR_FEATURES;f++) {
      float v=tmpOscLinWf[md][f];
      if(v!=v || v<-32767.0f || v>32767.0f) {
        error="Backup: OSC-LINEAR Gewicht ausserhalb Bereich";return false;
      }
      tmpOscLinW[md][f]=(int16_t)lroundf(v);
    }
  }

  for(int i=0;i<9;i++) {
    String obj;bool valid=false;float pct[5],mean[5];
    if(!extractObjectArrayItem(body,"comboProfiles",i,obj) ||
       !extractJsonBool(obj,"valid",valid) || !valid ||
       !extractNumberArray(obj,"pct",pct,5) || !extractNumberArray(obj,"mean",mean,5)) {
      error="Backup: 9P-Fallback unvollstaendig";return false;
    }
    int sp=i/3,md=i%3;
    for(int ch=0;ch<5;ch++){tmpCombo9[sp][md].avgPct[ch]=pct[ch];tmpCombo9[sp][md].avgMean[ch]=mean[ch];}
    tmpCombo9[sp][md].valid=true;
  }

  for(int i=0;i<18;i++) {
    String obj;bool valid=false;float pct[5],mean[5];
    if(!extractObjectArrayItem(body,"comboOscProfiles",i,obj) ||
       !extractJsonBool(obj,"valid",valid) || !valid ||
       !extractNumberArray(obj,"pct",pct,5) || !extractNumberArray(obj,"mean",mean,5)) {
      error="Backup: 18P-FINAL unvollstaendig";return false;
    }
    int sp=i/6,md=(i/2)%3,os=i%2;
    for(int ch=0;ch<5;ch++){tmpCombo18[sp][md][os].avgPct[ch]=pct[ch];tmpCombo18[sp][md][os].avgMean[ch]=mean[ch];}
    tmpCombo18[sp][md][os].valid=true;
  }

  // Optionale gemerkte Bedienwerte erst nach erfolgreicher Modellpruefung uebernehmen.
  uint8_t tmpLastSpeed=state.lastSpeed,tmpLastMode=state.lastMode;
  bool tmpLastOsc=state.lastOsc;
  String fanObj;
  int fanPos=body.indexOf("\"fan\":{");
  if(fanPos>=0) {
    int a=body.indexOf('{',fanPos),depth=0,b=-1;
    for(int i=a;i<body.length();i++) {
      if(body[i]=='{')depth++;
      else if(body[i]=='}'){depth--;if(depth==0){b=i;break;}}
    }
    if(b>a)fanObj=body.substring(a,b+1);
  }
  if(fanObj.length()) {
    int v;bool bo;
    if(extractJsonInt(fanObj,"lastSpeed",v))tmpLastSpeed=constrain(v,1,3);
    if(extractJsonInt(fanObj,"lastMode",v))tmpLastMode=constrain(v,0,2);
    if(extractJsonBool(fanObj,"lastOsc",bo))tmpLastOsc=bo;
  }

  // Jetzt atomar in das aktive RAM-Modell committen.
  for(int ch=0;ch<5;ch++)fbThresholds[ch]=tmpThr[ch];
  memcpy(speedSig,tmpSpeed,sizeof(speedSig));
  memcpy(modeSig,tmpMode,sizeof(modeSig));
  memcpy(&offSig,&tmpOff,sizeof(offSig));
  memcpy(comboSig,tmpCombo9,sizeof(comboSig));
  memcpy(comboOscSig,tmpCombo18,sizeof(comboOscSig));
  for(int ch=0;ch<5;ch++) {
    comboW[ch]=max(tmpW[ch],0.01f);
    comboMeanW[ch]=max(tmpMeanW[ch],0.01f);
  }
  comboWValid=tmpWValid;
  comboShape=tmpShape;
  comboMeanAlpha=constrain(tmpMeanAlpha,0.0f,16.0f);
  comboLevelAlpha=constrain(tmpLevelAlpha,0.0f,4.0f);
  memcpy(comboRichW,tmpRichW,sizeof(comboRichW));
  memcpy(comboRichPctQ,tmpRichQ,sizeof(comboRichPctQ));
  comboRichValid=true;
  comboGlobalSyncSafe=tmpGlobalSyncSafe;
  memcpy(comboOscLinW,tmpOscLinW,sizeof(comboOscLinW));
  memcpy(comboOscLinScale,tmpOscLinScale,sizeof(comboOscLinScale));
  memcpy(comboOscLinBias,tmpOscLinBias,sizeof(comboOscLinBias));
  comboOscLinValid=tmpOscLinValid;
  comboFitLimit=tmpFitLimit;
  calibrationSamplerCurrent=true;

  if(!saveCompactComboCalibration(true)) {
    // Voriger A/B-Slot bleibt Recovery. RAM ebenfalls auf diesen Stand zurueck.
    loadCompactComboCalibration();
    error="Backup konnte nicht CRC-verifiziert im A/B-NVS gespeichert werden";
    return false;
  }

  if(fanIsOn) {
    // Laufende Hardware niemals durch reine Backup-Metadaten umetikettieren.
    state.lastSpeed=state.speed;
    state.lastMode=state.mode;
    state.lastOsc=state.osc;
  } else {
    state.lastSpeed=tmpLastSpeed;
    state.lastMode=tmpLastMode;
    state.lastOsc=tmpLastOsc;
  }
  bumpStateRevision();
  saveFanState();
  resetStableDiagnostic();
  if(fanIsOn)armLedDiagnostic(state.osc?8000UL:LED_DETECT_SETTLE_MS);
#else
  error="LED-Feedback deaktiviert";
  return false;
#endif

  error="";
  return true;
}


static const char ANALYZE_PAGE[] PROGMEM = R"ANA(<!DOCTYPE html><html lang="de"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0d1116"><title>Finale Trennschaerfe-Analyse</title>
<style>
:root{--bg:#0d1116;--card:#161c24;--line:#2b3644;--txt:#e8edf3;--dim:#8496ab;--air:#4fd1c5;--bad:#e8624a;--warn:#f6a35c;--good:#5cd67f}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);padding:22px 16px 60px;font:400 16px/1.5 system-ui,sans-serif}
.wrap{max-width:900px;margin:auto}h1{font-size:22px;margin:0 0 4px}h2{font-size:12px;letter-spacing:.16em;text-transform:uppercase;color:var(--dim);margin:26px 0 10px}
p{color:var(--dim);font-size:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px;margin:12px 0}
button{background:var(--air);color:#06231f;border:0;border-radius:12px;padding:12px 18px;font-weight:700}button:disabled{opacity:.45}
a{color:var(--dim);text-decoration:none}table{width:100%;border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}
th,td{padding:7px 5px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}th{color:var(--dim)}
.sd{color:var(--dim);font-size:10px}
.g{border-color:#1f4d33;background:#0f2318;color:#bdf0cf}.w{border-color:#5a4420;background:#241d10;color:#f2ddb0}.b{border-color:#5a2320;background:#241110;color:#f2c0b8}
.bar{height:7px;border-radius:5px;background:var(--line);overflow:hidden;margin-top:14px}.bar i{display:block;height:100%;background:var(--air);transition:width .3s}
.act{background:#0d1116;border-left:3px solid var(--air);padding:10px;border-radius:8px}.dim{color:var(--dim)}
</style></head><body><div class="wrap">
<a href="/#einstellungen">&larr; zurueck</a>
<h1>Finale Trennschaerfe-Analyse · NORMAL-PRIORITY</h1>
<p>Messung und Profilverwaltung. <b>Normal</b> ist im laufenden Tracking bewusst priorisiert. Qualitaetsabweichungen werden weiter berechnet und exportiert, blockieren die manuelle Profiluebernahme aber nicht mehr.</p>

<div class="card">
<button id="go">FINAL-Efficient-Balanced RICH-CDF starten</button>
<button id="resume" hidden style="margin-left:8px">Checkpoint fortsetzen</button>
<button id="discard" hidden style="margin-left:8px;background:var(--warn);color:#251500">Checkpoint verwerfen</button>
<button id="stop" hidden style="margin-left:8px;background:var(--bad);color:#fff">Stoppen</button>
<button id="apply" hidden style="margin-left:8px">18 Profile uebernehmen</button>
<button id="exportAi" hidden style="margin-left:8px;background:#93c5fd;color:#071827">KI-Export JSON</button>
<button id="exportRaw" hidden style="margin-left:8px;background:#c4b5fd;color:#160b2d">KI-Export + Rohhistogramme</button>
<p class="dim">Nach einem vollstaendig abgeschlossenen Lauf ist die Profiluebernahme immer manuell moeglich. Messdifferenzen bleiben Diagnose, nicht Sperre.</p>
<div class="bar" id="bw" hidden><i id="bar"></i></div>
<p id="stat"></p><p id="live" class="dim"></p><p id="checkpoint" class="dim"></p><p id="act" class="act" hidden></p>
</div>


<p class="dim">Dauer: typischerweise etwa 40–60 Minuten; nur bei echten Grenzfaellen kommen bis zu sechs adaptive Holdout-Zusatzmessungen hinzu. Vier optimierte Trainingspfade geben jedem Zustand mindestens drei unterschiedliche Vorgaenger, reduzieren aber Speed-/Mode-Umschaltungen stark. Der Holdout misst zuerst zwei komplette 18-Zustands-Paesse mit jeweils unterschiedlichen Vorgaengern. 2/2 besteht sofort; nur ein 1/2-Grenzfall bekommt genau eine dritte Messung. 0/2 wird nicht durch weitere Ventilatorbelastung „repariert“, sondern sauber als Fehler ausgewiesen. Messfenster und Jitter bleiben mit 6/8 Fenstern in Normal bzw. 8/10 Fenstern in Breeze/Nacht unveraendert. Nach einem echten ESP-/Stromausfall wird ein angebrochener 18-Zustands-Repeat komplett neu gemessen; bereits vollstaendige Repeats bleiben erhalten.</p>
<div id="out"></div>

<script>
const MODE=['Normal','Breeze','Nacht'],CH=['LED1','LED2','LED3','LED8','LED9'];
const $=q=>document.querySelector(q);
const mean=a=>a.reduce((x,y)=>x+y,0)/a.length;
const sd=a=>{const m=mean(a);return Math.sqrt(a.reduce((s,v)=>s+(v-m)*(v-m),0)/Math.max(1,a.length-1))};

let pollTimer=null;
let tickBusy=false;
let lastRunning=false;

function scheduleTick(ms){
 if(pollTimer)clearTimeout(pollTimer);
 pollTimer=setTimeout(tick,ms);
}

async function fetchJson(url,opt={},timeout=6000){
 const ctl=new AbortController();
 const to=setTimeout(()=>ctl.abort(),timeout);
 try{
   const r=await fetch(url,{...opt,signal:ctl.signal,cache:'no-store'});
   let j=null;
   try{j=await r.json()}catch(e){}
   return {r,j};
 }finally{
   clearTimeout(to);
 }
}

function showConnectionWait(){
 $('#stat').textContent=lastRunning?
   'Verbindung kurz unterbrochen – Messlauf laeuft auf dem ESP weiter, neuer Versuch…':
   'Verbindung wird wiederhergestellt…';
}

function applyStatus(d){
 lastRunning=!!d.running;
 $('#bar').style.width=d.progress+'%';

 const recovery=!!d.recoveryAvailable;
 $('#resume').hidden=d.running||d.done||!recovery;
 $('#discard').hidden=d.running||!recovery;
 $('#resume').disabled=!d.recoverySafe||!!d.fanOn;
 $('#go').disabled=!!d.running||recovery;

 if(d.checkpointCapable){
   $('#checkpoint').textContent='Recovery: '+(d.checkpointStatus||'A/B CRC bereit')+
     (Number(d.resumeCount)>0
       ?(' · Resume '+d.resumeCount+': geladen '+d.resumeLoadedSteps+'/'+(d.trainingTotal||72)+', '+d.resumeRollbackProfiles+' Profil(e) des angebrochenen Repeats neu')
       :'')+
     (recovery&&!d.recoverySafe?' · vor Resume einmal komplett Netz AUS/EIN':'');
 }else{
   $('#checkpoint').textContent='Recovery: kein persistenter Reset-Checkpoint verfuegbar; Browser-Neuladen bleibt trotzdem sicher.';
 }

 if(d.running){
   $('#bw').hidden=false;
   $('#go').disabled=true;
   $('#stop').hidden=false;
   $('#apply').hidden=true;
   $('#exportAi').hidden=true;
   $('#exportRaw').hidden=true;
   $('#stat').textContent=d.preparing?
     ('Vorbereitung: '+d.progress+' %'):
     ('Messlauf: '+d.progress+' %');
 }else{
   $('#stop').hidden=true;
   if(d.done) $('#stat').textContent='Fertig';
   else if(d.aborted) $('#stat').textContent='Abgebrochen';
   else if(recovery) $('#stat').textContent='Recovery-Checkpoint vorhanden: '+d.checkpointSteps+'/'+(d.trainingTotal||72)+' Trainingsprofile gesichert';
   else $('#stat').textContent='Bereit';
 }

 const mode=MODE[Number(d.commandMode)||0]||'Normal';
 $('#live').textContent=d.fanOn?
   ('ESP-Befehlszustand: Stufe '+d.commandSpeed+' / '+mode+' / Drehen '+(d.commandOsc?'AN':'AUS')+
    (d.preparing?' · Vorbereitung':(' · Wiederholung '+Math.min(Number(d.repeat)+1,4)+'/4'))):
   ('ESP-Befehlszustand: AUS'+(d.preparing?' · Vorbereitung':''));

 if(d.action){
   $('#act').hidden=false;
   $('#act').textContent=d.action;
 }

 if(!d.running){
   if(d.done){
     $('#apply').hidden=!d.canApply;
     $('#exportAi').hidden=false;
     $('#exportRaw').hidden=false;
     render(d);
   }else if(d.aborted){
     $('#apply').hidden=true;
     $('#exportAi').hidden=true;
     $('#exportRaw').hidden=true;
     $('#out').innerHTML='<p class="dim">Abgebrochen. Bestehende Kalibrierung wurde nicht ueberschrieben.</p>';
   }
 }
}

async function tick(){
 if(tickBusy)return;
 tickBusy=true;
 try{
   const {r,j}=await fetchJson('/api/analyze',{},10000);
   if(!r.ok||!j)throw new Error('status');
   applyStatus(j);
   if(j.running)scheduleTick(900);
   else if(pollTimer){clearTimeout(pollTimer);pollTimer=null;}
 }catch(e){
   showConnectionWait();
   scheduleTick(1400);
 }finally{
   tickBusy=false;
 }
}

$('#resume').onclick=async()=>{
 $('#resume').disabled=true;
 $('#stat').textContent='Checkpoint wird auf ESP geladen…';
 try{
   const {r,j}=await fetchJson('/api/analyze/resume',{method:'POST'},6000);
   if(!r.ok){
     $('#stat').textContent='Resume nicht moeglich: '+((j&&j.error)||('HTTP '+r.status));
   }else{
     $('#stat').textContent='Resume bestaetigt – sichere physische Basis wird hergestellt…';
   }
 }catch(e){
   $('#stat').textContent='Resume-Antwort verloren – pruefe ESP-Status…';
 }
 scheduleTick(100);
};

$('#discard').onclick=async()=>{
 if(!confirm('Gesicherten Analyse-Checkpoint wirklich verwerfen?'))return;
 $('#discard').disabled=true;
 try{
   const {r,j}=await fetchJson('/api/analyze/recovery/discard',{method:'POST'},5000);
   $('#stat').textContent=r.ok?'Checkpoint verworfen – neuer Messlauf kann gestartet werden':
     ('Fehler: '+((j&&j.error)||('HTTP '+r.status)));
 }catch(e){
   $('#stat').textContent='Verbindung unterbrochen – Recovery-Status wird neu gelesen.';
 }
 $('#discard').disabled=false;
 scheduleTick(100);
};

$('#stop').onclick=async()=>{
 $('#stop').disabled=true;
 try{await fetchJson('/api/analyze/stop',{method:'POST'},5000)}catch(e){}
 $('#stop').disabled=false;
 scheduleTick(150);
};

$('#go').onclick=async()=>{
 $('#go').disabled=true;
 $('#stop').hidden=false;
 $('#bw').hidden=false;
 $('#out').innerHTML='';
 $('#stat').textContent='Start wird an ESP uebergeben…';

 try{
   const {r,j}=await fetchJson('/api/analyze/start',{method:'POST'},5000);
   if(!r.ok){
     $('#stat').textContent='Fehler: '+((j&&j.error)||('HTTP '+r.status));
     $('#go').disabled=false;
     $('#stop').hidden=true;
     return;
   }
   $('#stat').textContent='Start bestaetigt – Vorbereitung laeuft…';
 }catch(e){
   // Ein verlorenes POST-Response bedeutet nicht automatisch, dass der ESP
   // den Start nicht angenommen hat. Deshalb Status abfragen statt blind
   // einen zweiten Messlauf zu starten.
   $('#stat').textContent='Startantwort nicht angekommen – pruefe laufenden ESP-Messlauf…';
 }
 scheduleTick(100);
};

$('#exportAi').onclick=()=>{ location.href='/api/analyze/export?raw=0'; };
$('#exportRaw').onclick=()=>{ location.href='/api/analyze/export?raw=1'; };

$('#apply').onclick=async()=>{
 $('#apply').disabled=true;
 try{
   const {r,j}=await fetchJson('/api/analyze/apply',{method:'POST'},7000);
   $('#stat').textContent=r.ok&&j&&j.ok?
     'FINAL NORMAL-PRIORITY CRC-verifiziert gespeichert.':
     ('Fehler: '+((j&&j.error)||('HTTP '+r.status)));
 }catch(e){
   $('#stat').textContent='Verbindung unterbrochen – Speicherstatus erneut pruefen.';
 }
 $('#apply').disabled=false;
 scheduleTick(300);
};

// Browser ist nur Anzeige: Seite darf geschlossen/neugeladen werden.
// Bei ESP-Neustart kann der LittleFS-A/B-Checkpoint nach sicherem Netz-Power-On
// ueber "Checkpoint fortsetzen" wieder in ana geladen werden.
window.addEventListener('load',()=>scheduleTick(50));

function pairFactor(a,b){
 const dd=Math.sqrt(a.ch.reduce((s,v,i)=>{const e=mean(v)-mean(b.ch[i]);return s+e*e},0));
 const nn=Math.sqrt(a.ch.reduce((s,v,i)=>{const n=Math.max(sd(v),sd(b.ch[i]),0.05);return s+n*n},0));
 return {d:dd,n:nn,f:dd/nn};
}
function get(C,s,m,o){return C.find(x=>x.speed===s&&x.mode===m&&x.osc===o)}

function render(d){
 const C=d.combos;let h='';
 h+='<h2>18 Profile · feste Vergleichsbasis ADC '+(d.displayReferenceAdc||3072)+'</h2><div class="card"><table><tr><th>Zustand</th>'+CH.map(x=>'<th>'+x+'</th>').join('')+'</tr>';
 C.forEach(c=>{
  h+='<tr><td>St.'+c.speed+' / '+MODE[c.mode]+' / '+(c.osc?'Drehen AN':'Drehen AUS')+'</td>';
  c.ch.forEach(v=>h+='<td>'+mean(v).toFixed(1)+' <span class="sd">±'+sd(v).toFixed(1)+'</span></td>');
  h+='</tr>';
 });
 h+='</table></div>';

 // Osc-Paare
 let worstOsc=1e9,rows='';
 for(let s=1;s<=3;s++)for(let m=0;m<3;m++){
  const a=get(C,s,m,0),b=get(C,s,m,1),p=pairFactor(a,b);worstOsc=Math.min(worstOsc,p.f);
  const cls=p.f>=3?'g':p.f>=1.5?'w':'b';
  rows+='<tr><td>St.'+s+' / '+MODE[m]+'</td><td class="'+cls+'">'+p.f.toFixed(2)+'</td><td>'+p.d.toFixed(2)+'</td><td>'+p.n.toFixed(2)+'</td></tr>';
 }
 h+='<h2>Drehen-Rohtrennung · nur LED-%</h2><div class="card"><table><tr><th>Zustand</th><th>Faktor</th><th>Abstand</th><th>Rauschen</th></tr>'+rows+'</table></div>';
 h+='<h2>Optimierte Schwellwerte</h2><div class="card"><table><tr><th>Kanal</th><th>Schwelle</th><th>Speed</th><th>Modus</th><th>Drehen</th><th>AUS</th></tr>';
 d.opt.forEach((o,i)=>h+='<tr><td>'+CH[i]+'</td><td>'+o.thr+'</td><td>'+o.speed.toFixed(1)+'</td><td>'+o.mode.toFixed(1)+'</td><td>'+o.osc.toFixed(1)+'</td><td>'+o.off.toFixed(1)+'</td></tr>');
 h+='</table><p>Gewichte: '+d.weights.map((v,i)=>CH[i]+' '+Number(v).toFixed(2)).join(' · ')+'</p></div>';

 $('#out').innerHTML=h;
}
</script></body></html>)ANA";


// ===================== FINALE TRENNSCHAERFE-ANALYSE =====================
constexpr uint8_t ANA_COMBOS=18;       // 3 Speed x 3 Modi x 2 Osc
constexpr uint8_t ANA_REPEATS=4;

// Zeitlich gespreizte Messserien:
// Normal braucht weniger Fenster; Breeze/Nacht werden laenger beobachtet.
// Die Fenstertiefe bleibt bewusst unveraendert: wir sparen Belastung ueber
// weniger Zustandsfahrten, NICHT ueber weniger ADC-Information pro Messung.
constexpr uint8_t ANA_WINDOWS_NORMAL_OFF=6;
constexpr uint8_t ANA_WINDOWS_NORMAL_ON=8;
constexpr uint8_t ANA_WINDOWS_DYNAMIC_OFF=8;
constexpr uint8_t ANA_WINDOWS_DYNAMIC_ON=10;

inline uint8_t anaWindowsFor(uint8_t mode,bool osc) {
  if(mode==0)return osc?ANA_WINDOWS_NORMAL_ON:ANA_WINDOWS_NORMAL_OFF;
  return osc?ANA_WINDOWS_DYNAMIC_ON:ANA_WINDOWS_DYNAMIC_OFF;
}

constexpr uint8_t ANA_VERIFY_ROUNDS=5;
constexpr uint8_t ANA_VERIFY_WINDOWS_OFF=ANA_WINDOWS_NORMAL_OFF;
constexpr uint8_t ANA_VERIFY_WINDOWS_ON=ANA_WINDOWS_NORMAL_ON;
constexpr uint8_t ANA_BINS=64;
constexpr uint8_t ANA_BIN_SHIFT=6;

// -------- Efficient transition-balanced acquisition V6 --------
// Der reale 6.4.1-Export zeigte:
// - Training intern sehr gut,
// - Holdout deutlich schlechter,
// - alte Reihenfolge koppelte ON fast immer direkt an passendes OFF.
//
// V6 behaelt Transition-Vielfalt, optimiert aber gleichzeitig die reale
// Schaltarbeit. Jeder der 18 Zustaende hat ueber 4 Repeats mindestens
// 3 verschiedene Vorgaenger. Jeder ON-Zustand wird sowohl aus lokalem OFF
// als auch aus fremden Speed/Mode-Kontexten gelernt.
//
// Zusaetzlich werden Zielzustaende jetzt mit derselben DIREKTEN Bedienfolge
// wie im normalen Webbetrieb angefahren. Damit gibt es keinen Kalibrierpfad,
// den ein Nutzer spaeter im Alltag nie erzeugt.
//
// Gegenueber 6.4.2 sinkt die reine Beruhigungs-/Schaltzeit der Trainingspfade
// rechnerisch von ca. 1130 s auf ca. 486 s.
constexpr uint8_t ANA_TRAIN_ORDER[ANA_REPEATS][ANA_COMBOS] = {
  {13,12,7,6,16,17,14,15,4,5,3,2,0,1,9,8,11,10},
  {8,9,10,11,15,14,17,16,12,13,3,2,4,5,6,7,1,0},
  {11,10,6,7,9,8,14,15,17,16,1,0,5,4,12,13,2,3},
  {5,4,0,1,2,3,7,6,10,11,13,12,15,14,16,17,8,9}
};

inline uint8_t anaTrainingCombo(uint8_t repeat,uint8_t seq) {
  repeat=constrain(repeat,(uint8_t)0,(uint8_t)(ANA_REPEATS-1));
  seq%=ANA_COMBOS;
  return ANA_TRAIN_ORDER[repeat][seq];
}

constexpr uint8_t ANA_HOLDOUT_ROUNDS=3;
constexpr uint8_t ANA_HOLDOUT_BASE_ROUNDS=2;
constexpr uint8_t ANA_HOLDOUT_MAX_ADAPTIVE_STATES=6;
constexpr uint16_t ANA_HOLDOUT_BASE_TOTAL=(uint16_t)(ANA_COMBOS*ANA_HOLDOUT_BASE_ROUNDS);
constexpr uint16_t ANA_HOLDOUT_MAX_TOTAL=(uint16_t)(ANA_HOLDOUT_BASE_TOTAL+ANA_HOLDOUT_MAX_ADAPTIVE_STATES);

// Zwei volle, unabhaengige Holdout-Paesse sind Pflicht.
// Ein dritter Messwert wird NUR fuer einen Zustand aufgenommen, wenn nach
// zwei unterschiedlichen Transition-Pfaden genau eine Bestaetigung fehlt.
// 0/2 kann durch eine einzelne Zusatzmessung nicht mehr zu 2 Treffern werden
// und wird deshalb ohne weitere Ventilatorbelastung als echter Fehler markiert.
constexpr uint8_t ANA_HOLDOUT_ORDER[ANA_HOLDOUT_ROUNDS][ANA_COMBOS] = {
  {9,4,12,15,14,17,16,5,0,6,13,11,1,2,3,7,10,8},
  {4,3,6,7,2,1,0,15,10,14,12,16,17,11,5,13,8,9},
  {8,5,4,0,14,15,3,2,7,17,1,16,13,12,6,9,10,11}
};

// Fuer adaptive Runde 3 wird der vorgesehene Vorgaenger bewusst angefahren,
// auch wenn andere nicht benoetigte Zustaende der dritten Reihenfolge
// uebersprungen werden. Damit ist die Zusatzmessung wirklich ein dritter
// unabhaengiger Transition-Pfad und nicht zufaellig dieselbe Historie erneut.
constexpr uint8_t ANA_HOLDOUT_ADAPTIVE_PRIME[ANA_COMBOS] = {
  4,17,3,15,5,8,12,2,11,6,9,10,13,16,0,14,1,7
};

inline uint8_t anaHoldoutCombo(uint8_t round,uint8_t seq) {
  round=constrain(round,(uint8_t)0,(uint8_t)(ANA_HOLDOUT_ROUNDS-1));
  seq%=ANA_COMBOS;
  return ANA_HOLDOUT_ORDER[round][seq];
}

constexpr unsigned long ANA_SETTLE_SPEED_MS=2600;
constexpr unsigned long ANA_SETTLE_MODE_MS=3800;
constexpr unsigned long ANA_SETTLE_OSC_OFF_MS=4500;
constexpr unsigned long ANA_SETTLE_OSC_ON_MS=8000; // Motor/Mechanik anlaufen lassen

struct AnalyzeRun {
  bool running=false,done=false,aborted=false;
  bool preparing=false;
  bool resuming=false; // Checkpoint nach sicherem Netz-Power-On fortsetzen
  bool physicalStateTouched=false;
  uint8_t prepStage=0;
  volatile bool stopRequested=false;
  uint8_t repeat=0,combo=0;
  unsigned long nextStepMs=0;

  // Mess-Provenienz fuer KI-Export und sauberes Resume.
  uint32_t runStartBootId=0;
  uint32_t trainBootId[ANA_REPEATS]={0};
  uint8_t resumeCount=0;
  uint16_t resumeLoadedSteps=0;
  uint16_t resumeRollbackProfiles=0;

  float samples[ANA_COMBOS][5][ANA_REPEATS];
  float rawmean[ANA_COMBOS][5][ANA_REPEATS];
  uint16_t histC[ANA_COMBOS][5][ANA_REPEATS][ANA_BINS];
  uint16_t histN[ANA_COMBOS][5][ANA_REPEATS];

  uint16_t offHistC[5][ANA_BINS];
  uint16_t offHistN[5];
  float offPct[5],offMean[5];
  bool offMeasured=false;

  int bestThr[5];
  float scoreSpeed[5],scoreMode[5],scoreOsc[5],scoreOff[5];
  float rangeLo[5],rangeHi[5],weight[5]={1,1,1,1,1};

  float meanWeight[5]={1,1,1,1,1};
  float meanAlphaBest=0.0f;
  float levelAlphaBest=0.0f;
  float richWeight[5][RICH_CDF_LEVELS] = {};
  float fitLimit=12.0f;

  float shapeBest=0.0f,shapeScore=0.0f;
  bool optDone=false;

  // Unabhaengige AUS-Verifikation direkt nach der Optimierung.
  bool offVerifyDone=false;
  uint8_t offVerifyHits=0;
  float offVerifyWorstDrift=-1.0f;
  float offVerifyWorstRatio=0.0f;

  float verifyOffPct[5],verifyOnPct[5];
  float verifyOff=-1.0f,verifyOn=-1.0f;
  bool verifyDone=false;
  uint8_t verifyOffHits=0,verifyOnHits=0;

  uint16_t cvCorrect=0,cvTotal=0;
  uint16_t cvSpeedCorrect=0,cvModeCorrect=0,cvOscContextCorrect=0;
  uint8_t cvSpeedHits[ANA_COMBOS] = {};
  uint8_t cvModeHits[ANA_COMBOS] = {};
  uint8_t cvOscHits[ANA_COMBOS] = {};
  uint8_t cvContextStateFailures=0; // jeder Zustand mind. 3/4 je Dimension
  float cvWorstMargin=0.0f;

  // Unabhaengiger Holdout: 2 volle Transition-Paesse + adaptive dritte
  // Einzelmessung fuer maximal sechs Grenzfaelle.
  uint16_t holdCorrect=0,holdTotal=0;
  uint8_t holdStateFailures=0;
  uint16_t holdOscCorrect=0;
  uint16_t holdSpeedContextCorrect=0,holdModeContextCorrect=0,holdOscContextCorrect=0;
  uint8_t holdContextStateFailures=0; // jeder Zustand braucht 2 Kontexttreffer
  uint8_t holdAdaptiveStates=0;
  uint8_t holdImpossibleStates=0;
  float holdWorstMargin=0.0f;

  // Unabhaengige Holdout-Rohmerkmale fuer maschinenlesbaren KI-Export.
  float holdMeanSum[ANA_COMBOS][5] = {};
  float holdCdfSum[ANA_COMBOS][5][RICH_CDF_LEVELS] = {};
  uint8_t holdObs[ANA_COMBOS] = {};
  uint8_t holdGlobalHits[ANA_COMBOS] = {};
  uint8_t holdSpeedHits[ANA_COMBOS] = {};
  uint8_t holdModeHits[ANA_COMBOS] = {};
  uint8_t holdOscHits[ANA_COMBOS] = {};

  // Exakte 36 Pflicht-Holdouts + bis zu 6 adaptive Zusatzmessungen.
  // Damit kann eine KI jeden Transition-Pfad separat analysieren.
  float holdRoundMean[ANA_COMBOS][ANA_HOLDOUT_ROUNDS][5] = {};
  float holdRoundCdf[ANA_COMBOS][ANA_HOLDOUT_ROUNDS][5][RICH_CDF_LEVELS] = {};
  int8_t holdRoundPredecessor[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  int8_t holdRoundGlobalPred[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  int8_t holdRoundSpeedPred[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  int8_t holdRoundModePred[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  int8_t holdRoundOscPred[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  float holdRoundFit[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  float holdRoundTrueFit[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};
  float holdRoundMargin[ANA_COMBOS][ANA_HOLDOUT_ROUNDS] = {};

  uint8_t restoreSpeed=1,restoreMode=0;
  bool restoreOsc=false,wasOff=true;

  char action[120]="";
  char note[80]="";
} ana;

// ---------------- Analyse-Checkpoint / Resume ----------------
//
// Browser-Neuladen braucht KEIN Checkpoint: solange der ESP laeuft, bleibt
// ana komplett im RAM und die Analyse laeuft autonom weiter.
//
// Fuer echten ESP-/Stromausfall sichern wir ausschliesslich die teuren
// Trainings-Rohdaten in LittleFS. NVS bleibt damit fuer das kleine finale
// A/B-Kalibriermodell reserviert. Zwei CRC-gepruefte Dateislots verhindern,
// dass ein Stromausfall waehrend des Checkpoint-Schreibens den letzten guten
// Stand zerstoert.
//
// Resume wird nach einem ESP-Neustart absichtlich NUR nach ESP_RST_POWERON
// freigegeben. Dann wurde wegen der Versorgung aus der Fan-Platine auch der
// Ventilator selbst stromlos und seine physische Basis ist wieder eindeutig.
// Nach Watchdog/Software-Reset muss zuerst einmal komplett Netz AUS/EIN erfolgen.

constexpr uint32_t ANA_CP_MAGIC=0x414E4337u; // "ANC7"
constexpr uint16_t ANA_CP_VERSION=5;
constexpr const char* ANA_CP_FILE_A="/ana_cp_a.bin";
constexpr const char* ANA_CP_FILE_B="/ana_cp_b.bin";
constexpr uint8_t ANA_CP_EVERY_STEPS=3;

struct AnaCheckpointHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerBytes;
  uint32_t payloadBytes;
  uint32_t sequence;
  uint32_t crc32;

  uint8_t samplerVersion;
  uint8_t repeat;       // naechste noch nicht gemessene Wiederholung
  uint8_t combo;        // naechster Sequenzschritt innerhalb Wiederholung
  uint8_t restoreSpeed;
  uint8_t restoreMode;
  uint8_t restoreOsc;
  uint8_t wasOff;
  uint8_t resumeCount;

  uint32_t runStartBootId;
  int32_t thresholds[5];
};

bool anaCheckpointFsReady=false;
bool anaCheckpointCapable=false;
bool anaRecoveryAvailable=false;
bool anaRecoverySafePowerCycle=false;
uint32_t anaCheckpointSequence=0;
uint16_t anaCheckpointSavedSteps=0;
uint32_t anaCheckpointFreeBytes=0;
char anaCheckpointStatus[160]="nicht initialisiert";

uint32_t anaCheckpointPayloadBytes();
uint32_t anaCheckpointCrcUpdate(uint32_t crc,const uint8_t* data,size_t len);
bool anaValidateCheckpointFile(const char* path,AnaCheckpointHeader* headerOut);
void initAnalysisCheckpointStorage();
bool saveAnalysisCheckpoint(bool force);
bool loadAnalysisCheckpoint(String& err);
void clearAnalysisCheckpoint();
bool analyzeResume(String& err);
void anaSetAction(const char* fmt,...);
void sendAnalysisAiExport(bool includeHistograms);

uint32_t anaCheckpointPayloadBytes() {
  return (uint32_t)sizeof(ana.rawmean) +
         (uint32_t)sizeof(ana.histC) +
         (uint32_t)sizeof(ana.histN) +
         (uint32_t)sizeof(ana.trainBootId);
}

uint32_t anaCheckpointCrcUpdate(uint32_t crc,const uint8_t* data,size_t len) {
  for(size_t i=0;i<len;i++) {
    crc^=data[i];
    for(uint8_t b=0;b<8;b++)
      crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
  }
  return crc;
}

bool anaValidateCheckpointFile(const char* path,AnaCheckpointHeader* headerOut) {
  if(!anaCheckpointFsReady || !LittleFS.exists(path))return false;

  File f=LittleFS.open(path,"r");
  if(!f)return false;

  AnaCheckpointHeader h{};
  if(f.read((uint8_t*)&h,sizeof(h))!=sizeof(h)) {
    f.close();
    return false;
  }

  if(h.magic!=ANA_CP_MAGIC ||
     h.version!=ANA_CP_VERSION ||
     h.headerBytes!=sizeof(AnaCheckpointHeader) ||
     h.payloadBytes!=anaCheckpointPayloadBytes() ||
     h.samplerVersion!=CAL_SAMPLER_VERSION ||
     h.repeat>=ANA_REPEATS ||
     h.combo>=ANA_COMBOS ||
     f.size()!=(size_t)(sizeof(AnaCheckpointHeader)+h.payloadBytes)) {
    f.close();
    return false;
  }

  AnaCheckpointHeader hz=h;
  hz.crc32=0;

  uint32_t crc=0xFFFFFFFFu;
  crc=anaCheckpointCrcUpdate(crc,(const uint8_t*)&hz,sizeof(hz));

  uint8_t buf[512];
  while(f.available()) {
    size_t n=f.read(buf,sizeof(buf));
    if(n==0)break;
    crc=anaCheckpointCrcUpdate(crc,buf,n);
    yield();
  }
  f.close();
  crc^=0xFFFFFFFFu;

  if(crc==0 || crc!=h.crc32)return false;
  if(headerOut)*headerOut=h;
  return true;
}

void initAnalysisCheckpointStorage() {
  anaRecoverySafePowerCycle=(esp_reset_reason()==ESP_RST_POWERON);

  // Nie bei jedem Mount-Fehler blind formatieren: nach einem Stromausfall
  // koennte genau das die beiden Recovery-Slots vernichten.
  // Nur beim ALLERERSTEN Initialisieren darf ein leeres/unformatiertes
  // Filesystem automatisch angelegt werden.
  prefs.begin("anacpmeta",true);
  bool fsWasInitialized=prefs.getBool("initialized",false);
  prefs.end();

  anaCheckpointFsReady=LittleFS.begin(false);

  if(!anaCheckpointFsReady && !fsWasInitialized) {
    LittleFS.end();
    anaCheckpointFsReady=LittleFS.begin(true); // einmaliger First-Use-Format
  }

  if(anaCheckpointFsReady && !fsWasInitialized) {
    prefs.begin("anacpmeta",false);
    prefs.putBool("initialized",true);
    prefs.end();
    fsWasInitialized=true;
  }

  if(!anaCheckpointFsReady) {
    anaCheckpointCapable=false;
    anaRecoveryAvailable=false;
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
             fsWasInitialized?
             "LittleFS Mount-Fehler · NICHT formatiert · Recovery-Dateien geschuetzt":
             "LittleFS/Dateipartition nicht verfuegbar");
    Serial.printf("[ANA-CP] %s\n",anaCheckpointStatus);
    return;
  }

  uint32_t total=(uint32_t)LittleFS.totalBytes();
  uint32_t used=(uint32_t)LittleFS.usedBytes();
  anaCheckpointFreeBytes=total>used?total-used:0;

  // Zwei komplette ~60-kB-Slots plus Reserve fuer LittleFS-Metadaten.
  uint32_t one=(uint32_t)sizeof(AnaCheckpointHeader)+anaCheckpointPayloadBytes();
  uint32_t required=2u*one+32768u;
  anaCheckpointCapable=(total>=required);

  AnaCheckpointHeader a{},b{};
  bool va=anaValidateCheckpointFile(ANA_CP_FILE_A,&a);
  bool vb=anaValidateCheckpointFile(ANA_CP_FILE_B,&b);

  const AnaCheckpointHeader* newest=nullptr;
  if(va&&vb)newest=((int32_t)(a.sequence-b.sequence)>0)?&a:&b;
  else if(va)newest=&a;
  else if(vb)newest=&b;

  if(newest) {
    anaRecoveryAvailable=true;
    anaCheckpointSequence=newest->sequence;
    anaCheckpointSavedSteps=min((uint16_t)(ANA_COMBOS*ANA_REPEATS),
                                (uint16_t)(newest->repeat*ANA_COMBOS+newest->combo));
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
             "Checkpoint %u/%u · Seq %lu · A/B CRC",
             (unsigned)anaCheckpointSavedSteps,
             (unsigned)(ANA_COMBOS*ANA_REPEATS),
             (unsigned long)anaCheckpointSequence);
    Serial.printf("[ANA-CP] %s%s\n",anaCheckpointStatus,
                  anaRecoverySafePowerCycle?" · Resume sicher":" · Netz-Power-On vor Resume erforderlich");
  } else {
    anaRecoveryAvailable=false;
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
             anaCheckpointCapable?"bereit · A/B CRC":"Dateisystem zu klein fuer A/B-Resume");
    Serial.printf("[ANA-CP] %s · FS %lu/%lu Bytes frei\n",
                  anaCheckpointStatus,
                  (unsigned long)anaCheckpointFreeBytes,
                  (unsigned long)total);
  }
}

bool saveAnalysisCheckpoint(bool force) {
  if(!anaCheckpointFsReady || !anaCheckpointCapable || !ana.running)return false;

  uint16_t completed=min((uint16_t)(ANA_COMBOS*ANA_REPEATS),
                         (uint16_t)(ana.repeat*ANA_COMBOS+ana.combo));

  // repeat==5 ist bereits Post-Training. Der letzte echte Checkpoint
  // bleibt erhalten. Da Resume einen angebrochenen 18er-Repeat aus
  // Genauigkeitsgruenden komplett neu misst, ist kein zusaetzlicher
  // Sonder-Checkpoint kurz vor Trainingsende noetig.
  const uint16_t total=(uint16_t)(ANA_COMBOS*ANA_REPEATS);
  if(completed>=total)return true;

  bool checkpointPoint=(completed>0) &&
                       ((completed%ANA_CP_EVERY_STEPS)==0);
  if(!force && !checkpointPoint)return true;
  if(completed==0)return true;

  AnaCheckpointHeader h{};
  h.magic=ANA_CP_MAGIC;
  h.version=ANA_CP_VERSION;
  h.headerBytes=sizeof(AnaCheckpointHeader);
  h.payloadBytes=anaCheckpointPayloadBytes();
  h.sequence=anaCheckpointSequence+1;
  h.crc32=0;
  h.samplerVersion=CAL_SAMPLER_VERSION;
  h.repeat=ana.repeat;
  h.combo=ana.combo;
  h.restoreSpeed=ana.restoreSpeed;
  h.restoreMode=ana.restoreMode;
  h.restoreOsc=ana.restoreOsc?1:0;
  h.wasOff=ana.wasOff?1:0;
  h.resumeCount=ana.resumeCount;
  h.runStartBootId=ana.runStartBootId;
  for(int ch=0;ch<5;ch++)h.thresholds[ch]=fbThresholds[ch];

  const char* target=(h.sequence&1u)?ANA_CP_FILE_A:ANA_CP_FILE_B;
  if(LittleFS.exists(target))LittleFS.remove(target);

  File f=LittleFS.open(target,"w");
  if(!f) {
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
             "Checkpoint-Datei konnte nicht geoeffnet werden");
    return false;
  }

  // Header zunaechst mit crc32=0 schreiben.
  if(f.write((const uint8_t*)&h,sizeof(h))!=sizeof(h)) {
    f.close();
    LittleFS.remove(target);
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),"Checkpoint-Header Schreibfehler");
    return false;
  }

  uint32_t crc=0xFFFFFFFFu;
  crc=anaCheckpointCrcUpdate(crc,(const uint8_t*)&h,sizeof(h));

  auto writeBlock=[&](const void* ptr,size_t len)->bool{
    const uint8_t* p=(const uint8_t*)ptr;
    while(len) {
      size_t n=min((size_t)1024,len);
      if(f.write(p,n)!=n)return false;
      crc=anaCheckpointCrcUpdate(crc,p,n);
      p+=n;
      len-=n;
      anaServiceIo();
      if(ana.stopRequested)return false;
    }
    return true;
  };

  bool ok=writeBlock(ana.rawmean,sizeof(ana.rawmean)) &&
          writeBlock(ana.histC,sizeof(ana.histC)) &&
          writeBlock(ana.histN,sizeof(ana.histN)) &&
          writeBlock(ana.trainBootId,sizeof(ana.trainBootId));

  if(!ok) {
    f.close();
    LittleFS.remove(target);
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),"Checkpoint-Payload Schreibfehler");
    return false;
  }

  crc^=0xFFFFFFFFu;
  h.crc32=crc;

  if(!f.seek(0,SeekSet) ||
     f.write((const uint8_t*)&h,sizeof(h))!=sizeof(h)) {
    f.close();
    LittleFS.remove(target);
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),"Checkpoint-CRC Schreibfehler");
    return false;
  }
  f.flush();
  f.close();

  AnaCheckpointHeader verify{};
  if(!anaValidateCheckpointFile(target,&verify) ||
     verify.sequence!=h.sequence ||
     verify.repeat!=h.repeat ||
     verify.combo!=h.combo) {
    LittleFS.remove(target);
    snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),"Checkpoint-CRC Verifikation FEHLER");
    return false;
  }

  anaCheckpointSequence=h.sequence;
  anaCheckpointSavedSteps=completed;
  anaRecoveryAvailable=true;
  {
    uint32_t totalFs=(uint32_t)LittleFS.totalBytes();
    uint32_t usedFs=(uint32_t)LittleFS.usedBytes();
    anaCheckpointFreeBytes=totalFs>usedFs?totalFs-usedFs:0;
  }

  snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
           "gesichert %u/%u · Seq %lu · A/B CRC",
           (unsigned)completed,(unsigned)(ANA_COMBOS*ANA_REPEATS),
           (unsigned long)anaCheckpointSequence);
  Serial.printf("[ANA-CP] %s\n",anaCheckpointStatus);
  return true;
}

bool loadAnalysisCheckpoint(String& err) {
  if(!anaCheckpointFsReady || !anaRecoveryAvailable) {
    err="kein gueltiger Analyse-Checkpoint";
    return false;
  }

  AnaCheckpointHeader a{},b{};
  bool va=anaValidateCheckpointFile(ANA_CP_FILE_A,&a);
  bool vb=anaValidateCheckpointFile(ANA_CP_FILE_B,&b);

  const char* path=nullptr;
  AnaCheckpointHeader h{};
  if(va&&vb) {
    if((int32_t)(a.sequence-b.sequence)>0){path=ANA_CP_FILE_A;h=a;}
    else {path=ANA_CP_FILE_B;h=b;}
  } else if(va){path=ANA_CP_FILE_A;h=a;}
  else if(vb){path=ANA_CP_FILE_B;h=b;}
  else {
    anaRecoveryAvailable=false;
    err="Checkpoint-CRC ungueltig";
    return false;
  }

  File f=LittleFS.open(path,"r");
  if(!f || !f.seek(sizeof(AnaCheckpointHeader),SeekSet)) {
    if(f)f.close();
    err="Checkpoint kann nicht gelesen werden";
    return false;
  }

  auto readBlock=[&](void* ptr,size_t len)->bool{
    uint8_t* p=(uint8_t*)ptr;
    while(len) {
      size_t n=min((size_t)1024,len);
      size_t got=f.read(p,n);
      if(got!=n)return false;
      p+=n;
      len-=n;
      yield();
    }
    return true;
  };

  // Nur die teuren Trainingsdaten werden wiederhergestellt. Alle Holdout-/
  // Optimierungswerte werden nach Resume bewusst NEU erzeugt.
  memset(ana.samples,0,sizeof(ana.samples));
  memset(ana.rawmean,0,sizeof(ana.rawmean));
  memset(ana.histC,0,sizeof(ana.histC));
  memset(ana.histN,0,sizeof(ana.histN));

  bool ok=readBlock(ana.rawmean,sizeof(ana.rawmean)) &&
          readBlock(ana.histC,sizeof(ana.histC)) &&
          readBlock(ana.histN,sizeof(ana.histN)) &&
          readBlock(ana.trainBootId,sizeof(ana.trainBootId));
  f.close();

  if(!ok) {
    err="Checkpoint-Payload unvollstaendig";
    return false;
  }

  ana.repeat=h.repeat;
  ana.combo=h.combo;
  ana.restoreSpeed=constrain(h.restoreSpeed,(uint8_t)1,(uint8_t)3);
  ana.restoreMode=constrain(h.restoreMode,(uint8_t)0,(uint8_t)2);
  ana.restoreOsc=h.restoreOsc!=0;
  ana.wasOff=h.wasOff!=0;
  ana.resumeCount=h.resumeCount;
  ana.runStartBootId=h.runStartBootId;
  for(int ch=0;ch<5;ch++)fbThresholds[ch]=constrain((int)h.thresholds[ch],0,4095);

  anaCheckpointSequence=h.sequence;
  anaCheckpointSavedSteps=min((uint16_t)(ANA_COMBOS*ANA_REPEATS),
                              (uint16_t)(h.repeat*ANA_COMBOS+h.combo));
  return true;
}

void clearAnalysisCheckpoint() {
  if(anaCheckpointFsReady) {
    if(LittleFS.exists(ANA_CP_FILE_A))LittleFS.remove(ANA_CP_FILE_A);
    if(LittleFS.exists(ANA_CP_FILE_B))LittleFS.remove(ANA_CP_FILE_B);
  }
  anaRecoveryAvailable=false;
  anaCheckpointSequence=0;
  anaCheckpointSavedSteps=0;
  snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
           anaCheckpointCapable?"bereit · A/B CRC":"kein Reset-Checkpoint");
  Serial.println("[ANA-CP] Checkpoint verworfen");
}

bool analyzeResume(String& err) {
#if !ENABLE_LED_FEEDBACK
  err="LED-Feedback deaktiviert";
  return false;
#else
  if(ana.running){err="Messlauf laeuft bereits";return false;}
  if(learnBusy){err="manuelles Lernen laeuft";return false;}
  if(timerDeadlineMs){err="Sleep-Timer zuerst deaktivieren";return false;}
  if(!anaRecoveryAvailable){err="kein Checkpoint vorhanden";return false;}

  // Nach SW-/WDT-Reset ist der Fan selbst nicht sicher auf einer bekannten
  // Basis. Ein kompletter Netz-Power-On setzt Fan UND ESP gemeinsam zurueck.
  if(!anaRecoverySafePowerCycle) {
    err="Vor Resume Ventilator komplett vom Netz trennen und wieder einschalten";
    return false;
  }
  if(fanIsOn) {
    err="Nach Netz-Power-On Fan im Web AUS lassen und dann Resume starten";
    return false;
  }

  if(!loadAnalysisCheckpoint(err))return false;

  // Nach einem Netz-Neustart niemals nur ein einzelnes Profil in einen
  // alten 18er-Durchlauf mischen. Ein angebrochener Repeat wird komplett
  // geloescht und auf diesem Boot fuer ALLE 18 Zustaende neu gemessen.
  uint16_t loadedSteps=min((uint16_t)(ANA_COMBOS*ANA_REPEATS),
                           (uint16_t)(ana.repeat*ANA_COMBOS+ana.combo));
  uint16_t rollbackSteps=0;

  if(ana.combo>0 && ana.repeat<ANA_REPEATS) {
    uint8_t rr=ana.repeat;
    rollbackSteps=ana.combo;

    for(int c=0;c<ANA_COMBOS;c++) {
      for(int ch=0;ch<5;ch++) {
        ana.samples[c][ch][rr]=0.0f;
        ana.rawmean[c][ch][rr]=0.0f;
        ana.histN[c][ch][rr]=0;
        memset(ana.histC[c][ch][rr],0,sizeof(ana.histC[c][ch][rr]));
      }
    }

    ana.trainBootId[rr]=0;
    ana.combo=0;
    anaCheckpointSavedSteps=(uint16_t)(rr*ANA_COMBOS);
  }

  ana.resumeLoadedSteps=loadedSteps;
  ana.resumeRollbackProfiles=rollbackSteps;
  if(ana.resumeCount<255)ana.resumeCount++;

  memset(ana.offHistC,0,sizeof(ana.offHistC));
  memset(ana.offHistN,0,sizeof(ana.offHistN));
  memset(ana.offPct,0,sizeof(ana.offPct));
  memset(ana.offMean,0,sizeof(ana.offMean));
  memset(ana.holdMeanSum,0,sizeof(ana.holdMeanSum));
  memset(ana.holdCdfSum,0,sizeof(ana.holdCdfSum));
  memset(ana.holdObs,0,sizeof(ana.holdObs));
  memset(ana.holdGlobalHits,0,sizeof(ana.holdGlobalHits));
  memset(ana.holdSpeedHits,0,sizeof(ana.holdSpeedHits));
  memset(ana.holdModeHits,0,sizeof(ana.holdModeHits));
  memset(ana.holdOscHits,0,sizeof(ana.holdOscHits));
  memset(ana.holdRoundMean,0,sizeof(ana.holdRoundMean));
  memset(ana.holdRoundCdf,0,sizeof(ana.holdRoundCdf));
  memset(ana.holdRoundPredecessor,-1,sizeof(ana.holdRoundPredecessor));
  memset(ana.holdRoundGlobalPred,-1,sizeof(ana.holdRoundGlobalPred));
  memset(ana.holdRoundSpeedPred,-1,sizeof(ana.holdRoundSpeedPred));
  memset(ana.holdRoundModePred,-1,sizeof(ana.holdRoundModePred));
  memset(ana.holdRoundOscPred,-1,sizeof(ana.holdRoundOscPred));
  memset(ana.holdRoundFit,0,sizeof(ana.holdRoundFit));
  memset(ana.holdRoundTrueFit,0,sizeof(ana.holdRoundTrueFit));
  memset(ana.holdRoundMargin,0,sizeof(ana.holdRoundMargin));

  ana.offMeasured=false;
  ana.optDone=false;
  ana.verifyDone=false;
  ana.verifyOff=-1;
  ana.verifyOn=-1;
  ana.offVerifyDone=false;
  ana.offVerifyHits=0;
  ana.offVerifyWorstDrift=-1.0f;
  ana.offVerifyWorstRatio=0.0f;
  ana.verifyOffHits=0;
  ana.verifyOnHits=0;
  ana.cvCorrect=0;
  ana.cvTotal=0;
  ana.cvSpeedCorrect=0;
  ana.cvModeCorrect=0;
  ana.cvOscContextCorrect=0;
  memset(ana.cvSpeedHits,0,sizeof(ana.cvSpeedHits));
  memset(ana.cvModeHits,0,sizeof(ana.cvModeHits));
  memset(ana.cvOscHits,0,sizeof(ana.cvOscHits));
  ana.cvContextStateFailures=0;
  ana.cvWorstMargin=0.0f;
  ana.holdCorrect=0;
  ana.holdTotal=0;
  ana.holdStateFailures=0;
  ana.holdOscCorrect=0;
  ana.holdSpeedContextCorrect=0;
  ana.holdModeContextCorrect=0;
  ana.holdOscContextCorrect=0;
  ana.holdContextStateFailures=0;
  ana.holdAdaptiveStates=0;
  ana.holdImpossibleStates=0;
  ana.holdWorstMargin=0.0f;
  ana.meanAlphaBest=0.0f;
  ana.levelAlphaBest=0.0f;
  ana.fitLimit=12.0f;
  for(int ch=0;ch<5;ch++) {
    ana.meanWeight[ch]=1.0f;
    for(int q=0;q<RICH_CDF_LEVELS;q++)ana.richWeight[ch][q]=1.0f/(5.0f*RICH_CDF_LEVELS);
  }

  suppressFanStateSave=true;
  ana.stopRequested=false;
  ana.done=false;
  ana.aborted=false;
  ana.preparing=true;
  ana.resuming=true;
  ana.prepStage=0;
  ana.physicalStateTouched=false;
  ana.running=true;
  ana.nextStepMs=millis();
  ana.note[0]=0;

  anaSetAction("Resume: Checkpoint %u/%u · %u Profil(e) fuer balancierten Repeat erneut · sichere Netz-Basis",
               (unsigned)ana.resumeLoadedSteps,
               (unsigned)(ANA_COMBOS*ANA_REPEATS),
               (unsigned)ana.resumeRollbackProfiles);
  return true;
#endif
}

inline uint8_t anaSpeedOf(uint8_t c){return (c/6)+1;}
inline uint8_t anaModeOf(uint8_t c){return (c/2)%3;}
inline uint8_t anaOscOf(uint8_t c){return c%2;}
inline uint8_t anaIndex(uint8_t sp0,uint8_t md,uint8_t os){return sp0*6+md*2+os;}

void anaSetAction(const char* fmt,...) {
  va_list ap;va_start(ap,fmt);vsnprintf(ana.action,sizeof(ana.action),fmt,ap);va_end(ap);
  Serial.printf("[ANA18] %s\n",ana.action);
}

#if ENABLE_LED_FEEDBACK
void anaSampleComboHist(uint8_t combo,uint8_t rep,float meanOut[5]) {
  const uint8_t pins[5]={PIN_LED1_FB,PIN_LED2_FB,PIN_LED3_FB,PIN_LED8_FB,PIN_LED9_FB};
  double sum[5]={0,0,0,0,0};
  uint32_t rng=profileNewSeed(0xC0010000u ^ ((uint32_t)combo<<8) ^ rep ^ ana.histN[combo][0][rep]);
  delayMicroseconds((uint16_t)(profileRand32(rng)%2001u));

  for(int i=0;i<PROFILE_SAMPLE_CYCLES;i++) {
    uint32_t r=profileRand32(rng);
    int first=(int)(r%5u);
    int dir=(r&0x100u)?1:-1;

    for(int j=0;j<5;j++) {
      int ch=(first+dir*j+10)%5;
      (void)analogRead(pins[ch]);
      int v=constrain(analogRead(pins[ch]),0,4095);
      sum[ch]+=v;
      ana.histC[combo][ch][rep][v>>ANA_BIN_SHIFT]++;
      ana.histN[combo][ch][rep]++;
    }
    delayMicroseconds(profileJitterUs(rng));
  }

  for(int ch=0;ch<5;ch++)meanOut[ch]=sum[ch]/PROFILE_SAMPLE_CYCLES;
}

void anaSampleOffHist(float meanOut[5]) {
  const uint8_t pins[5]={PIN_LED1_FB,PIN_LED2_FB,PIN_LED3_FB,PIN_LED8_FB,PIN_LED9_FB};
  double sum[5]={0,0,0,0,0};
  uint32_t rng=profileNewSeed(0x0FF12345u ^ ana.offHistN[0]);
  delayMicroseconds((uint16_t)(profileRand32(rng)%2001u));

  for(int i=0;i<PROFILE_SAMPLE_CYCLES;i++) {
    uint32_t r=profileRand32(rng);
    int first=(int)(r%5u);
    int dir=(r&0x100u)?1:-1;

    for(int j=0;j<5;j++) {
      int ch=(first+dir*j+10)%5;
      (void)analogRead(pins[ch]);
      int v=constrain(analogRead(pins[ch]),0,4095);
      sum[ch]+=v;
      ana.offHistC[ch][v>>ANA_BIN_SHIFT]++;
      ana.offHistN[ch]++;
    }
    delayMicroseconds(profileJitterUs(rng));
  }

  for(int ch=0;ch<5;ch++)meanOut[ch]=sum[ch]/PROFILE_SAMPLE_CYCLES;
}
#endif

float anaAbovePct(uint8_t c,uint8_t ch,uint8_t r,uint8_t t) {
  uint32_t sum=0;for(uint8_t b=t;b<ANA_BINS;b++)sum+=ana.histC[c][ch][r][b];
  uint16_t n=ana.histN[c][ch][r];return n?(sum*100.0f/n):0.0f;
}
float anaOffPctAt(uint8_t ch,uint8_t t) {
  uint32_t sum=0;for(uint8_t b=t;b<ANA_BINS;b++)sum+=ana.offHistC[ch][b];
  return ana.offHistN[ch]?(sum*100.0f/ana.offHistN[ch]):0.0f;
}
void anaMeanSd(uint8_t c,uint8_t ch,uint8_t t,float& m,float& sd) {
  float v[ANA_REPEATS];m=0;
  for(int r=0;r<ANA_REPEATS;r++){v[r]=anaAbovePct(c,ch,r,t);m+=v[r];}
  m/=ANA_REPEATS;float a=0;
  for(int r=0;r<ANA_REPEATS;r++){float d=v[r]-m;a+=d*d;}
  sd=sqrtf(a/max(1,(int)ANA_REPEATS-1));
}

void anaServiceIo() {
  // Nur aus analyzeStep()/Analyse-Helfern aufrufen, niemals aus einem HTTP-Handler.
  // So bleibt das Webinterface auch waehrend langer Beruhigungs-/Holdout-Phasen
  // erreichbar, ohne WebServer::handleClient() rekursiv aufzurufen.
  server.handleClient();

  if(WiFi.status()!=WL_CONNECTED && millis()-lastWifiRetryMs>30000UL) {
    lastWifiRetryMs=millis();
    WiFi.reconnect();
  }
  yield();
}

bool anaMoveToState(uint8_t sp,uint8_t md,bool os) {
  sp=constrain(sp,(uint8_t)1,(uint8_t)3);
  md=constrain(md,(uint8_t)0,(uint8_t)2);
  if(!fanIsOn)return false;

  // V6: exakt dieselbe Bedienlogik wie spaetere Web-Kommandos.
  // Kein kuenstliches "Drehen AUS" und kein Umweg ueber Normal vor Speed.
  // So lernt Training/Holdout die Signalzustände nach REALEN Benutzerpfaden.
  //
  // Wartezeiten entsprechen den Runtime-Diagnose-Settles nahezu 1:1:
  // Speed ~2.6 s (Runtime 3 s), Modus ~3.8 s (Runtime 4 s),
  // Drehen AUS 4.5 s, Drehen AN 8 s.
  if(state.speed!=sp) {
    ana.physicalStateTouched=true;
    setSpeed(sp);
    if(!anaResponsiveDelay(ANA_SETTLE_SPEED_MS))return false;
  }

  if(state.mode!=md) {
    ana.physicalStateTouched=true;
    setMode(md);
    if(!anaResponsiveDelay(ANA_SETTLE_MODE_MS))return false;
  }

  if(state.osc!=os) {
    ana.physicalStateTouched=true;
    toggleOsc();
    if(!anaResponsiveDelay(os?ANA_SETTLE_OSC_ON_MS:ANA_SETTLE_OSC_OFF_MS))return false;
  }

  return state.speed==sp && state.mode==md && state.osc==os;
}

bool anaRestoreUserState() {
  // Stop-Anforderung gilt fuer die Messung, nicht fuer die sichere Rueckfahrt.
  ana.stopRequested=false;

  if(ana.wasOff) {
    // WICHTIG: Die Fan-Platine merkt Speed/Modus/Drehen auch im AUS-Zustand.
    // Die Analyse hat diesen latenten Zustand veraendert. Deshalb muessen wir
    // vor dem finalen Ausschalten den urspruenglichen letzten Nutzerzustand
    // PHYSISCH wieder herstellen. Nur Software-lastSpeed umzuschreiben wuerde
    // beim naechsten EIN zu einem echten Hardware/Web-Mismatch fuehren.
    if(ana.physicalStateTouched) {
      if(!fanIsOn) {
        togglePower();
        ana.physicalStateTouched=true;
        if(!anaResponsiveDelay(4200))return false;
      }

      if(!anaMoveToState(ana.restoreSpeed,ana.restoreMode,ana.restoreOsc))
        return false;

      powerOff();
      if(!anaResponsiveDelay(900))return false;
    } else if(fanIsOn) {
      // Sicherheitsfall: Analyse hat laut Flag nichts veraendert, Fan ist aber an.
      powerOff();
      if(!anaResponsiveDelay(900))return false;
    }

    state.lastSpeed=ana.restoreSpeed;
    state.lastMode=ana.restoreMode;
    state.lastOsc=ana.restoreOsc;
    return !fanIsOn;
  }

  if(!fanIsOn) {
    togglePower();
    ana.physicalStateTouched=true;
    if(!anaResponsiveDelay(4200))return false;
  }

  if(!anaMoveToState(ana.restoreSpeed,ana.restoreMode,ana.restoreOsc))
    return false;

  state.lastSpeed=ana.restoreSpeed;
  state.lastMode=ana.restoreMode;
  state.lastOsc=ana.restoreOsc;
  return true;
}

void analyzeAbort(const char* why) {
  char whyCopy[96];
  snprintf(whyCopy,sizeof(whyCopy),"%s",why?why:"unbekannt");

  // Messung stoppen, den echten Nutzerzustand aber noch mit unterdrueckten
  // Zwischen-Saves physisch wiederherstellen.
  ana.running=true;   // Bedienrouten bis nach der Rueckfahrt gesperrt lassen
  ana.done=false;
  ana.aborted=true;
  ana.stopRequested=false;

  bool restored=anaRestoreUserState();

  ana.preparing=false;
  ana.resuming=false;
  ana.running=false;
  suppressFanStateSave=false;
  clearAnalysisCheckpoint();
  bumpStateRevision();
  saveFanState();

  if(fanIsOn)armLedDiagnostic(state.osc?8000UL:LED_DETECT_SETTLE_MS);
  else {ledDetectReadyMs=0;resetStableDiagnostic();}

  snprintf(ana.action,sizeof(ana.action),"Abgebrochen: %s%s",whyCopy,restored?"":" · Rueckfahrt unvollstaendig");
  strncpy(ana.note,restored?"Messlauf abgebrochen · Nutzerzustand wiederhergestellt":"Messlauf abgebrochen · Zustand bitte pruefen",sizeof(ana.note)-1);
  ana.note[sizeof(ana.note)-1]=0;
}


// Der HO-5500RE steht beim definierten Start der Abschlussanalyse physisch auf:
// Stufe 1 / Normal / Drehen AUS.
// Die normale togglePower()-Funktion stellt absichtlich den zuletzt bekannten
// Web-Zustand wieder her. Fuer die Kalibrierung waere genau das falsch, weil
// dadurch insbesondere state.osc gegenueber dem physischen Drehen invertiert
// sein kann. Die Analyse besitzt deshalb ab hier eine feste Referenzbasis.
bool anaNormalizePhysicalBaseline(String& err) {
  if(!fanIsOn) {
    err="Ventilator muss fuer die Basis eingeschaltet sein";
    return false;
  }

  // Nicht mehr blind Softwarewerte auf St1/Normal/AUS setzen.
  // Die Fan-Platine behaelt bei normalem AUS/AN ihren Zustand. Deshalb
  // fahren wir den bekannten, persistenten Befehlszustand physisch zur Basis.
  if(state.osc) {
    anaSetAction("Basis: Drehen -> AUS");
    toggleOsc();
    if(!anaResponsiveDelay(ANA_SETTLE_OSC_OFF_MS)){err="abgebrochen";return false;}
  }

  if(state.mode!=0) {
    anaSetAction("Basis: Modus -> Normal");
    setMode(0);
    if(!anaResponsiveDelay(ANA_SETTLE_MODE_MS)){err="abgebrochen";return false;}
  }

  if(state.speed!=1) {
    anaSetAction("Basis: Stufe -> 1");
    setSpeed(1);
    if(!anaResponsiveDelay(ANA_SETTLE_SPEED_MS)){err="abgebrochen";return false;}
  }

  // Ab hier stimmen physischer und interner Zustand konstruktiv ueberein.
  state.speed=1;
  state.mode=0;
  state.osc=false;

  ledDetectReadyMs=0;
  resetStableDiagnostic();

  Serial.println("[ANA18] Physische Basis hergestellt: Stufe 1 / Normal / Drehen AUS");
  return true;
}

bool analyzeStart(String& err) {
#if !ENABLE_LED_FEEDBACK
  err="LED-Feedback deaktiviert";return false;
#else
  if(ana.running){err="laeuft bereits";return false;}
  if(learnBusy){err="manuelles Lernen laeuft";return false;}
  if(timerDeadlineMs){err="Sleep-Timer zuerst deaktivieren";return false;}
  if(anaRecoveryAvailable){
    err="Checkpoint vorhanden - zuerst fortsetzen oder verwerfen";
    return false;
  }

  // Ein neuer Lauf beginnt bewusst ohne alte Recovery-Daten.
  clearAnalysisCheckpoint();

  ana.wasOff=!fanIsOn;
  ana.restoreSpeed=fanIsOn&&state.speed>=1?state.speed:state.lastSpeed;
  if(ana.restoreSpeed<1||ana.restoreSpeed>3)ana.restoreSpeed=1;
  ana.restoreMode=fanIsOn?state.mode:state.lastMode;
  if(ana.restoreMode>2)ana.restoreMode=0;
  ana.restoreOsc=fanIsOn?state.osc:state.lastOsc;

  // Ab hier nur noch Initialisierung. KEINE langen Delays, keine ADC-Serie und
  // kein server.handleClient() innerhalb des POST-Handlers. Der HTTP-Start kann
  // dadurch sofort mit 200 antworten; die physische Vorbereitung beginnt erst
  // danach in analyzeStep().
  suppressFanStateSave=true;
  ana.stopRequested=false;

  memset(ana.samples,0,sizeof(ana.samples));
  memset(ana.rawmean,0,sizeof(ana.rawmean));
  memset(ana.histC,0,sizeof(ana.histC));
  memset(ana.histN,0,sizeof(ana.histN));
  memset(ana.offHistC,0,sizeof(ana.offHistC));
  memset(ana.offHistN,0,sizeof(ana.offHistN));
  memset(ana.offPct,0,sizeof(ana.offPct));
  memset(ana.offMean,0,sizeof(ana.offMean));
  memset(ana.trainBootId,0,sizeof(ana.trainBootId));
  memset(ana.holdMeanSum,0,sizeof(ana.holdMeanSum));
  memset(ana.holdCdfSum,0,sizeof(ana.holdCdfSum));
  memset(ana.holdObs,0,sizeof(ana.holdObs));
  memset(ana.holdGlobalHits,0,sizeof(ana.holdGlobalHits));
  memset(ana.holdSpeedHits,0,sizeof(ana.holdSpeedHits));
  memset(ana.holdModeHits,0,sizeof(ana.holdModeHits));
  memset(ana.holdOscHits,0,sizeof(ana.holdOscHits));
  memset(ana.holdRoundMean,0,sizeof(ana.holdRoundMean));
  memset(ana.holdRoundCdf,0,sizeof(ana.holdRoundCdf));
  memset(ana.holdRoundPredecessor,-1,sizeof(ana.holdRoundPredecessor));
  memset(ana.holdRoundGlobalPred,-1,sizeof(ana.holdRoundGlobalPred));
  memset(ana.holdRoundSpeedPred,-1,sizeof(ana.holdRoundSpeedPred));
  memset(ana.holdRoundModePred,-1,sizeof(ana.holdRoundModePred));
  memset(ana.holdRoundOscPred,-1,sizeof(ana.holdRoundOscPred));
  memset(ana.holdRoundFit,0,sizeof(ana.holdRoundFit));
  memset(ana.holdRoundTrueFit,0,sizeof(ana.holdRoundTrueFit));
  memset(ana.holdRoundMargin,0,sizeof(ana.holdRoundMargin));

  ana.runStartBootId=bootSessionId;
  ana.resumeCount=0;
  ana.resumeLoadedSteps=0;
  ana.resumeRollbackProfiles=0;

  ana.offMeasured=false;
  ana.optDone=false;
  ana.verifyDone=false;
  ana.verifyOff=-1;
  ana.verifyOn=-1;
  ana.offVerifyDone=false;
  ana.offVerifyHits=0;
  ana.offVerifyWorstDrift=-1.0f;
  ana.offVerifyWorstRatio=0.0f;
  ana.verifyOffHits=0;
  ana.verifyOnHits=0;
  ana.cvCorrect=0;
  ana.cvTotal=0;
  ana.cvSpeedCorrect=0;
  ana.cvModeCorrect=0;
  ana.cvOscContextCorrect=0;
  memset(ana.cvSpeedHits,0,sizeof(ana.cvSpeedHits));
  memset(ana.cvModeHits,0,sizeof(ana.cvModeHits));
  memset(ana.cvOscHits,0,sizeof(ana.cvOscHits));
  ana.cvContextStateFailures=0;
  ana.cvWorstMargin=0.0f;
  ana.holdCorrect=0;
  ana.holdTotal=0;
  ana.holdStateFailures=0;
  ana.holdOscCorrect=0;
  ana.holdSpeedContextCorrect=0;
  ana.holdModeContextCorrect=0;
  ana.holdOscContextCorrect=0;
  ana.holdContextStateFailures=0;
  ana.holdAdaptiveStates=0;
  ana.holdImpossibleStates=0;
  ana.holdWorstMargin=0.0f;
  ana.meanAlphaBest=0.0f;
  ana.levelAlphaBest=0.0f;
  ana.fitLimit=12.0f;
  for(int ch=0;ch<5;ch++) {
    ana.meanWeight[ch]=1.0f;
    for(int q=0;q<RICH_CDF_LEVELS;q++)ana.richWeight[ch][q]=1.0f/(5.0f*RICH_CDF_LEVELS);
  }

  ana.repeat=0;
  ana.combo=0;
  ana.done=false;
  ana.aborted=false;
  ana.preparing=true;
  ana.resuming=false;
  ana.prepStage=0;
  ana.physicalStateTouched=false;
  ana.running=true;
  ana.nextStepMs=millis();
  ana.note[0]=0;
  anaSetAction("Vorbereitung: Startzustand wird geprueft");
  return true;
#endif
}

#if ENABLE_LED_FEEDBACK
void anaOptimize();

inline float anaRichPct(uint8_t c,uint8_t ch,uint8_t r,uint8_t q) {
  uint8_t t=(uint8_t)(RICH_CDF_ADC[q]>>ANA_BIN_SHIFT);
  return anaAbovePct(c,ch,r,t);
}

void anaRichProfileMean(uint8_t c,float rich[5][RICH_CDF_LEVELS]) {
  for(int ch=0;ch<5;ch++)
    for(int q=0;q<RICH_CDF_LEVELS;q++) {
      float s=0.0f;
      for(int r=0;r<ANA_REPEATS;r++)s+=anaRichPct(c,ch,r,q);
      rich[ch][q]=s/ANA_REPEATS;
    }
}

void anaRichReferenceExcluding(uint8_t profile,
                               uint8_t testProfile,uint8_t testRepeat,
                               float rich[5][RICH_CDF_LEVELS],
                               float meanNorm[5],float& level) {
  float raw[5]={0,0,0,0,0};

  for(int ch=0;ch<5;ch++) {
    float ms=0.0f;
    int n=0;
    for(int r=0;r<ANA_REPEATS;r++) {
      if(profile==testProfile && r==testRepeat)continue;
      ms+=ana.rawmean[profile][ch][r];
      n++;
    }
    raw[ch]=n?ms/n:0.0f;

    for(int q=0;q<RICH_CDF_LEVELS;q++) {
      float ps=0.0f;
      int pn=0;
      for(int r=0;r<ANA_REPEATS;r++) {
        if(profile==testProfile && r==testRepeat)continue;
        ps+=anaRichPct(profile,ch,r,q);
        pn++;
      }
      rich[ch][q]=pn?ps/pn:0.0f;
    }
  }

  normalizedMeanFeature(raw,meanNorm);
  level=rawLevelFeature(raw);
}

void anaRichTestFeatures(uint8_t c,uint8_t r,
                         float rich[5][RICH_CDF_LEVELS],
                         float meanNorm[5],float& level) {
  float raw[5];
  for(int ch=0;ch<5;ch++) {
    raw[ch]=ana.rawmean[c][ch][r];
    for(int q=0;q<RICH_CDF_LEVELS;q++)rich[ch][q]=anaRichPct(c,ch,r,q);
  }
  normalizedMeanFeature(raw,meanNorm);
  level=rawLevelFeature(raw);
}

float anaRichDistance(const float testRich[5][RICH_CDF_LEVELS],
                      const float testMean[5],float testLevel,
                      const float refRich[5][RICH_CDF_LEVELS],
                      const float refMean[5],float refLevel,
                      float shape,float meanAlpha,float levelAlpha) {
  float d=0.0f;

  // Pro CDF-Pegel common-mode-Anteil optional abziehen. Dadurch koennen
  // Speed/Mode-Kontraste genutzt werden, ohne die absolute Helligkeits-
  // information fuer Drehen vollstaendig zu verlieren.
  for(int q=0;q<RICH_CDF_LEVELS;q++) {
    float tm=0.0f,rm=0.0f;
    for(int ch=0;ch<5;ch++){tm+=testRich[ch][q];rm+=refRich[ch][q];}
    tm/=5.0f;rm/=5.0f;

    for(int ch=0;ch<5;ch++) {
      float te=testRich[ch][q]-shape*tm;
      float re=refRich[ch][q]-shape*rm;
      float e=te-re;
      d+=ana.richWeight[ch][q]*e*e;
    }
  }

  for(int ch=0;ch<5;ch++) {
    float e=testMean[ch]-refMean[ch];
    d+=meanAlpha*ana.meanWeight[ch]*e*e;
  }

  float el=testLevel-refLevel;
  d+=levelAlpha*el*el;
  return d;
}

int anaRichContextWinner(const float testRich[5][RICH_CDF_LEVELS],
                         const float testMean[5],float testLevel,
                         uint8_t trueCombo,uint8_t dimension,
                         float shape,float meanAlpha,float levelAlpha) {
  int tsp=anaSpeedOf(trueCombo)-1;
  int tmd=anaModeOf(trueCombo);
  int tos=anaOscOf(trueCombo);

  float best=1e30f;
  int bestValue=-1;

  for(int c=0;c<ANA_COMBOS;c++) {
    int sp=anaSpeedOf(c)-1,md=anaModeOf(c),os=anaOscOf(c);

    if(dimension==0 && (md!=tmd || os!=tos))continue; // Speed
    if(dimension==1 && (sp!=tsp || os!=tos))continue; // Mode
    if(dimension==2 && (sp!=tsp || md!=tmd))continue; // Osc

    float rr[5][RICH_CDF_LEVELS],rm[5],rl=0.0f;
    anaRichProfileMean(c,rr);
    float raw[5];anaProfileRawMean(c,raw);normalizedMeanFeature(raw,rm);rl=rawLevelFeature(raw);

    float d=anaRichDistance(testRich,testMean,testLevel,rr,rm,rl,
                            shape,meanAlpha,levelAlpha);
    if(d<best) {
      best=d;
      bestValue=(dimension==0)?sp:((dimension==1)?md:os);
    }
  }
  return bestValue;
}

void anaProfileMean(uint8_t c,const uint8_t tb[5],float pct[5]) {
  for(int ch=0;ch<5;ch++) {
    float s=0;
    for(int r=0;r<ANA_REPEATS;r++)s+=anaAbovePct(c,ch,r,tb[ch]);
    pct[ch]=s/ANA_REPEATS;
  }
}

void anaProfileRawMean(uint8_t c,float meanv[5]) {
  for(int ch=0;ch<5;ch++) {
    float s=0;
    for(int r=0;r<ANA_REPEATS;r++)s+=ana.rawmean[c][ch][r];
    meanv[ch]=s/ANA_REPEATS;
  }
}

void anaReferenceExcluding(uint8_t profile,
                           uint8_t testProfile,uint8_t testRepeat,
                           const uint8_t tb[5],
                           float pct[5],float meanNorm[5]) {
  float raw[5]={0,0,0,0,0};

  for(int ch=0;ch<5;ch++) {
    float ps=0,ms=0;
    int n=0;

    for(int r=0;r<ANA_REPEATS;r++) {
      if(profile==testProfile && r==testRepeat)continue;
      ps+=anaAbovePct(profile,ch,r,tb[ch]);
      ms+=ana.rawmean[profile][ch][r];
      n++;
    }

    pct[ch]=n?ps/n:0.0f;
    raw[ch]=n?ms/n:0.0f;
  }

  normalizedMeanFeature(raw,meanNorm);
}

void anaTestFeatures(uint8_t c,uint8_t r,const uint8_t tb[5],
                     float pct[5],float meanNorm[5]) {
  float raw[5];
  for(int ch=0;ch<5;ch++) {
    pct[ch]=anaAbovePct(c,ch,r,tb[ch]);
    raw[ch]=ana.rawmean[c][ch][r];
  }
  normalizedMeanFeature(raw,meanNorm);
}

float anaFeatureDistance(const float testPct[5],const float testMean[5],
                         const float refPct[5],const float refMean[5],
                         float shape,float meanAlpha) {
  float tf[5],rf[5];
  comboFeature(testPct,shape,tf);
  comboFeature(refPct,shape,rf);

  float d=0;
  for(int ch=0;ch<5;ch++) {
    float ep=tf[ch]-rf[ch];
    float em=testMean[ch]-refMean[ch];
    d+=ana.weight[ch]*ep*ep;
    d+=meanAlpha*ana.meanWeight[ch]*em*em;
  }
  return d;
}

int anaClassifyStats(const FbStats cur[5],float* marginOut,float* fitOut) {
  float testRich[5][RICH_CDF_LEVELS],testMean[5];
  for(int ch=0;ch<5;ch++)
    for(int q=0;q<RICH_CDF_LEVELS;q++)testRich[ch][q]=cur[ch].cdfPct[q];
  normalizedMeanFeature(cur,testMean);
  float testLevel=rawLevelFeature(cur);

  float best=1e30f,second=1e30f;
  int bestC=-1;

  for(int c=0;c<ANA_COMBOS;c++) {
    float refRich[5][RICH_CDF_LEVELS],refRaw[5],refMean[5];
    anaRichProfileMean(c,refRich);
    anaProfileRawMean(c,refRaw);
    normalizedMeanFeature(refRaw,refMean);
    float refLevel=rawLevelFeature(refRaw);

    float d=anaRichDistance(testRich,testMean,testLevel,
                            refRich,refMean,refLevel,
                            ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
    if(d<best){second=best;best=d;bestC=c;}
    else if(d<second)second=d;
  }

  float rb=sqrtf(max(best,0.0f));
  float rs=sqrtf(max(second,0.0f));
  float margin=(rs>0.0001f)?((rs-rb)/rs*100.0f):0.0f;
  if(marginOut)*marginOut=margin;

  float lim=max(ana.fitLimit,0.25f);
  float fit=rb<=lim?100.0f:constrain(1.0f-(rb-lim)/lim,0.0f,1.0f)*100.0f;
  if(fitOut)*fitOut=fit;

  return bestC;
}

bool anaSampleProfileAverageResponsive(FbStats cur[5],uint8_t windows,uint32_t salt) {
  windows=constrain(windows,(uint8_t)1,(uint8_t)10);

  double sumMean[5]={0,0,0,0,0};
  double sumPct[5]={0,0,0,0,0};
  double sumCdf[5][RICH_CDF_LEVELS] = {};
  int minv[5]={4095,4095,4095,4095,4095};
  int maxv[5]={0,0,0,0,0};

  uint32_t rng=profileNewSeed(salt);

  for(uint8_t w=0;w<windows;w++) {
    FbStats one[5];
    sampleProfileWindow(one);

    for(int ch=0;ch<5;ch++) {
      sumMean[ch]+=one[ch].avg;
      sumPct[ch]+=one[ch].abovePct;
      for(int q=0;q<RICH_CDF_LEVELS;q++)sumCdf[ch][q]+=one[ch].cdfPct[q];
      if(one[ch].minv<minv[ch])minv[ch]=one[ch].minv;
      if(one[ch].maxv>maxv[ch])maxv[ch]=one[ch].maxv;
    }

    if(w+1<windows) {
      if(!anaResponsiveDelay(profileSeriesGapMs(rng)))return false;
    } else {
      anaServiceIo();
      if(ana.stopRequested)return false;
    }
  }

  for(int ch=0;ch<5;ch++) {
    cur[ch].avg=(int)(sumMean[ch]/windows+0.5);
    cur[ch].abovePct=(float)(sumPct[ch]/windows);
    cur[ch].belowPct=100.0f-cur[ch].abovePct;
    for(int q=0;q<RICH_CDF_LEVELS;q++)cur[ch].cdfPct[q]=(float)(sumCdf[ch][q]/windows);
    cur[ch].minv=minv[ch];
    cur[ch].maxv=maxv[ch];
    cur[ch].rangev=maxv[ch]-minv[ch];
  }

  return true;
}


void anaVerifyOffCurrent() {
  ana.offVerifyDone=false;
  ana.offVerifyHits=0;
  ana.offVerifyWorstDrift=0.0f;
  ana.offVerifyWorstRatio=0.0f;

  if(!ana.offMeasured)return;

  int oldThr[5];
  uint8_t tb[5];
  float refPct[5];
  for(int ch=0;ch<5;ch++) {
    oldThr[ch]=fbThresholds[ch];
    fbThresholds[ch]=ana.bestThr[ch];
    tb[ch]=constrain(ana.bestThr[ch]>>ANA_BIN_SHIFT,1,ANA_BINS-1);
    refPct[ch]=anaOffPctAt(ch,tb[ch]);
  }

  float refOffTotal=0.0f;
  for(int ch=0;ch<5;ch++)refOffTotal+=ana.offMean[ch];

  // Naechster EIN-Pegel aus den 18 Trainingszentren.
  float minOnTotal=1e30f;
  for(int c=0;c<ANA_COMBOS;c++) {
    float total=0.0f;
    for(int ch=0;ch<5;ch++) {
      float m=0.0f;
      for(int r=0;r<ANA_REPEATS;r++)m+=ana.rawmean[c][ch][r];
      total+=m/ANA_REPEATS;
    }
    minOnTotal=min(minOnTotal,total);
  }

  constexpr uint8_t ROUNDS=3;
  for(uint8_t r=0;r<ROUNDS;r++) {
    FbStats cur[5];
    if(!anaSampleProfileAverageResponsive(cur,ANA_WINDOWS_NORMAL_OFF,
                                          0x0FF44000u ^ r))break;

    float dOff=0.0f,bestOn=1e30f;
    float curTotal=0.0f;
    float drift=0.0f;

    for(int ch=0;ch<5;ch++) {
      float e=cur[ch].abovePct-refPct[ch];
      dOff+=ana.weight[ch]*e*e;
      curTotal+=cur[ch].avg;
      drift=max(drift,fabsf(e));
    }

    for(int c=0;c<ANA_COMBOS;c++) {
      float d=0.0f;
      for(int ch=0;ch<5;ch++) {
        float p=0.0f;
        for(int rr=0;rr<ANA_REPEATS;rr++)p+=anaAbovePct(c,ch,rr,tb[ch]);
        p/=ANA_REPEATS;
        float e=cur[ch].abovePct-p;
        d+=ana.weight[ch]*e*e;
      }
      bestOn=min(bestOn,d);
    }

    float offErr=fabsf(curTotal-refOffTotal);
    float onErr=fabsf(curTotal-minOnTotal);
    float sep=max(minOnTotal-refOffTotal,1.0f);
    float ratio=offErr/sep*100.0f;

    ana.offVerifyWorstDrift=max(ana.offVerifyWorstDrift,drift);
    ana.offVerifyWorstRatio=max(ana.offVerifyWorstRatio,ratio);

    // Drei unabhaengige Kriterien: Prozentmuster naeher an AUS, absoluter
    // ADC-Pegel naeher an AUS und kein grosser Phasen-/Jitter-Drift.
    if(dOff<bestOn && offErr<onErr && drift<3.0f && ratio<20.0f)
      ana.offVerifyHits++;

    if(!anaResponsiveDelay(85+(r*17)))break;
  }

  for(int ch=0;ch<5;ch++)fbThresholds[ch]=oldThr[ch];
  ana.offVerifyDone=!ana.stopRequested;
}

void anaVerifyCurrent(uint8_t expectedCombo,uint8_t windowsPerRound,
                      float outPct[5],float& worstMeanDrift,uint8_t& correctHits) {
  for(int ch=0;ch<5;ch++)outPct[ch]=0.0f;
  correctHits=0;

  int oldThr[5];
  for(int ch=0;ch<5;ch++) {
    oldThr[ch]=fbThresholds[ch];
    fbThresholds[ch]=ana.bestThr[ch];
  }

  for(int r=0;r<ANA_VERIFY_ROUNDS;r++) {
    FbStats cur[5];
    if(!anaSampleProfileAverageResponsive(cur,windowsPerRound,
                                          0xA11CE000u ^ ((uint32_t)expectedCombo<<8) ^ r))
      break;

    for(int ch=0;ch<5;ch++)
      outPct[ch]+=cur[ch].abovePct/ANA_VERIFY_ROUNDS;

    float margin=0,fit=0;
    if(anaClassifyStats(cur,&margin,&fit)==expectedCombo && fit>=50.0f)
      correctHits++;

    if(!anaResponsiveDelay(70+(r*13)))break;
  }

  for(int ch=0;ch<5;ch++)fbThresholds[ch]=oldThr[ch];

  uint8_t tb[5];
  for(int ch=0;ch<5;ch++)
    tb[ch]=constrain(ana.bestThr[ch]>>ANA_BIN_SHIFT,1,ANA_BINS-1);

  worstMeanDrift=0.0f;
  for(int ch=0;ch<5;ch++) {
    float ref=0;
    for(int r=0;r<ANA_REPEATS;r++)
      ref+=anaAbovePct(expectedCombo,ch,r,tb[ch]);
    ref/=ANA_REPEATS;
    worstMeanDrift=max(worstMeanDrift,fabsf(outPct[ch]-ref));
  }
}

bool anaResponsiveDelay(unsigned long ms) {
  unsigned long end=millis()+ms;
  while((long)(millis()-end)<0) {
    anaServiceIo();
    if(ana.stopRequested)return false;
    delay(12);
  }
  return true;
}

// Physische Zielzustandsfahrt fuer die unabhaengige Holdout-Runde.
bool anaMoveToCombo(uint8_t combo) {
  return anaMoveToState(anaSpeedOf(combo),anaModeOf(combo),anaOscOf(combo)!=0);
}

bool anaValidateAll18() {
  ana.holdCorrect=0;
  ana.holdTotal=0;
  ana.holdStateFailures=0;
  ana.holdOscCorrect=0;
  ana.holdSpeedContextCorrect=0;
  ana.holdModeContextCorrect=0;
  ana.holdOscContextCorrect=0;
  ana.holdContextStateFailures=0;
  ana.holdAdaptiveStates=0;
  ana.holdImpossibleStates=0;
  ana.holdWorstMargin=100.0f;

  memset(ana.holdMeanSum,0,sizeof(ana.holdMeanSum));
  memset(ana.holdCdfSum,0,sizeof(ana.holdCdfSum));
  memset(ana.holdObs,0,sizeof(ana.holdObs));
  memset(ana.holdGlobalHits,0,sizeof(ana.holdGlobalHits));
  memset(ana.holdSpeedHits,0,sizeof(ana.holdSpeedHits));
  memset(ana.holdModeHits,0,sizeof(ana.holdModeHits));
  memset(ana.holdOscHits,0,sizeof(ana.holdOscHits));
  memset(ana.holdRoundMean,0,sizeof(ana.holdRoundMean));
  memset(ana.holdRoundCdf,0,sizeof(ana.holdRoundCdf));
  memset(ana.holdRoundPredecessor,-1,sizeof(ana.holdRoundPredecessor));
  memset(ana.holdRoundGlobalPred,-1,sizeof(ana.holdRoundGlobalPred));
  memset(ana.holdRoundSpeedPred,-1,sizeof(ana.holdRoundSpeedPred));
  memset(ana.holdRoundModePred,-1,sizeof(ana.holdRoundModePred));
  memset(ana.holdRoundOscPred,-1,sizeof(ana.holdRoundOscPred));
  memset(ana.holdRoundFit,0,sizeof(ana.holdRoundFit));
  memset(ana.holdRoundTrueFit,0,sizeof(ana.holdRoundTrueFit));
  memset(ana.holdRoundMargin,0,sizeof(ana.holdRoundMargin));

  uint8_t stateHits[ANA_COMBOS] = {};
  uint8_t ctxSpeedHits[ANA_COMBOS] = {};
  uint8_t ctxModeHits[ANA_COMBOS] = {};
  uint8_t ctxOscHits[ANA_COMBOS] = {};

  int oldThr[5];
  for(int ch=0;ch<5;ch++) {
    oldThr[ch]=fbThresholds[ch];
    fbThresholds[ch]=ana.bestThr[ch];
  }

  auto measureOne=[&](uint8_t c,uint8_t round,uint8_t seq,bool adaptive)->bool {
    int predecessor=-1;
    if(fanIsOn && state.speed>=1 && state.speed<=3 && state.mode<=2)
      predecessor=anaIndex((uint8_t)(state.speed-1),state.mode,state.osc?1:0);
    ana.holdRoundPredecessor[c][round]=(int8_t)predecessor;

    if(!adaptive) {
      anaSetAction("Holdout R%u/%u · Ziel %u/%u: St.%u / %s / Drehen %s · fahre Zustand an",
                   (unsigned)(round+1),(unsigned)ANA_HOLDOUT_BASE_ROUNDS,
                   (unsigned)(seq+1),(unsigned)ANA_COMBOS,
                   anaSpeedOf(c),
                   anaModeOf(c)==0?"Normal":(anaModeOf(c)==1?"Breeze":"Nacht"),
                   anaOscOf(c)?"AN":"AUS");
    } else {
      anaSetAction("Holdout Zusatz %u/%u · St.%u / %s / Drehen %s · fahre Zustand an",
                   (unsigned)(seq+1),(unsigned)ana.holdAdaptiveStates,
                   anaSpeedOf(c),
                   anaModeOf(c)==0?"Normal":(anaModeOf(c)==1?"Breeze":"Nacht"),
                   anaOscOf(c)?"AN":"AUS");
    }

    if(!anaMoveToCombo(c))return false;

    if(!adaptive) {
      anaSetAction("Holdout R%u/%u · Zustand erreicht: St.%u / %s / Drehen %s · MESSE",
                   (unsigned)(round+1),(unsigned)ANA_HOLDOUT_BASE_ROUNDS,
                   anaSpeedOf(c),
                   anaModeOf(c)==0?"Normal":(anaModeOf(c)==1?"Breeze":"Nacht"),
                   anaOscOf(c)?"AN":"AUS");
    } else {
      anaSetAction("Holdout Zusatz %u/%u · Zustand erreicht: St.%u / %s / Drehen %s · MESSE",
                   (unsigned)(seq+1),(unsigned)ana.holdAdaptiveStates,
                   anaSpeedOf(c),
                   anaModeOf(c)==0?"Normal":(anaModeOf(c)==1?"Breeze":"Nacht"),
                   anaOscOf(c)?"AN":"AUS");
    }

    FbStats cur[5];
    if(!anaSampleProfileAverageResponsive(
          cur,anaWindowsFor(anaModeOf(c),anaOscOf(c)!=0),
          0xB01D0000u ^ ((uint32_t)c<<8) ^ round))return false;

    for(int ch=0;ch<5;ch++) {
      ana.holdMeanSum[c][ch]+=cur[ch].avg;
      ana.holdRoundMean[c][round][ch]=(float)cur[ch].avg;
      for(int q=0;q<RICH_CDF_LEVELS;q++) {
        ana.holdCdfSum[c][ch][q]+=cur[ch].cdfPct[q];
        ana.holdRoundCdf[c][round][ch][q]=cur[ch].cdfPct[q];
      }
    }
    if(ana.holdObs[c]<255)ana.holdObs[c]++;

    float margin=0,fit=0;
    int got=anaClassifyStats(cur,&margin,&fit);

    float tr[5][RICH_CDF_LEVELS],tm[5];
    for(int ch=0;ch<5;ch++)
      for(int q=0;q<RICH_CDF_LEVELS;q++)tr[ch][q]=cur[ch].cdfPct[q];
    normalizedMeanFeature(cur,tm);
    float tl=rawLevelFeature(cur);

    int ctxSpeed=anaRichContextWinner(tr,tm,tl,c,0,
                                      ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
    int ctxMode=anaRichContextWinner(tr,tm,tl,c,1,
                                     ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
    int ctxOsc=anaRichContextWinner(tr,tm,tl,c,2,
                                    ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);

    float trueRich[5][RICH_CDF_LEVELS],trueRaw[5],trueMean[5];
    anaRichProfileMean(c,trueRich);
    anaProfileRawMean(c,trueRaw);
    normalizedMeanFeature(trueRaw,trueMean);
    float trueLevel=rawLevelFeature(trueRaw);
    float trueD=anaRichDistance(tr,tm,tl,trueRich,trueMean,trueLevel,
                                ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
    float trueRoot=sqrtf(max(trueD,0.0f));
    float lim=max(ana.fitLimit,0.25f);
    float trueFit=trueRoot<=lim?100.0f:
      constrain(1.0f-(trueRoot-lim)/lim,0.0f,1.0f)*100.0f;

    ana.holdRoundGlobalPred[c][round]=(int8_t)got;
    ana.holdRoundSpeedPred[c][round]=(int8_t)ctxSpeed;
    ana.holdRoundModePred[c][round]=(int8_t)ctxMode;
    ana.holdRoundOscPred[c][round]=(int8_t)ctxOsc;
    ana.holdRoundFit[c][round]=fit;
    ana.holdRoundTrueFit[c][round]=trueFit;
    ana.holdRoundMargin[c][round]=margin;

    ana.holdTotal++;
    if(got==c && fit>=50.0f) {
      ana.holdCorrect++;
      stateHits[c]++;
      if(ana.holdGlobalHits[c]<255)ana.holdGlobalHits[c]++;
    }

    if(got>=0 && anaOscOf(got)==anaOscOf(c) && fit>=50.0f)
      ana.holdOscCorrect++;

    if(trueFit>=50.0f && ctxSpeed==anaSpeedOf(c)-1) {
      ana.holdSpeedContextCorrect++;ctxSpeedHits[c]++;
      if(ana.holdSpeedHits[c]<255)ana.holdSpeedHits[c]++;
    }
    if(trueFit>=50.0f && ctxMode==anaModeOf(c)) {
      ana.holdModeContextCorrect++;ctxModeHits[c]++;
      if(ana.holdModeHits[c]<255)ana.holdModeHits[c]++;
    }
    if(trueFit>=50.0f && ctxOsc==anaOscOf(c)) {
      ana.holdOscContextCorrect++;ctxOscHits[c]++;
      if(ana.holdOscHits[c]<255)ana.holdOscHits[c]++;
    }

    ana.holdWorstMargin=min(ana.holdWorstMargin,margin);
    return anaResponsiveDelay(120);
  };

  // 1) Zwei komplette Paesse. Jeder Zustand hat dabei zwei unterschiedliche
  // Vorgaenger. 2/2 ist sofort ausreichend und braucht keine dritte Messung.
  for(uint8_t round=0;round<ANA_HOLDOUT_BASE_ROUNDS;round++) {
    for(uint8_t seq=0;seq<ANA_COMBOS;seq++) {
      uint8_t c=anaHoldoutCombo(round,seq);
      if(!measureOne(c,round,seq,false)) {
        for(int ch=0;ch<5;ch++)fbThresholds[ch]=oldThr[ch];
        return false;
      }
    }
  }

  // 2) Nur echte 1/2-Grenzfaelle bekommen genau eine dritte Chance.
  // 0/2 kann mathematisch nicht mehr auf die geforderten 2 Treffer kommen.
  bool needThird[ANA_COMBOS] = {};
  for(int c=0;c<ANA_COMBOS;c++) {
    uint8_t mn=min(ctxSpeedHits[c],min(ctxModeHits[c],ctxOscHits[c]));
    if(mn==0) {
      ana.holdImpossibleStates++;
    } else if(mn<2) {
      needThird[c]=true;
      ana.holdAdaptiveStates++;
    }
  }

  if(ana.holdImpossibleStates==0 &&
     ana.holdAdaptiveStates>0 &&
     ana.holdAdaptiveStates<=ANA_HOLDOUT_MAX_ADAPTIVE_STATES) {
    uint8_t doneAdaptive=0;
    for(uint8_t seq=0;seq<ANA_COMBOS;seq++) {
      uint8_t c=anaHoldoutCombo(2,seq);
      if(!needThird[c])continue;

      uint8_t prime=ANA_HOLDOUT_ADAPTIVE_PRIME[c];
      anaSetAction("Holdout Zusatz %u/%u · bereite unabhaengigen Vorgaenger St.%u / %s / Drehen %s",
                   (unsigned)(doneAdaptive+1),(unsigned)ana.holdAdaptiveStates,
                   anaSpeedOf(prime),
                   anaModeOf(prime)==0?"Normal":(anaModeOf(prime)==1?"Breeze":"Nacht"),
                   anaOscOf(prime)?"AN":"AUS");
      if(!anaMoveToCombo(prime)) {
        for(int ch=0;ch<5;ch++)fbThresholds[ch]=oldThr[ch];
        return false;
      }

      if(!measureOne(c,2,doneAdaptive,true)) {
        for(int ch=0;ch<5;ch++)fbThresholds[ch]=oldThr[ch];
        return false;
      }
      doneAdaptive++;
    }
  }

  // 3) State-level Gate: zwei unabhaengige Kontexttreffer pro Zustand.
  for(int c=0;c<ANA_COMBOS;c++) {
    if(stateHits[c]<2)ana.holdStateFailures++;
    if(ctxSpeedHits[c]<2 || ctxModeHits[c]<2 || ctxOscHits[c]<2)
      ana.holdContextStateFailures++;
  }

  for(int ch=0;ch<5;ch++)fbThresholds[ch]=oldThr[ch];
  return true;
}

void analyzeStep() {
  if(!ana.running)return;
  if(ana.stopRequested){analyzeAbort("vom Benutzer gestoppt");return;}
  if((long)(millis()-ana.nextStepMs)<0)return;

  const char* MN[3]={"Normal","Breeze","Nacht"};

  // ---------------- nichtblockierende Startvorbereitung ----------------
  // Dieser Teil laeuft NACH der HTTP-Antwort von /api/analyze/start.
  if(ana.preparing) {
    if(ana.prepStage==0) {
      // Bei Resume wurde vorher ein kompletter Netz-Power-On verlangt.
      // Damit ist der Ventilator physisch AUS und der naechste Power-EIN
      // startet garantiert bei Stufe1 / Normal / Drehen AUS.
      if(ana.resuming || ana.wasOff) {
        anaSetAction(ana.resuming?
                     "Resume: bestaetige physischen AUS-Zustand":
                     "Vorbereitung: bestaetige physischen AUS-Zustand");
        FbStats pre[5];
        sampleProfileAverage(pre,4,180);

        if(!ledsLookOff(pre)) {
          analyzeAbort(ana.resuming?
                       "Resume-Basis nicht AUS - erneut Netz AUS/EIN":
                       "Software sagt AUS, LEDs zeigen aber laufenden Ventilator");
          return;
        }

        anaSetAction(ana.resuming?
                     "Resume: Ventilator EIN · sichere Basis · 4,2 s":
                     "Vorbereitung: Ventilator EIN · 4,2 s Beruhigung");
        togglePower();
        ana.physicalStateTouched=true;
        ana.prepStage=1;
        ana.nextStepMs=millis()+4200UL;
        return;
      }

      ana.prepStage=1;
    }

    if(!fanIsOn) {
      analyzeAbort("Vorbereitung: Ventilator unerwartet AUS");
      return;
    }

    // Absolute Befehlsbasis Stufe1 / Normal / Drehen AUS herstellen.
    // Reihenfolge: Drehen AUS -> Modus Normal -> Stufe1.
    if(state.osc) {
      anaSetAction("Vorbereitung: Drehen -> AUS · 4,5 s");
      ana.physicalStateTouched=true;
      toggleOsc();
      ana.nextStepMs=millis()+ANA_SETTLE_OSC_OFF_MS;
      return;
    }

    if(state.mode!=0) {
      anaSetAction("Vorbereitung: Modus -> Normal · 3,8 s");
      ana.physicalStateTouched=true;
      setMode(0);
      ana.nextStepMs=millis()+ANA_SETTLE_MODE_MS;
      return;
    }

    if(state.speed!=1) {
      anaSetAction("Vorbereitung: Stufe -> 1 · 2,6 s");
      ana.physicalStateTouched=true;
      setSpeed(1);
      ana.nextStepMs=millis()+ANA_SETTLE_SPEED_MS;
      return;
    }

    state.speed=1;
    state.mode=0;
    state.osc=false;
    ledDetectReadyMs=0;
    resetStableDiagnostic();

    bool wasResume=ana.resuming;
    ana.preparing=false;
    ana.resuming=false;
    ana.prepStage=0;
    anaSetAction(wasResume?
                 "Resume-Basis bereit - Messlauf wird am Checkpoint fortgesetzt":
                 "Basis bereit: Stufe 1 / Normal / Drehen AUS");
    ana.nextStepMs=millis()+350UL;
    return;
  }

  // V6: jeder Repeat benutzt eine optimierte andere Permutation UND damit andere
  // Vorgaenger-/Umschaltpfade. Zeitposition und Transition-Historie variieren.
  uint8_t measureCombo=anaTrainingCombo(ana.repeat,ana.combo);
  if(ana.combo==0 && ana.repeat<ANA_REPEATS)
    ana.trainBootId[ana.repeat]=bootSessionId;

  uint8_t sp=anaSpeedOf(measureCombo),md=anaModeOf(measureCombo),os=anaOscOf(measureCombo);

  // V6: Training faehrt EXAKT wie spaetere Web-Kommandos direkt zum Ziel.
  // Speed wird direkt geschaltet, ohne vorher den Modus auf Normal zu setzen;
  // Drehen bleibt bei Speed-/Mode-Aenderungen aktiv, wenn das Ziel ebenfalls
  // Drehen AN ist. Damit stimmen Trainingshistorie und reale Nutzung ueberein.
  if(state.speed!=sp) {
    anaSetAction("Stufe %u -> %u · direkter Web-Pfad",state.speed,sp);
    ana.physicalStateTouched=true;
    setSpeed(sp);ana.nextStepMs=millis()+ANA_SETTLE_SPEED_MS;return;
  }
  if(state.mode!=md) {
    anaSetAction("Modus -> %s · direkter Web-Pfad",MN[md]);
    ana.physicalStateTouched=true;
    setMode(md);ana.nextStepMs=millis()+ANA_SETTLE_MODE_MS;return;
  }
  if((state.osc?1:0)!=os) {
    anaSetAction("Drehen -> %s (%s)",os?"AN":"AUS",os?"8 s Anlaufzeit":"4,5 s Beruhigung");
    ana.physicalStateTouched=true;
    toggleOsc();ana.nextStepMs=millis()+(os?ANA_SETTLE_OSC_ON_MS:ANA_SETTLE_OSC_OFF_MS);return;
  }

  uint16_t nr=ana.repeat*ANA_COMBOS+ana.combo+1;
  anaSetAction("Messe %u/%u: St.%u / %s / Drehen %s",
               nr,ANA_COMBOS*ANA_REPEATS,sp,MN[md],os?"AN":"AUS");

  uint8_t windows=anaWindowsFor(md,os!=0);
  float am[5]={0,0,0,0,0};

  uint32_t seriesRng=profileNewSeed(0x53455249u ^ ((uint32_t)measureCombo<<8) ^ ana.repeat);
  for(int w=0;w<windows;w++) {
    float mw[5];
    anaSampleComboHist(measureCombo,ana.repeat,mw);
    for(int ch=0;ch<5;ch++)am[ch]+=mw[ch];

    if(w+1<windows) {
      if(!anaResponsiveDelay(profileSeriesGapMs(seriesRng))) {
        analyzeAbort("vom Benutzer gestoppt");
        return;
      }
    } else {
      anaServiceIo();
    }
  }

  for(int ch=0;ch<5;ch++) {
    ana.rawmean[measureCombo][ch][ana.repeat]=am[ch]/windows;
    uint8_t t=constrain(fbThresholds[ch]>>ANA_BIN_SHIFT,1,ANA_BINS-1);
    ana.samples[measureCombo][ch][ana.repeat]=anaAbovePct(measureCombo,ch,ana.repeat,t);
  }

  ana.combo++;
  if(ana.combo>=ANA_COMBOS) {
    ana.combo=0;
    ana.repeat++;
  }

  // Recovery-Punkt immer NACH einer vollstaendig abgeschlossenen Messung.
  // Alle 3 Profile wird gesichert. Beim Resume wird ein angebrochener
  // 18-Zustands-Repeat komplett neu aufgenommen, damit kein Repeat aus
  // zwei unterschiedlichen Boot-Sitzungen zusammengesetzt ist.
  {
    const uint16_t total=(uint16_t)(ANA_COMBOS*ANA_REPEATS);
    uint16_t completed=min(total,(uint16_t)(ana.repeat*ANA_COMBOS+ana.combo));
    bool checkpointPoint=(completed>0 && completed<total) &&
                         ((completed%ANA_CP_EVERY_STEPS)==0);
    if(checkpointPoint) {
      if(!saveAnalysisCheckpoint(false)) {
        Serial.println("[ANA-CP] WARNUNG: Messlauf laeuft weiter, letzter Checkpoint konnte nicht aktualisiert werden");
      }
      if(ana.stopRequested){analyzeAbort("vom Benutzer gestoppt");return;}
    }
  }

  if(ana.repeat>=ANA_REPEATS) {
      snprintf(anaCheckpointStatus,sizeof(anaCheckpointStatus),
               "EFFICIENT Training %u/%u · Checkpoint %u/%u · bei Reset wird nur der letzte 18er-Repeat neu gemessen",
               (unsigned)(ANA_COMBOS*ANA_REPEATS),
               (unsigned)(ANA_COMBOS*ANA_REPEATS),
               (unsigned)anaCheckpointSavedSteps,
               (unsigned)(ANA_COMBOS*ANA_REPEATS));

      // 1) AUS-Profil mit derselben JITTER_V6-Verteilung aufnehmen.
      if(state.osc) {
        ana.physicalStateTouched=true;
        toggleOsc();
        if(!anaResponsiveDelay(ANA_SETTLE_OSC_OFF_MS)) {
          analyzeAbort("waehrend AUS-Vorbereitung gestoppt");
          return;
        }
      }

      anaSetAction("Messe AUS-Profil phasenunabhaengig");
      ana.physicalStateTouched=true;
      powerOff();
      if(!anaResponsiveDelay(3200)) {
        analyzeAbort("waehrend AUS-Messung gestoppt");
        return;
      }

      memset(ana.offPct,0,sizeof(ana.offPct));
      memset(ana.offMean,0,sizeof(ana.offMean));

      for(int w=0;w<6;w++) {
        float mw[5];
        anaSampleOffHist(mw);
        for(int ch=0;ch<5;ch++)ana.offMean[ch]+=mw[ch]/6.0f;

        if(!anaResponsiveDelay(70+(w*11))) {
          analyzeAbort("waehrend AUS-Messung gestoppt");
          return;
        }
      }
      ana.offMeasured=true;

      // 2) Rich-Modell optimieren: Prozent-CDF + relativer ADC-Mittelwert.
      anaSetAction("Optimiere FINAL-RICH-CDF: Schwellen, Rich-Features und Cross-Validation");
      anaOptimize();
      if(ana.stopRequested || !ana.optDone) {
        analyzeAbort("Optimierung gestoppt");
        return;
      }

      // 3) AUS-Profil noch einmal mit neuen Jitter-Phasen pruefen. Damit ist
      //    auch der physische AUS-Zustand ein echter Holdout und nicht nur Teil
      //    desselben Trainingsblocks.
      anaSetAction("AUS-Holdout: pruefe Profil und absoluten ADC-Pegel");
      anaVerifyOffCurrent();

      // 4) Zweiter, unabhaengiger kompletter 18P-Durchlauf.
      //    Das ist bewusst KEIN Teil des Trainingsdatensatzes.
      anaSetAction("Holdout: Ventilator EIN");
      ana.physicalStateTouched=true;
      togglePower();
      if(!anaResponsiveDelay(4200)) {
        analyzeAbort("vor Holdout gestoppt");
        return;
      }

      String baseErr;
      if(!anaNormalizePhysicalBaseline(baseErr)) {
        analyzeAbort("Basis fuer Holdout konnte nicht hergestellt werden");
        return;
      }

      anaSetAction("Holdout: 2 Pflichtpaesse + nur bei 1/2 adaptive Zusatzmessung");
      if(!anaValidateAll18()) {
        analyzeAbort("Holdout gestoppt");
        return;
      }

      // 5) Zusaetzliche Mehrfachpruefung des historisch kritischen
      //    Stufe1/Normal-Drehen-Paars.
      if(!anaMoveToCombo(anaIndex(0,0,0))) {
        analyzeAbort("vor Schlusspruefung gestoppt");
        return;
      }

      anaSetAction("Schlusspruefung: Stufe 1 / Normal / Drehen AUS");
      anaVerifyCurrent(anaIndex(0,0,0),ANA_VERIFY_WINDOWS_OFF,
                       ana.verifyOffPct,ana.verifyOff,ana.verifyOffHits);

      if(!anaMoveToCombo(anaIndex(0,0,1))) {
        analyzeAbort("vor Drehen-AN-Schlusspruefung gestoppt");
        return;
      }

      anaSetAction("Schlusspruefung: Stufe 1 / Normal / Drehen AN");
      anaVerifyCurrent(anaIndex(0,0,1),ANA_VERIFY_WINDOWS_ON,
                       ana.verifyOnPct,ana.verifyOn,ana.verifyOnHits);
      ana.verifyDone=true;

      // 6) Nutzerzustand reproduzierbar wiederherstellen.
      if(!anaRestoreUserState()) {
        analyzeAbort("Messung fertig, Rueckfahrt aber nicht vollstaendig");
        return;
      }

      suppressFanStateSave=false;
      bumpStateRevision();
      saveFanState();

      if(fanIsOn)armLedDiagnostic(state.osc?8000UL:LED_DETECT_SETTLE_MS);
      else {ledDetectReadyMs=0;resetStableDiagnostic();}

      ana.preparing=false;
      ana.running=false;
      ana.done=true;
      anaSetAction("Fertig - Profil kann manuell uebernommen werden");
      return;
  }

  ana.nextStepMs=millis()+250;
}
#else
void analyzeStep(){}
#endif

bool analyzeApply(String& err) {
#if !ENABLE_LED_FEEDBACK
  err="LED-Feedback deaktiviert";
  return false;
#else
  if(ana.running){err="Messlauf laeuft noch";return false;}
  if(!ana.done){err="kein abgeschlossener Messlauf";return false;}
  if(!ana.optDone){err="Optimierung fehlt";return false;}

  // Ab 6.4.6 blockieren Messdifferenzen die manuelle Uebernahme nicht mehr.
  // Ein abgeschlossener Lauf darf bewusst gespeichert werden. Webbefehle sind
  // autoritativ; die LED-Auswertung bleibt Plausibilitaet/Diagnose.
  if(!ana.offVerifyDone || !ana.verifyDone || !ana.offMeasured) {
    err="Messlauf technisch unvollstaendig";
    return false;
  }

  // Blindes 18P ist NICHT Voraussetzung fuer das normale Tracking.
  // Ist es physikalisch nicht eindeutig genug, darf das kontextuelle Modell
  // trotzdem FINAL werden; nur der manuelle 3/3-LED-Sync bleibt gesperrt.
  bool globalSyncSafe =
     ((uint32_t)ana.cvCorrect*100u)>=((uint32_t)ana.cvTotal*90u) &&
     ((uint32_t)ana.holdCorrect*100u)>=((uint32_t)ana.holdTotal*85u) &&
     ana.holdStateFailures<=3;

  suppressCalibrationSave=true;

  for(int ch=0;ch<5;ch++)
    fbThresholds[ch]=ana.bestThr[ch];

  uint8_t tb[5];
  for(int ch=0;ch<5;ch++)
    tb[ch]=constrain(fbThresholds[ch]>>ANA_BIN_SHIFT,1,ANA_BINS-1);

  // Fallback-Speedprofile: ueber Modus UND Drehen mitteln.
  for(int sp=1;sp<=3;sp++) {
    for(int ch=0;ch<5;ch++) {
      double p=0,m=0;
      int n=0;

      for(int c=0;c<ANA_COMBOS;c++)if(anaSpeedOf(c)==sp)
        for(int r=0;r<ANA_REPEATS;r++) {
          p+=anaAbovePct(c,ch,r,tb[ch]);
          m+=ana.rawmean[c][ch][r];
          n++;
        }

      speedSig[sp-1].avgPct[ch]=n?p/n:0;
      speedSig[sp-1].avgMean[ch]=n?m/n:0;
    }
    speedSig[sp-1].valid=true;
  }

  // Fallback-Modusprofile: ueber Speed UND Drehen mitteln.
  for(int md=0;md<3;md++) {
    for(int ch=0;ch<5;ch++) {
      double p=0,m=0;
      int n=0;

      for(int c=0;c<ANA_COMBOS;c++)if(anaModeOf(c)==md)
        for(int r=0;r<ANA_REPEATS;r++) {
          p+=anaAbovePct(c,ch,r,tb[ch]);
          m+=ana.rawmean[c][ch][r];
          n++;
        }

      modeSig[md].avgPct[ch]=n?p/n:0;
      modeSig[md].avgMean[ch]=n?m/n:0;
    }
    modeSig[md].valid=true;
  }

  // 9P-Fallback: bewusst Drehen AUS.
  for(int sp=0;sp<3;sp++) {
    for(int md=0;md<3;md++) {
      int c=anaIndex(sp,md,0);

      for(int ch=0;ch<5;ch++) {
        double p=0,m=0;
        for(int r=0;r<ANA_REPEATS;r++) {
          p+=anaAbovePct(c,ch,r,tb[ch]);
          m+=ana.rawmean[c][ch][r];
        }
        comboSig[sp][md].avgPct[ch]=p/ANA_REPEATS;
        comboSig[sp][md].avgMean[ch]=m/ANA_REPEATS;
      }
      comboSig[sp][md].valid=true;
    }
  }

  // Finale 18 Profile.
  //
  // Wichtig: Der Holdout bleibt bis HIER vollstaendig unabhaengig und
  // entscheidet, ob das Modell uebernommen werden darf. Erst NACH bestandenem
  // Gate bleibt der Holdout reine Generalisierungsdiagnose. Der spaetere
  // Cross-Boot-Praxistest zeigte, dass ein Holdout aus derselben Boot-Sitzung
  // nicht in die Zentren hineingemischt werden sollte. Gespeichert werden daher
  // die vier transition-balancierten Trainingszentren; der Holdout bewertet sie
  // nur und veraendert sie nicht.
  constexpr float HOLDOUT_BLEND_EQUIV=0.0f;

  for(int c=0;c<ANA_COMBOS;c++) {
    int sp=anaSpeedOf(c)-1;
    int md=anaModeOf(c);
    int os=anaOscOf(c);
    float hn=(float)ana.holdObs[c];
    float denom=(float)ANA_REPEATS+(hn>0.0f?HOLDOUT_BLEND_EQUIV:0.0f);

    for(int ch=0;ch<5;ch++) {
      double p=0,m=0;
      for(int r=0;r<ANA_REPEATS;r++) {
        p+=anaAbovePct(c,ch,r,tb[ch]);
        m+=ana.rawmean[c][ch][r];
      }

      comboOscSig[sp][md][os].avgPct[ch]=p/ANA_REPEATS;

      if(hn>0.0f) {
        float holdMean=ana.holdMeanSum[c][ch]/hn;
        comboOscSig[sp][md][os].avgMean[ch]=
          (m+holdMean*HOLDOUT_BLEND_EQUIV)/denom;
      } else {
        comboOscSig[sp][md][os].avgMean[ch]=m/ANA_REPEATS;
      }
    }
    comboOscSig[sp][md][os].valid=true;

    for(int ch=0;ch<5;ch++)
      for(int q=0;q<RICH_CDF_LEVELS;q++) {
        float train=0.0f;
        for(int r=0;r<ANA_REPEATS;r++)train+=anaRichPct(c,ch,r,q);

        float finalPct=train/ANA_REPEATS;
        if(hn>0.0f) {
          float holdPct=ana.holdCdfSum[c][ch][q]/hn;
          finalPct=(train+holdPct*HOLDOUT_BLEND_EQUIV)/denom;
        }
        comboRichPctQ[sp][md][os][ch][q]=richEncodePct(finalPct);
      }
  }

  // AUS-Profil.
  if(ana.offMeasured) {
    for(int ch=0;ch<5;ch++) {
      offSig.avgPct[ch]=anaOffPctAt(ch,tb[ch]);
      offSig.avgMean[ch]=ana.offMean[ch];
    }
    offSig.valid=true;
  }

  for(int ch=0;ch<5;ch++) {
    comboW[ch]=ana.weight[ch];
    comboMeanW[ch]=ana.meanWeight[ch];
    for(int q=0;q<RICH_CDF_LEVELS;q++)comboRichW[ch][q]=ana.richWeight[ch][q];
  }
  comboWValid=true;
  comboRichValid=true;
  comboGlobalSyncSafe=globalSyncSafe;
  comboShape=ana.shapeBest;
  comboMeanAlpha=constrain(ana.meanAlphaBest,0.0f,16.0f);
  comboLevelAlpha=constrain(ana.levelAlphaBest,0.0f,4.0f);
  comboFitLimit=max(ana.fitLimit,0.25f);

  // Letzte Sicherheitspruefung OHNE neue Ventilatormessung:
  // Die exakt gespeicherten Holdout-Messungen werden noch einmal gegen das
  // TATSAECHLICH zu speichernde, bereits mit Holdout geblendete Profil
  // abgespielt. Damit kann die Uebernahme nicht dadurch schlechter werden,
  // dass der Post-Validation-Blend die Zentren unguenstig verschiebt.
  uint16_t replayTotal=0,replaySpeed=0,replayMode=0,replayOsc=0;
  uint8_t replaySpeedHits[ANA_COMBOS] = {};
  uint8_t replayModeHits[ANA_COMBOS] = {};
  uint8_t replayOscHits[ANA_COMBOS] = {};
  uint8_t replayStateFailures=0;

  for(int c=0;c<ANA_COMBOS;c++) {
    int tsp=anaSpeedOf(c)-1,tmd=anaModeOf(c),tos=anaOscOf(c);

    for(int r=0;r<ANA_HOLDOUT_ROUNDS;r++) {
      if(ana.holdRoundPredecessor[c][r]<0)continue;

      FbStats cur[5] = {};
      for(int ch=0;ch<5;ch++) {
        cur[ch].avg=(int)lroundf(ana.holdRoundMean[c][r][ch]);
        for(int q=0;q<RICH_CDF_LEVELS;q++)
          cur[ch].cdfPct[q]=ana.holdRoundCdf[c][r][ch][q];
      }

      float dTrue=comboRichDistanceToState(tsp,tmd,tos,cur);
      float rr=sqrtf(max(dTrue,0.0f));
      float lim=max(comboFitLimit,0.25f);
      float trueFit=rr<=lim?100.0f:
        constrain(1.0f-(rr-lim)/lim,0.0f,1.0f)*100.0f;

      float best=1e30f;int got=-1;
      for(int s=0;s<3;s++) {
        float d=comboRichDistanceToState(s,tmd,tos,cur);
        if(d<best){best=d;got=s;}
      }
      if(trueFit>=50.0f && got==tsp){replaySpeed++;replaySpeedHits[c]++;}

      best=1e30f;got=-1;
      for(int m=0;m<3;m++) {
        float d=comboRichDistanceToState(tsp,m,tos,cur);
        if(d<best){best=d;got=m;}
      }
      if(trueFit>=50.0f && got==tmd){replayMode++;replayModeHits[c]++;}

      best=1e30f;got=-1;
      for(int o=0;o<2;o++) {
        float d=comboRichDistanceToState(tsp,tmd,o,cur);
        if(d<best){best=d;got=o;}
      }
      if(trueFit>=50.0f && got==tos){replayOsc++;replayOscHits[c]++;}

      replayTotal++;
    }
  }

  for(int c=0;c<ANA_COMBOS;c++) {
    if(replaySpeedHits[c]<2 || replayModeHits[c]<2 || replayOscHits[c]<2)
      replayStateFailures++;
  }

  bool replayStrong =
     replayTotal>=ANA_HOLDOUT_BASE_TOTAL &&
     ((uint32_t)replaySpeed*100u)>=((uint32_t)replayTotal*90u) &&
     ((uint32_t)replayMode*100u)>=((uint32_t)replayTotal*90u) &&
     ((uint32_t)replayOsc*100u)>=((uint32_t)replayTotal*90u) &&
     replayStateFailures==0;

  if(!replayStrong) {
    Serial.printf("[ANA] Profil-Replay mit Abweichungen: Speed %u/%u Mode %u/%u Osc %u/%u StateFail %u - Uebernahme bewusst erlaubt\n",
                  (unsigned)replaySpeed,(unsigned)replayTotal,
                  (unsigned)replayMode,(unsigned)replayTotal,
                  (unsigned)replayOsc,(unsigned)replayTotal,
                  (unsigned)replayStateFailures);
  }

  calibrationSamplerCurrent=true;
  suppressCalibrationSave=false;

  if(!saveCompactComboCalibration(true)) {
    comboRichValid=false;
    comboWValid=false;
    calibrationSamplerCurrent=false;
    (void)loadCompactComboCalibration();
    err="FINAL-Modell konnte nicht CRC-verifiziert im A/B-NVS gespeichert werden";
    return false;
  }

  resetStableDiagnostic();
  if(fanIsOn)
    armLedDiagnostic(state.osc?8000UL:LED_DETECT_SETTLE_MS);

  clearAnalysisCheckpoint();
  return true;
#endif
}


void anaOptimize() {
  // ------------------------------------------------------------
  // 1) Legacy/UI-Einzelschwellwerte weiterhin robust bestimmen.
  //    Das FINAL-Modell selbst nutzt danach alle 50 CDF-Merkmale.
  // ------------------------------------------------------------
  for(int ch=0;ch<5;ch++) {
    bool modeCh=ch>=3;
    float best=-1;
    int bestT=-1;

    for(int t=1;t<ANA_BINS;t++) {
      if((t&7)==0) {
        anaServiceIo();
        if(ana.stopRequested){ana.optDone=false;return;}
      }

      float m[ANA_COMBOS],sd[ANA_COMBOS],lo=1e9f,hi=-1e9f;
      for(int c=0;c<ANA_COMBOS;c++) {
        anaMeanSd(c,ch,t,m[c],sd[c]);
        lo=min(lo,m[c]);hi=max(hi,m[c]);
      }

      float sSpeed=1e9f,sMode=1e9f,sOsc=1e9f;
      for(int md=0;md<3;md++)for(int os=0;os<2;os++)
        for(int a=0;a<3;a++)for(int b=a+1;b<3;b++) {
          int ca=anaIndex(a,md,os),cb=anaIndex(b,md,os);
          sSpeed=min(sSpeed,fabsf(m[ca]-m[cb])/max(max(sd[ca],sd[cb]),0.15f));
        }
      for(int sp=0;sp<3;sp++)for(int os=0;os<2;os++)
        for(int a=0;a<3;a++)for(int b=a+1;b<3;b++) {
          int ca=anaIndex(sp,a,os),cb=anaIndex(sp,b,os);
          sMode=min(sMode,fabsf(m[ca]-m[cb])/max(max(sd[ca],sd[cb]),0.15f));
        }
      for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++) {
        int ca=anaIndex(sp,md,0),cb=anaIndex(sp,md,1);
        sOsc=min(sOsc,fabsf(m[ca]-m[cb])/max(max(sd[ca],sd[cb]),0.15f));
      }

      float sOff=0.0f;
      if(ana.offMeasured) {
        sOff=1e9f;
        float mo=anaOffPctAt(ch,t);
        for(int c=0;c<ANA_COMBOS;c++)sOff=min(sOff,fabsf(m[c]-mo)/max(sd[c],0.15f));
      }

      float mid=(lo+hi)*0.5f;
      float saturationPenalty=(mid>94||mid<6)?0.20f:((mid>90||mid<10)?0.65f:1.0f);
      float wS=modeCh?0.50f:1.0f,wM=modeCh?1.0f:0.50f,wO=0.90f;
      float score=(wS*min(sSpeed,5.0f)+wM*min(sMode,5.0f)+
                   wO*min(sOsc,5.0f)+0.35f*min(sOff,5.0f))*saturationPenalty;

      if(score>best) {
        best=score;bestT=t;
        ana.scoreSpeed[ch]=sSpeed;ana.scoreMode[ch]=sMode;
        ana.scoreOsc[ch]=sOsc;ana.scoreOff[ch]=sOff;
        ana.rangeLo[ch]=lo;ana.rangeHi[ch]=hi;
      }
    }
    ana.bestThr[ch]=bestT<0?FB_DEFAULT_THR[ch]:(bestT<<ANA_BIN_SHIFT);
  }

  // ------------------------------------------------------------
  // 2) 50 CDF-Feature-Gewichte.
  //    Score kombiniert Speed-, Mode- und Drehen-Trennung ueber ALLE
  //    relevanten Paare. So darf z.B. ein High-Tail-Feature gezielt Speed
  //    tragen, waehrend ein anderer Pegel Drehen/common-mode traegt.
  // ------------------------------------------------------------
  float richScore[5][RICH_CDF_LEVELS] = {};
  float totalScore=0.0f;

  for(int ch=0;ch<5;ch++) {
    for(int q=0;q<RICH_CDF_LEVELS;q++) {
      float means[ANA_COMBOS],sds[ANA_COMBOS];

      for(int c=0;c<ANA_COMBOS;c++) {
        float m=0.0f;
        for(int r=0;r<ANA_REPEATS;r++)m+=anaRichPct(c,ch,r,q);
        m/=ANA_REPEATS;
        means[c]=m;

        float var=0.0f;
        for(int r=0;r<ANA_REPEATS;r++) {
          float e=anaRichPct(c,ch,r,q)-m;
          var+=e*e;
        }
        sds[c]=sqrtf(var/max(1,(int)ANA_REPEATS-1));
      }

      float sumS=0.0f,sumM=0.0f,sumO=0.0f;
      int nS=0,nM=0,nO=0;

      for(int md=0;md<3;md++)for(int os=0;os<2;os++)
        for(int a=0;a<3;a++)for(int b=a+1;b<3;b++) {
          int ca=anaIndex(a,md,os),cb=anaIndex(b,md,os);
          float z=fabsf(means[ca]-means[cb])/max(max(sds[ca],sds[cb]),0.20f);
          sumS+=min(z,8.0f);nS++;
        }

      for(int sp=0;sp<3;sp++)for(int os=0;os<2;os++)
        for(int a=0;a<3;a++)for(int b=a+1;b<3;b++) {
          int ca=anaIndex(sp,a,os),cb=anaIndex(sp,b,os);
          float z=fabsf(means[ca]-means[cb])/max(max(sds[ca],sds[cb]),0.20f);
          sumM+=min(z,8.0f);nM++;
        }

      for(int sp=0;sp<3;sp++)for(int md=0;md<3;md++) {
        int ca=anaIndex(sp,md,0),cb=anaIndex(sp,md,1);
        float z=fabsf(means[ca]-means[cb])/max(max(sds[ca],sds[cb]),0.20f);
        sumO+=min(z,8.0f);nO++;
      }

      float aS=nS?sumS/nS:0.0f;
      float aM=nM?sumM/nM:0.0f;
      float aO=nO?sumO/nO:0.0f;

      // Drehen bekommt leicht mehr Gewicht, weil genau diese Dimension
      // in v6.3.4 bei St2/St3 am schwaechsten war.
      float score=0.90f*aS + 0.90f*aM + 1.20f*aO + 0.05f;
      richScore[ch][q]=score;
      totalScore+=score;
    }
  }

  if(totalScore<0.001f)totalScore=1.0f;
  float wsum=0.0f;
  for(int ch=0;ch<5;ch++)
    for(int q=0;q<RICH_CDF_LEVELS;q++) {
      ana.richWeight[ch][q]=constrain(richScore[ch][q]*5.0f/totalScore,0.015f,0.35f);
      wsum+=ana.richWeight[ch][q];
    }
  if(wsum>0.001f)
    for(int ch=0;ch<5;ch++)
      for(int q=0;q<RICH_CDF_LEVELS;q++)ana.richWeight[ch][q]*=5.0f/wsum;

  // UI/Legacy-Kanalgewicht als Summe der acht Rich-Gewichte.
  for(int ch=0;ch<5;ch++) {
    ana.weight[ch]=0.0f;
    for(int q=0;q<RICH_CDF_LEVELS;q++)ana.weight[ch]+=ana.richWeight[ch][q];
  }

  // ------------------------------------------------------------
  // 3) Relative ADC-Mittelwert-Gewichte.
  // ------------------------------------------------------------
  float meanRaw[5],meanRawSum=0.0f;
  for(int ch=0;ch<5;ch++) {
    float profileLo=1e9f,profileHi=-1e9f,noise=0.0f;

    for(int c=0;c<ANA_COMBOS;c++) {
      float vals[ANA_REPEATS],m=0.0f;
      for(int r=0;r<ANA_REPEATS;r++) {
        float raw[5],nf[5];
        for(int k=0;k<5;k++)raw[k]=ana.rawmean[c][k][r];
        normalizedMeanFeature(raw,nf);
        vals[r]=nf[ch];m+=vals[r];
      }
      m/=ANA_REPEATS;

      float var=0.0f;
      for(int r=0;r<ANA_REPEATS;r++){float e=vals[r]-m;var+=e*e;}
      float sd=sqrtf(var/max(1,(int)ANA_REPEATS-1));

      profileLo=min(profileLo,m);profileHi=max(profileHi,m);noise+=sd;
    }

    noise/=ANA_COMBOS;
    meanRaw[ch]=(profileHi-profileLo)/max(noise,0.015f);
    meanRawSum+=meanRaw[ch];
  }

  if(meanRawSum<0.001f)meanRawSum=1.0f;
  float meanNormSum=0.0f;
  for(int ch=0;ch<5;ch++) {
    ana.meanWeight[ch]=constrain(meanRaw[ch]*5.0f/meanRawSum,0.20f,2.50f);
    meanNormSum+=ana.meanWeight[ch];
  }
  if(meanNormSum>0.001f)
    for(int ch=0;ch<5;ch++)ana.meanWeight[ch]*=5.0f/meanNormSum;

  // ------------------------------------------------------------
  // 4) Parameteroptimierung ohne 40-Feature-Neuberechnung pro Kandidat.
  //    Zuerst Formanteil, dann gecachte Distanzkomponenten fuer Mean/Level.
  // ------------------------------------------------------------
  const float shapeCand[5]={0.0f,0.25f,0.50f,0.75f,1.0f};
  const float alphaCand[8]={0.0f,1.0f,2.0f,4.0f,8.0f,16.0f,32.0f,64.0f};
  const float levelCand[8]={0.0f,0.5f,1.0f,2.0f,4.0f,8.0f,16.0f,32.0f};

  float chosenShape=0.0f;
  int chosenShapeContext=-1,chosenShapeGlobal=-1;
  float chosenShapeWorst=-1e9f;

  // Form nur mit CDF-Information bestimmen.
  for(float shape:shapeCand) {
    int globalCorrect=0,ctxCorrect=0;
    float worstMargin=100.0f;

    for(int c=0;c<ANA_COMBOS;c++) {
      int tsp=anaSpeedOf(c)-1,tmd=anaModeOf(c),tos=anaOscOf(c);

      for(int r=0;r<ANA_REPEATS;r++) {
        float tr[5][RICH_CDF_LEVELS],tm[5],tl=0.0f;
        anaRichTestFeatures(c,r,tr,tm,tl);

        float best=1e30f,second=1e30f;
        float bS=1e30f,bM=1e30f,bO=1e30f;
        int got=-1,gS=-1,gM=-1,gO=-1;

        for(int k=0;k<ANA_COMBOS;k++) {
          float rr[5][RICH_CDF_LEVELS],rm[5],rl=0.0f;
          anaRichReferenceExcluding(k,c,r,rr,rm,rl);
          float d=anaRichDistance(tr,tm,tl,rr,rm,rl,shape,0.0f,0.0f);

          if(d<best){second=best;best=d;got=k;}
          else if(d<second)second=d;

          int sp=anaSpeedOf(k)-1,md=anaModeOf(k),os=anaOscOf(k);
          if(md==tmd&&os==tos&&d<bS){bS=d;gS=sp;}
          if(sp==tsp&&os==tos&&d<bM){bM=d;gM=md;}
          if(sp==tsp&&md==tmd&&d<bO){bO=d;gO=os;}
        }

        if(got==c)globalCorrect++;
        if(gS==tsp)ctxCorrect++;
        if(gM==tmd)ctxCorrect++;
        if(gO==tos)ctxCorrect++;

        float rb=sqrtf(max(best,0.0f)),rs=sqrtf(max(second,0.0f));
        float margin=(rs>0.0001f)?((rs-rb)/rs*100.0f):0.0f;
        worstMargin=min(worstMargin,margin);
      }
    }

    bool better=(ctxCorrect>chosenShapeContext) ||
                (ctxCorrect==chosenShapeContext&&globalCorrect>chosenShapeGlobal) ||
                (ctxCorrect==chosenShapeContext&&globalCorrect==chosenShapeGlobal&&
                 worstMargin>chosenShapeWorst);
    if(better) {
      chosenShapeContext=ctxCorrect;
      chosenShapeGlobal=globalCorrect;
      chosenShapeWorst=worstMargin;
      chosenShape=shape;
    }

    anaServiceIo();
    if(ana.stopRequested){ana.optDone=false;return;}
  }

  ana.shapeBest=chosenShape;

  // ANA_COMBOS * ANA_REPEATS Tests x 18 Referenzen x drei additive Distanzkomponenten.
  // static -> kein grosser Stackframe; rund 19 kB temporaerer Analyse-BSS.
  static float cvCdfD[ANA_COMBOS*ANA_REPEATS][ANA_COMBOS];
  static float cvMeanD[ANA_COMBOS*ANA_REPEATS][ANA_COMBOS];
  static float cvLevelD[ANA_COMBOS*ANA_REPEATS][ANA_COMBOS];

  for(int c=0;c<ANA_COMBOS;c++) {
    for(int r=0;r<ANA_REPEATS;r++) {
      int si=c*ANA_REPEATS+r;

      float tr[5][RICH_CDF_LEVELS],tm[5],tl=0.0f;
      anaRichTestFeatures(c,r,tr,tm,tl);

      for(int k=0;k<ANA_COMBOS;k++) {
        float rr[5][RICH_CDF_LEVELS],rm[5],rl=0.0f;
        anaRichReferenceExcluding(k,c,r,rr,rm,rl);

        cvCdfD[si][k]=anaRichDistance(tr,tm,tl,rr,rm,rl,
                                      ana.shapeBest,0.0f,0.0f);

        float md=0.0f;
        for(int ch=0;ch<5;ch++) {
          float e=tm[ch]-rm[ch];
          md+=ana.meanWeight[ch]*e*e;
        }
        cvMeanD[si][k]=md;

        float el=tl-rl;
        cvLevelD[si][k]=el*el;
      }
    }
    anaServiceIo();
    if(ana.stopRequested){ana.optDone=false;return;}
  }

  int chosenContext=-1,chosenGlobal=-1;
  float chosenWorst=-1e9f,chosenAvg=-1e9f;
  float chosenAlpha=0.0f,chosenLevel=0.0f;

  for(float alpha:alphaCand) {
    for(float levelAlpha:levelCand) {
      int globalCorrect=0,ctxCorrect=0;
      float worstMargin=100.0f,sumMargin=0.0f;

      for(int c=0;c<ANA_COMBOS;c++) {
        int tsp=anaSpeedOf(c)-1,tmd=anaModeOf(c),tos=anaOscOf(c);

        for(int r=0;r<ANA_REPEATS;r++) {
          int si=c*ANA_REPEATS+r;
          float best=1e30f,second=1e30f;
          float bS=1e30f,bM=1e30f,bO=1e30f;
          int got=-1,gS=-1,gM=-1,gO=-1;

          for(int k=0;k<ANA_COMBOS;k++) {
            float d=cvCdfD[si][k] +
                    alpha*cvMeanD[si][k] +
                    levelAlpha*cvLevelD[si][k];

            if(d<best){second=best;best=d;got=k;}
            else if(d<second)second=d;

            int sp=anaSpeedOf(k)-1,md=anaModeOf(k),os=anaOscOf(k);
            if(md==tmd&&os==tos&&d<bS){bS=d;gS=sp;}
            if(sp==tsp&&os==tos&&d<bM){bM=d;gM=md;}
            if(sp==tsp&&md==tmd&&d<bO){bO=d;gO=os;}
          }

          if(got==c)globalCorrect++;
          if(gS==tsp)ctxCorrect++;
          if(gM==tmd)ctxCorrect++;
          if(gO==tos)ctxCorrect++;

          float rb=sqrtf(max(best,0.0f)),rs=sqrtf(max(second,0.0f));
          float margin=(rs>0.0001f)?((rs-rb)/rs*100.0f):0.0f;
          worstMargin=min(worstMargin,margin);
          sumMargin+=margin;
        }
      }

      float avgMargin=sumMargin/(ANA_COMBOS*ANA_REPEATS);
      bool better=(ctxCorrect>chosenContext) ||
                  (ctxCorrect==chosenContext&&globalCorrect>chosenGlobal) ||
                  (ctxCorrect==chosenContext&&globalCorrect==chosenGlobal&&
                   worstMargin>chosenWorst+0.001f) ||
                  (ctxCorrect==chosenContext&&globalCorrect==chosenGlobal&&
                   fabsf(worstMargin-chosenWorst)<0.001f&&avgMargin>chosenAvg);

      if(better) {
        chosenContext=ctxCorrect;chosenGlobal=globalCorrect;
        chosenWorst=worstMargin;chosenAvg=avgMargin;
        chosenAlpha=alpha;chosenLevel=levelAlpha;
      }
    }
    anaServiceIo();
    if(ana.stopRequested){ana.optDone=false;return;}
  }

  ana.meanAlphaBest=chosenAlpha;
  ana.levelAlphaBest=chosenLevel;

  // ------------------------------------------------------------
  // 5) Finale Leave-one-repeat-out CV aus gecachten Komponenten.
  // ------------------------------------------------------------
  ana.cvCorrect=0;ana.cvTotal=ANA_COMBOS*ANA_REPEATS;
  ana.cvSpeedCorrect=0;ana.cvModeCorrect=0;ana.cvOscContextCorrect=0;
  memset(ana.cvSpeedHits,0,sizeof(ana.cvSpeedHits));
  memset(ana.cvModeHits,0,sizeof(ana.cvModeHits));
  memset(ana.cvOscHits,0,sizeof(ana.cvOscHits));
  ana.cvContextStateFailures=0;
  ana.cvWorstMargin=100.0f;

  float rootSum=0.0f,rootSq=0.0f;
  int rootN=0;

  for(int c=0;c<ANA_COMBOS;c++) {
    int tsp=anaSpeedOf(c)-1,tmd=anaModeOf(c),tos=anaOscOf(c);

    for(int r=0;r<ANA_REPEATS;r++) {
      int si=c*ANA_REPEATS+r;

      float best=1e30f,second=1e30f,correctD=1e30f;
      float bS=1e30f,bM=1e30f,bO=1e30f;
      int got=-1,gS=-1,gM=-1,gO=-1;

      for(int k=0;k<ANA_COMBOS;k++) {
        float d=cvCdfD[si][k] +
                ana.meanAlphaBest*cvMeanD[si][k] +
                ana.levelAlphaBest*cvLevelD[si][k];

        if(k==c)correctD=d;
        if(d<best){second=best;best=d;got=k;}
        else if(d<second)second=d;

        int sp=anaSpeedOf(k)-1,md=anaModeOf(k),os=anaOscOf(k);
        if(md==tmd&&os==tos&&d<bS){bS=d;gS=sp;}
        if(sp==tsp&&os==tos&&d<bM){bM=d;gM=md;}
        if(sp==tsp&&md==tmd&&d<bO){bO=d;gO=os;}
      }

      if(got==c)ana.cvCorrect++;
      if(gS==tsp){ana.cvSpeedCorrect++;ana.cvSpeedHits[c]++;}
      if(gM==tmd){ana.cvModeCorrect++;ana.cvModeHits[c]++;}
      if(gO==tos){ana.cvOscContextCorrect++;ana.cvOscHits[c]++;}

      float rb=sqrtf(max(best,0.0f)),rs=sqrtf(max(second,0.0f));
      float margin=(rs>0.0001f)?((rs-rb)/rs*100.0f):0.0f;
      ana.cvWorstMargin=min(ana.cvWorstMargin,margin);

      float rc=sqrtf(max(correctD,0.0f));
      rootSum+=rc;rootSq+=rc*rc;rootN++;
    }
  }

  for(int c=0;c<ANA_COMBOS;c++) {
    if(ana.cvSpeedHits[c]<3 || ana.cvModeHits[c]<3 || ana.cvOscHits[c]<3)
      ana.cvContextStateFailures++;
  }

  float rootMean=rootN?rootSum/rootN:1.0f;
  float rootVar=rootN>1?(rootSq-rootN*rootMean*rootMean)/(rootN-1):0.0f;
  if(rootVar<0)rootVar=0;
  float rootSd=sqrtf(rootVar);
  ana.fitLimit=max(0.50f,(rootMean+4.0f*rootSd)*1.12f);

  // ------------------------------------------------------------
  // 6) Rich-Modell Gesamttrennung.
  // ------------------------------------------------------------
  float worstFactor=1e30f;

  for(int a=0;a<ANA_COMBOS;a++) {
    float ar[5][RICH_CDF_LEVELS],araw[5],am[5];
    anaRichProfileMean(a,ar);anaProfileRawMean(a,araw);normalizedMeanFeature(araw,am);
    float al=rawLevelFeature(araw);

    float noiseA=0.0f;
    for(int r=0;r<ANA_REPEATS;r++) {
      float tr[5][RICH_CDF_LEVELS],tm[5],tl=0.0f;
      anaRichTestFeatures(a,r,tr,tm,tl);
      noiseA+=anaRichDistance(tr,tm,tl,ar,am,al,
                              ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
    }
    noiseA=sqrtf(noiseA/ANA_REPEATS);

    for(int b=a+1;b<ANA_COMBOS;b++) {
      float br[5][RICH_CDF_LEVELS],braw[5],bm[5];
      anaRichProfileMean(b,br);anaProfileRawMean(b,braw);normalizedMeanFeature(braw,bm);
      float bl=rawLevelFeature(braw);

      float noiseB=0.0f;
      for(int r=0;r<ANA_REPEATS;r++) {
        float tr[5][RICH_CDF_LEVELS],tm[5],tl=0.0f;
        anaRichTestFeatures(b,r,tr,tm,tl);
        noiseB+=anaRichDistance(tr,tm,tl,br,bm,bl,
                                ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
      }
      noiseB=sqrtf(noiseB/ANA_REPEATS);

      float d=anaRichDistance(ar,am,al,br,bm,bl,
                              ana.shapeBest,ana.meanAlphaBest,ana.levelAlphaBest);
      float denom=max(max(noiseA,noiseB),0.05f);
      worstFactor=min(worstFactor,sqrtf(max(d,0.0f))/denom);
    }
  }

  ana.shapeScore=worstFactor;
  ana.optDone=true;
}

String analyzeJson() {
  // Waehrend des langen RICH-CDF-Laufs nur einen kleinen Status senden.
  // Die alten Builds erzeugten bei JEDEM Poll den kompletten ~15-kB-
  // Ergebnisblock. Das kostet Heap, Funkzeit und kann die Webverbindung
  // gerade waehrend langer Messphasen unnötig instabil machen.
  String j;
  j.reserve(ana.done?15000:1200);
  j="{";
  j+="\"running\":"+String(ana.running?"true":"false")+",";
  j+="\"done\":"+String(ana.done?"true":"false")+",";
  j+="\"aborted\":"+String(ana.aborted?"true":"false")+",";
  j+="\"preparing\":"+String(ana.preparing?"true":"false")+",";
  j+="\"resuming\":"+String(ana.resuming?"true":"false")+",";
  j+="\"checkpointFs\":"+String(anaCheckpointFsReady?"true":"false")+",";
  j+="\"checkpointCapable\":"+String(anaCheckpointCapable?"true":"false")+",";
  j+="\"recoveryAvailable\":"+String(anaRecoveryAvailable?"true":"false")+",";
  j+="\"recoverySafe\":"+String(anaRecoverySafePowerCycle?"true":"false")+",";
  j+="\"checkpointSteps\":"+String(anaCheckpointSavedSteps)+",";
  j+="\"checkpointSequence\":"+String((unsigned long)anaCheckpointSequence)+",";
  j+="\"checkpointFreeBytes\":"+String((unsigned long)anaCheckpointFreeBytes)+",";
  j+="\"checkpointStatus\":\""+String(anaCheckpointStatus)+"\",";
  j+="\"runStartBootId\":"+String((unsigned long)ana.runStartBootId)+",";
  j+="\"resumeCount\":"+String(ana.resumeCount)+",";
  j+="\"resumeLoadedSteps\":"+String(ana.resumeLoadedSteps)+",";
  j+="\"resumeRollbackProfiles\":"+String(ana.resumeRollbackProfiles)+",";
  j+="\"repeat\":"+String(ana.repeat)+",";
  j+="\"sequenceStep\":"+String(ana.combo)+",";
  j+="\"fanOn\":"+String(fanIsOn?"true":"false")+",";
  j+="\"commandSpeed\":"+String(fanIsOn?state.speed:0)+",";
  j+="\"commandMode\":"+String(fanIsOn?state.mode:0)+",";
  j+="\"commandOsc\":"+String(fanIsOn&&state.osc?"true":"false")+",";
  int total=ANA_COMBOS*ANA_REPEATS,steps=ana.repeat*ANA_COMBOS+ana.combo;
  int progress=0;
  if(ana.done)progress=100;
  else if(ana.preparing)progress=1;
  else if(ana.repeat<ANA_REPEATS)progress=total?min(70,steps*70/total):0;
  else if(!ana.verifyDone) {
    int hp=min((int)ana.holdTotal,(int)ANA_HOLDOUT_MAX_TOTAL);
    progress=70+hp*24/max(1,(int)ANA_HOLDOUT_MAX_TOTAL);
  }
  else progress=99;
  j+="\"trainingTotal\":"+String(total)+",";
  j+="\"holdoutBaseTotal\":"+String(ANA_HOLDOUT_BASE_TOTAL)+",";
  j+="\"holdoutMaxTotal\":"+String(ANA_HOLDOUT_MAX_TOTAL)+",";
  j+="\"progress\":"+String(progress)+",";
  j+="\"action\":\""+String(ana.action)+"\",";

  if(!ana.done) {
    j+="\"canApply\":false";
    j+="}";
    return j;
  }

  j+="\"verifyDone\":"+String(ana.verifyDone?"true":"false")+",";
  j+="\"offVerifyDone\":"+String(ana.offVerifyDone?"true":"false")+",";
  j+="\"offVerifyHits\":"+String(ana.offVerifyHits)+",";
  j+="\"offVerifyWorstDrift\":"+String(ana.offVerifyWorstDrift,2)+",";
  j+="\"offVerifyWorstRatio\":"+String(ana.offVerifyWorstRatio,2)+",";
  j+="\"verifyOff\":"+String(ana.verifyOff,2)+",\"verifyOn\":"+String(ana.verifyOn,2)+",";
  j+="\"verifyOffHits\":"+String(ana.verifyOffHits)+",\"verifyOnHits\":"+String(ana.verifyOnHits)+",";
  j+="\"verifyRounds\":"+String(ANA_VERIFY_ROUNDS)+",";
  j+="\"cvCorrect\":"+String(ana.cvCorrect)+",\"cvTotal\":"+String(ana.cvTotal)+",";
  j+="\"cvSpeedCorrect\":"+String(ana.cvSpeedCorrect)+",";
  j+="\"cvModeCorrect\":"+String(ana.cvModeCorrect)+",";
  j+="\"cvOscContextCorrect\":"+String(ana.cvOscContextCorrect)+",";
  j+="\"cvContextStateFailures\":"+String(ana.cvContextStateFailures)+",";
  j+="\"cvWorstMargin\":"+String(ana.cvWorstMargin,2)+",";
  j+="\"holdCorrect\":"+String(ana.holdCorrect)+",\"holdTotal\":"+String(ana.holdTotal)+",";
  j+="\"holdAdaptiveStates\":"+String(ana.holdAdaptiveStates)+",";
  j+="\"holdImpossibleStates\":"+String(ana.holdImpossibleStates)+",";
  j+="\"holdStateFailures\":"+String(ana.holdStateFailures)+",";
  j+="\"holdOscCorrect\":"+String(ana.holdOscCorrect)+",";
  j+="\"holdSpeedContextCorrect\":"+String(ana.holdSpeedContextCorrect)+",";
  j+="\"holdModeContextCorrect\":"+String(ana.holdModeContextCorrect)+",";
  j+="\"holdOscContextCorrect\":"+String(ana.holdOscContextCorrect)+",";
  j+="\"holdContextStateFailures\":"+String(ana.holdContextStateFailures)+",";
  bool analysisGlobalSyncSafe=ana.cvTotal>0&&ana.holdTotal>=ANA_HOLDOUT_BASE_TOTAL&&
    ((uint32_t)ana.cvCorrect*100u)>=((uint32_t)ana.cvTotal*90u)&&
    ((uint32_t)ana.holdCorrect*100u)>=((uint32_t)ana.holdTotal*85u)&&
    ana.holdStateFailures<=3;
  j+="\"analysisGlobalSyncSafe\":"+String(analysisGlobalSyncSafe?"true":"false")+",";
  j+="\"holdWorstMargin\":"+String(ana.holdWorstMargin,2)+",";
  j+="\"meanAlpha\":"+String(ana.meanAlphaBest,3)+",";
  j+="\"levelAlpha\":"+String(ana.levelAlphaBest,3)+",";
  j+="\"fitLimit\":"+String(ana.fitLimit,3)+",";
  j+="\"meanWeights\":[";for(int ch=0;ch<5;ch++){if(ch)j+=",";j+=String(ana.meanWeight[ch],3);}j+="],";
  // Qualitaetswerte bleiben sichtbar/exportierbar, sind aber keine Sperre.
  bool canApply=ana.done&&ana.optDone&&ana.offVerifyDone&&ana.verifyDone&&ana.offMeasured;
  j+="\"canApply\":"+String(canApply?"true":"false")+",";
  j+="\"offMeasured\":"+String(ana.offMeasured?"true":"false")+",";

  // Feste Vergleichsbasis fuer Menschen und Run-zu-Run-Vergleich.
  // Die v6.3.x-Resume-Messung zeigte, dass ein Optimierer-Sprung von
  // LED2 3072->3136 scheinbar ALLE Profile veraenderte, obwohl 17/18
  // Histogramme identisch aus dem Checkpoint kamen.
  constexpr uint8_t refBin3072=(uint8_t)(3072>>ANA_BIN_SHIFT);

  j+="\"displayReferenceAdc\":3072,";
  j+="\"off\":[";
  for(int ch=0;ch<5;ch++){if(ch)j+=",";j+=String(ana.offMeasured?anaOffPctAt(ch,refBin3072):ana.offPct[ch],1);}
  j+="],\"combos\":[";
  for(int c=0;c<ANA_COMBOS;c++) {
    if(c)j+=",";
    j+="{\"speed\":"+String(anaSpeedOf(c))+",\"mode\":"+String(anaModeOf(c))+",\"osc\":"+String(anaOscOf(c))+",\"ch\":[";
    for(int ch=0;ch<5;ch++) {
      if(ch)j+=",";
      j+="[";
      for(int r=0;r<ANA_REPEATS;r++){if(r)j+=",";float v=anaAbovePct(c,ch,r,refBin3072);j+=String(v,2);}
      j+="]";
    }
    j+="]}";
  }
  j+="],";

  j+="\"shape\":"+String(ana.shapeBest,2)+",\"shapeScore\":"+String(ana.shapeScore,2)+",";
  j+="\"weights\":[";for(int ch=0;ch<5;ch++){if(ch)j+=",";j+=String(ana.weight[ch],3);}j+="],";
  j+="\"opt\":[";
  for(int ch=0;ch<5;ch++) {
    if(ch)j+=",";
    j+="{\"thr\":"+String(ana.bestThr[ch])+",\"speed\":"+String(ana.scoreSpeed[ch],1)+
       ",\"mode\":"+String(ana.scoreMode[ch],1)+",\"osc\":"+String(ana.scoreOsc[ch],1)+
       ",\"off\":"+String(ana.scoreOff[ch],1)+"}";
  }
  j+="]}";return j;
}

void sendAnalysisAiExport(bool includeHistograms) {
  if(ana.running) {
    server.send(409,"application/json",
                "{\"ok\":false,\"error\":\"analyse_laeuft_noch\"}");
    return;
  }
  if(!ana.done) {
    server.send(409,"application/json",
                "{\"ok\":false,\"error\":\"kein_abgeschlossener_messlauf\"}");
    return;
  }

  String filename="fan_ai_measurement_";
  filename+=String(FW_VERSION);
  filename+=includeHistograms?"_raw.json":"_compact.json";

  server.sendHeader("Cache-Control","no-store");
  String disposition="attachment; filename=\""+filename+"\"";
  server.sendHeader("Content-Disposition",disposition.c_str());
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"application/json; charset=utf-8","");

  String out;
  out.reserve(4096);

  auto flush=[&](){
    if(out.length()) {
      server.sendContent(out);
      out="";
      yield();
    }
  };
  auto add=[&](const String& s){
    out+=s;
    if(out.length()>3400)flush();
  };
  auto addFloatArray=[&](const float* v,int n,int digits){
    add("[");
    for(int i=0;i<n;i++){if(i)add(",");add(String(v[i],digits));}
    add("]");
  };

  add("{");
  add("\"schema\":\"fan-analysis-ai-v4\",");
  add("\"firmware\":\""+String(FW_VERSION)+"\",");
  add("\"sampler\":\"FINAL_EFFICIENT_BALANCED_V6\",");
  add("\"include_histograms\":");add(includeHistograms?"true,":"false,");
  add("\"boot_id_current\":");add(String((unsigned long)bootSessionId));add(",");
  add("\"run_start_boot_id\":");add(String((unsigned long)ana.runStartBootId));add(",");

  add("\"resume\":{");
  add("\"count\":");add(String(ana.resumeCount));add(",");
  add("\"loaded_steps\":");add(String(ana.resumeLoadedSteps));add(",");
  add("\"rollback_profiles\":");add(String(ana.resumeRollbackProfiles));add(",");
  add("\"balanced_repeat_resume\":true,");
  add("\"repeat_boot_ids\":[");
  for(int r=0;r<ANA_REPEATS;r++){if(r)add(",");add(String((unsigned long)ana.trainBootId[r]));}
  add("]},");

  add("\"acquisition\":{");
  add("\"adc_bits\":12,");
  add("\"sample_cycles_per_window\":");add(String(PROFILE_SAMPLE_CYCLES));add(",");
  add("\"training_repeats\":");add(String(ANA_REPEATS));add(",");
  add("\"holdout_base_rounds\":");add(String(ANA_HOLDOUT_BASE_ROUNDS));add(",");
  add("\"holdout_max_adaptive_states\":");add(String(ANA_HOLDOUT_MAX_ADAPTIVE_STATES));add(",");
  add("\"holdout_strategy\":\"2_full_passes_plus_1_adaptive_if_1_of_2\",");
  add("\"transition_strategy\":\"web_equivalent_direct_speed_mode_osc\",");
  add("\"final_profile_blend\":\"after_validation_train4_plus_holdout_equivalent2\",");
  add("\"histogram_bins\":");add(String(ANA_BINS));add(",");
  add("\"histogram_bin_shift\":");add(String(ANA_BIN_SHIFT));add(",");
  add("\"jitter_us\":[");add(String(PROFILE_JITTER_MIN_US));add(",");add(String(PROFILE_JITTER_MAX_US));add("],");
  add("\"series_gap_ms\":[");add(String(PROFILE_SERIES_GAP_MIN_MS));add(",");add(String(PROFILE_SERIES_GAP_MAX_MS));add("],");
  add("\"windows\":{\"normal_off\":");add(String(ANA_WINDOWS_NORMAL_OFF));
  add(",\"normal_on\":");add(String(ANA_WINDOWS_NORMAL_ON));
  add(",\"dynamic_off\":");add(String(ANA_WINDOWS_DYNAMIC_OFF));
  add(",\"dynamic_on\":");add(String(ANA_WINDOWS_DYNAMIC_ON));add("},");
  add("\"cdf_adc\":[");
  for(int q=0;q<RICH_CDF_LEVELS;q++){if(q)add(",");add(String(RICH_CDF_ADC[q]));}
  add("],\"human_reference_adc\":3072,");

  // Reihenfolgen explizit exportieren, damit eine KI zeit-/temperaturbedingte
  // Drift gegen die Position im Messlauf analysieren kann.
  add("\"training_order\":[");
  for(int r=0;r<ANA_REPEATS;r++) {
    if(r)add(",");
    add("[");
    for(int seq=0;seq<ANA_COMBOS;seq++) {
      if(seq)add(",");
      add(String(anaTrainingCombo(r,seq)));
    }
    add("]");
  }
  add("],\"holdout_order\":[");
  for(int r=0;r<ANA_HOLDOUT_ROUNDS;r++) {
    if(r)add(",");
    add("[");
    for(int seq=0;seq<ANA_COMBOS;seq++) {
      if(seq)add(",");
      add(String(anaHoldoutCombo(r,seq)));
    }
    add("]");
  }
  add("],\"adaptive_prime\":[");
  for(int c=0;c<ANA_COMBOS;c++){if(c)add(",");add(String(ANA_HOLDOUT_ADAPTIVE_PRIME[c]));}
  add("]");
  add("},");

  add("\"summary\":{");
  add("\"off_verify_hits\":");add(String(ana.offVerifyHits));add(",");
  add("\"off_verify_worst_drift_pp\":");add(String(ana.offVerifyWorstDrift,4));add(",");
  add("\"off_verify_worst_ratio_pct\":");add(String(ana.offVerifyWorstRatio,4));add(",");
  add("\"verify_off_drift_pp\":");add(String(ana.verifyOff,4));add(",");
  add("\"verify_on_drift_pp\":");add(String(ana.verifyOn,4));add(",");
  add("\"verify_off_hits\":");add(String(ana.verifyOffHits));add(",");
  add("\"verify_on_hits\":");add(String(ana.verifyOnHits));add(",");
  add("\"cv_global_correct\":");add(String(ana.cvCorrect));add(",");
  add("\"cv_total\":");add(String(ana.cvTotal));add(",");
  add("\"cv_speed_context_correct\":");add(String(ana.cvSpeedCorrect));add(",");
  add("\"cv_mode_context_correct\":");add(String(ana.cvModeCorrect));add(",");
  add("\"cv_osc_context_correct\":");add(String(ana.cvOscContextCorrect));add(",");
  add("\"cv_context_state_failures\":");add(String(ana.cvContextStateFailures));add(",");
  add("\"cv_worst_margin_pct\":");add(String(ana.cvWorstMargin,4));add(",");
  add("\"hold_global_correct\":");add(String(ana.holdCorrect));add(",");
  add("\"hold_total\":");add(String(ana.holdTotal));add(",");
  add("\"hold_speed_context_correct\":");add(String(ana.holdSpeedContextCorrect));add(",");
  add("\"hold_mode_context_correct\":");add(String(ana.holdModeContextCorrect));add(",");
  add("\"hold_osc_context_correct\":");add(String(ana.holdOscContextCorrect));add(",");
  add("\"hold_adaptive_states\":");add(String(ana.holdAdaptiveStates));add(",");
  add("\"hold_impossible_states\":");add(String(ana.holdImpossibleStates));add(",");
  add("\"hold_context_state_failures\":");add(String(ana.holdContextStateFailures));add(",");
  add("\"hold_global_state_failures\":");add(String(ana.holdStateFailures));add(",");
  add("\"hold_worst_margin_pct\":");add(String(ana.holdWorstMargin,4));
  add("},");

  add("\"model\":{");
  add("\"best_threshold_adc\":[");
  for(int ch=0;ch<5;ch++){if(ch)add(",");add(String(ana.bestThr[ch]));}
  add("],");
  add("\"shape\":");add(String(ana.shapeBest,6));add(",");
  add("\"shape_score\":");add(String(ana.shapeScore,6));add(",");
  add("\"mean_alpha\":");add(String(ana.meanAlphaBest,6));add(",");
  add("\"level_alpha\":");add(String(ana.levelAlphaBest,6));add(",");
  add("\"fit_limit\":");add(String(ana.fitLimit,6));add(",");
  add("\"channel_weights\":");addFloatArray(ana.weight,5,7);add(",");
  add("\"mean_weights\":");addFloatArray(ana.meanWeight,5,7);add(",");
  add("\"rich_weights\":[");
  for(int ch=0;ch<5;ch++){
    if(ch)add(",");
    add("[");
    for(int q=0;q<RICH_CDF_LEVELS;q++){if(q)add(",");add(String(ana.richWeight[ch][q],8));}
    add("]");
  }
  add("]},");

  add("\"off\":{");
  add("\"mean_adc\":");addFloatArray(ana.offMean,5,3);add(",");
  add("\"cdf_pct\":[");
  for(int ch=0;ch<5;ch++){
    if(ch)add(",");
    add("[");
    for(int q=0;q<RICH_CDF_LEVELS;q++){
      if(q)add(",");
      add(String(anaOffPctAt(ch,(uint8_t)(RICH_CDF_ADC[q]>>ANA_BIN_SHIFT)),5));
    }
    add("]");
  }
  add("]");
  if(includeHistograms){
    add(",\"hist64\":[");
    for(int ch=0;ch<5;ch++){
      if(ch)add(",");
      add("[");
      for(int b=0;b<ANA_BINS;b++){if(b)add(",");add(String(ana.offHistC[ch][b]));}
      add("]");
    }
    add("]");
  }
  add("},");

  add("\"states\":[");
  constexpr uint8_t refBin=(uint8_t)(3072>>ANA_BIN_SHIFT);

  for(int c=0;c<ANA_COMBOS;c++) {
    if(c)add(",");
    add("{");
    add("\"index\":");add(String(c));add(",");
    add("\"speed\":");add(String(anaSpeedOf(c)));add(",");
    add("\"mode\":");add(String(anaModeOf(c)));add(",");
    add("\"osc\":");add(anaOscOf(c)?"true,":"false,");
    add("\"cv_context_hits\":{\"speed\":");add(String(ana.cvSpeedHits[c]));
    add(",\"mode\":");add(String(ana.cvModeHits[c]));
    add(",\"osc\":");add(String(ana.cvOscHits[c]));
    add(",\"total\":");add(String(ANA_REPEATS));add("},");

    add("\"reference_3072\":{\"mean_pct\":[");
    for(int ch=0;ch<5;ch++){
      if(ch)add(",");
      float m=0.0f;
      for(int r=0;r<ANA_REPEATS;r++)m+=anaAbovePct(c,ch,r,refBin);
      add(String(m/ANA_REPEATS,5));
    }
    add("],\"sd_pct\":[");
    for(int ch=0;ch<5;ch++){
      if(ch)add(",");
      float m=0.0f;
      for(int r=0;r<ANA_REPEATS;r++)m+=anaAbovePct(c,ch,r,refBin);
      m/=ANA_REPEATS;
      float v=0.0f;
      for(int r=0;r<ANA_REPEATS;r++){float e=anaAbovePct(c,ch,r,refBin)-m;v+=e*e;}
      add(String(sqrtf(v/max(1,(int)ANA_REPEATS-1)),5));
    }
    add("]},");

    add("\"training\":[");
    for(int r=0;r<ANA_REPEATS;r++) {
      if(r)add(",");
      add("{\"repeat\":");add(String(r));add(",");
      add("\"boot_id\":");add(String((unsigned long)ana.trainBootId[r]));add(",");
      add("\"hist_n\":[");
      for(int ch=0;ch<5;ch++){if(ch)add(",");add(String(ana.histN[c][ch][r]));}
      add("],\"mean_adc\":[");
      for(int ch=0;ch<5;ch++){if(ch)add(",");add(String(ana.rawmean[c][ch][r],4));}
      add("],\"cdf_pct\":[");
      for(int ch=0;ch<5;ch++){
        if(ch)add(",");
        add("[");
        for(int q=0;q<RICH_CDF_LEVELS;q++){
          if(q)add(",");
          add(String(anaRichPct(c,ch,r,q),5));
        }
        add("]");
      }
      add("]");

      if(includeHistograms) {
        add(",\"hist64\":[");
        for(int ch=0;ch<5;ch++){
          if(ch)add(",");
          add("[");
          for(int b=0;b<ANA_BINS;b++){if(b)add(",");add(String(ana.histC[c][ch][r][b]));}
          add("]");
        }
        add("]");
      }
      add("}");
    }
    add("],");

    uint8_t hn=ana.holdObs[c];
    add("\"holdout\":{");
    add("\"boot_id\":");add(String((unsigned long)bootSessionId));add(",");
    add("\"samples\":");add(String(hn));add(",");
    add("\"global_hits\":");add(String(ana.holdGlobalHits[c]));add(",");
    add("\"speed_context_hits\":");add(String(ana.holdSpeedHits[c]));add(",");
    add("\"mode_context_hits\":");add(String(ana.holdModeHits[c]));add(",");
    add("\"osc_context_hits\":");add(String(ana.holdOscHits[c]));add(",");
    add("\"mean_adc\":[");
    for(int ch=0;ch<5;ch++){
      if(ch)add(",");
      add(String(hn?ana.holdMeanSum[c][ch]/hn:0.0f,4));
    }
    add("],\"cdf_pct\":[");
    for(int ch=0;ch<5;ch++){
      if(ch)add(",");
      add("[");
      for(int q=0;q<RICH_CDF_LEVELS;q++){
        if(q)add(",");
        add(String(hn?ana.holdCdfSum[c][ch][q]/hn:0.0f,5));
      }
      add("]");
    }
    add("],\"rounds\":[");
    for(int r=0;r<ANA_HOLDOUT_ROUNDS;r++) {
      if(r)add(",");
      bool measured=ana.holdRoundPredecessor[c][r]>=0;

      add("{\"round\":");add(String(r));
      add(",\"measured\":");add(measured?"true":"false");
      add(",\"predecessor\":");add(String((int)ana.holdRoundPredecessor[c][r]));
      add(",\"global_pred\":");add(String((int)ana.holdRoundGlobalPred[c][r]));
      add(",\"speed_pred\":");add(String((int)ana.holdRoundSpeedPred[c][r]));
      add(",\"mode_pred\":");add(String((int)ana.holdRoundModePred[c][r]));
      add(",\"osc_pred\":");add(String((int)ana.holdRoundOscPred[c][r]));
      add(",\"fit\":");add(String(ana.holdRoundFit[c][r],4));
      add(",\"true_fit\":");add(String(ana.holdRoundTrueFit[c][r],4));
      add(",\"margin\":");add(String(ana.holdRoundMargin[c][r],4));
      add(",\"mean_adc\":[");
      for(int ch=0;ch<5;ch++){if(ch)add(",");add(String(ana.holdRoundMean[c][r][ch],4));}
      add("],\"cdf_pct\":[");
      for(int ch=0;ch<5;ch++) {
        if(ch)add(",");
        add("[");
        for(int q=0;q<RICH_CDF_LEVELS;q++) {
          if(q)add(",");
          add(String(ana.holdRoundCdf[c][r][ch][q],5));
        }
        add("]");
      }
      add("]}");
    }
    add("]}");
    add("}");
  }

  add("]}");
  flush();
  server.sendContent("");
}

// =================== ENDE FINALE TRENNSCHAERFE-ANALYSE ===================


void setupRoutes() {
  server.on("/",HTTP_GET,[](){
    server.send_P(200,"text/html; charset=utf-8",PAGE);
  });

  // Leichtgewichtiger Verbindungscheck OHNE ADC/LED-Messung.
  // Dadurch bleibt die Online-Anzeige unabhängig von den relativ langsamen Diagnosemessungen.
  server.on("/api/health",HTTP_GET,[](){
    String ip = WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    int rssi = WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : 0;

    String j="{";
    j+="\"ok\":true,";
    j+="\"bootId\":"+String((unsigned long)bootSessionId)+",";
    j+="\"version\":\""+String(FW_VERSION)+"\",";
    j+="\"build\":\""+String(__DATE__)+" "+String(__TIME__)+"\",";
    j+="\"ip\":\""+ip+"\",";
    j+="\"rssi\":"+String(rssi)+",";
    j+="\"uptime\":"+String(millis()/1000UL);
    j+="}";
    server.send(200,"application/json",j);
  });

  server.on("/api/status",HTTP_GET,sendStatus);

  // AUS-Zustand lernen: die LED-Messung muss vorher wirklich AUS bestaetigen.
  server.on("/api/learn-off",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    if(ana.running || learnBusy) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"messung_laeuft\"}");
      return;
    }

    FbStats verify[5];
    sampleProfileAverage(verify,4,35);
    if(!ledsLookOff(verify)) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"leds_sind_nicht_aus\"}");
      return;
    }

    learnBusy=true;
    bool ok=learnOffSignature();
    learnBusy=false;

    if(ok) {
      fanIsOn=false;
      state.speed=0;state.osc=false;state.mode=0;
      cancelTimer();
      startupScanDone=true;
      startupSyncUntilMs=0;
      startupSyncReadyMs=0;
      ledDetectReadyMs=0;
      resetStableDiagnostic();
      bumpStateRevision();
      saveFanState();

      String j="{\"ok\":true,\"state\":";
      j+=buildStateObjectJson();
      j+="}";
      server.send(200,"application/json",j);
    } else {
      server.send(500,"application/json","{\"ok\":false}");
    }
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });

  server.on("/api/sync-from-leds",HTTP_POST,[](){
    if(ana.running) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");
      return;
    }

#if ENABLE_LED_FEEDBACK
    if(!calibrationSamplerCurrent || !comboOscComplete() || !comboRichValid) {
      server.send(409,"application/json",
                  "{\"ok\":false,\"error\":\"finale_18p_kalibrierung_fehlt\"}");
      return;
    }

    // v6.4.8: Manueller Sync und laufende Diagnose muessen dieselbe
    // Messmethodik verwenden. v6.4.7 hatte hier einen konkreten Fehler:
    // fuer NORMAL wurde immer mit 6 Fenstern gemessen, auch wenn Drehen AN
    // war. Die trainierte/diagnostische Tiefe ist aber Normal AUS=6 und
    // Normal AN=8 (Breeze/Nacht AUS=8, AN=10). Genau dadurch konnte die
    // Kalibrierungsanzeige Speed 3 sehen, waehrend der manuelle Sync denselben
    // Zustand als Speed 1 uebernahm.
    //
    // Zusaetzlich wird ein frischer, bereits stabilisierter Diagnose-Speed
    // bevorzugt. Falls keiner vorhanden ist, entscheidet zuerst die Mehrheit
    // der drei einzeln klassifizierten Batches; nur bei fehlender Mehrheit
    // faellt die gemittelte Formdistanz die Entscheidung.
    constexpr uint8_t BATCHES=3;
    float pairDist[3][3]={{0}};
    float oscScoreSum[3][3]={{0}};
    uint8_t speedVotes[3][3]={{0}};
    uint8_t oscOnVotes[3][3]={{0}};
    uint8_t oscOffVotes[3][3]={{0}};
    uint8_t offVotes=0;
    uint8_t nonOffBatches=0;

    int preferredMode=(state.mode<=2)?(int)state.mode:0;
    if(!fanIsOn)preferredMode=0; // Einschalten von Hand -> NORMAL als sicherster Prior.

    // Exakt dieselbe Messtiefe wie FINAL_EFFICIENT_BALANCED_V6 und die
    // laufende Hintergrunddiagnose. Besonders wichtig: Normal + Drehen AN
    // braucht 8 statt 6 Fenster.
    bool expectedOsc=fanIsOn?state.osc:false;
    uint8_t windows=anaWindowsFor((uint8_t)preferredMode,expectedOsc);

    // Wenn die Kalibrierungsanzeige unmittelbar vor dem Klick bereits einen
    // anderen Speed stabil erkannt hat, soll genau dieser Wert auch wirklich
    // uebernommen werden. Nur frische und intern stabilisierte Werte zaehlen.
    int cachedStableSpeed=-1;
    float cachedStableSpeedConf=0.0f;
    if(diagCacheValid && !diagCacheOff && diagCacheAtMs!=0 &&
       (unsigned long)(millis()-diagCacheAtMs)<=7000UL &&
       stableDiagSpeed>=1 && stableDiagSpeed<=3 &&
       diagCacheSpeed==stableDiagSpeed &&
       (!fanIsOn || stableDiagSpeed!=(int)state.speed) &&
       stableDiagSpeedConf>=20.0f) {
      cachedStableSpeed=stableDiagSpeed-1;
      cachedStableSpeedConf=stableDiagSpeedConf;
    }

    for(uint8_t b=0;b<BATCHES;b++) {
      FbStats cur[5];
      sampleProfileAverage(cur,windows,250+(b*37));

      if(ledsLookOff(cur)) {
        offVotes++;
      } else {
        nonOffBatches++;
        manualSyncAccumulateBatch(cur,pairDist,speedVotes,
                                  oscOnVotes,oscOffVotes,oscScoreSum);
      }

      delay(65+(b*17));
      yield();
    }

    if(offVotes>=2) {
      fanIsOn=false;
      state.speed=0;
      state.mode=0;
      state.osc=false;
      cancelTimer();
      ledDetectReadyMs=0;
      resetStableDiagnostic();

      bumpStateRevision();
      saveFanState();

      String j="{\"ok\":true,\"detectedOff\":true,\"votes\":"+String(offVotes)+",\"state\":";
      j+=buildStateObjectJson();
      j+="}";
      server.send(200,"application/json",j);
      return;
    }

    if(nonOffBatches==0) {
      server.send(409,"application/json",
                  "{\"ok\":false,\"error\":\"keine_led_messung\"}");
      return;
    }

    // 1) Speed zuerst INNERHALB des bekannten Modus bestimmen.
    // Genau das ist fuer den fast ausschliesslichen NORMAL-Betrieb die
    // robusteste Cross-Boot-Entscheidung.
    int md=preferredMode;

    // Distanzgewinner ueber alle Batches.
    int spDist=0;
    if(pairDist[1][md]<pairDist[spDist][md])spDist=1;
    if(pairDist[2][md]<pairDist[spDist][md])spDist=2;

    // Mehrheitsgewinner der einzeln klassifizierten Batches. Das ist
    // robuster als nur die Distanzen zusammenzumitteln, wenn das Multiplex
    // zwischen zwei Formen schwankt.
    int spVote=0;
    if(speedVotes[md][1]>speedVotes[md][spVote])spVote=1;
    if(speedVotes[md][2]>speedVotes[md][spVote])spVote=2;
    uint8_t bestVoteCount=speedVotes[md][spVote];

    int sp=spDist;
    const char* speedSource="distance";
    if(bestVoteCount>=2) {
      sp=spVote;
      speedSource="batch_vote";
    }
    if(cachedStableSpeed>=0) {
      sp=cachedStableSpeed;
      speedSource="stable_diag";
    }

    // 2) Modus nur dann vom Web-Prior wegbewegen, wenn derselbe erkannte Speed
    // in einem anderen Modus wesentlich besser zur relativen Kanalform passt.
    int bestMd=0;
    if(pairDist[sp][1]<pairDist[sp][bestMd])bestMd=1;
    if(pairDist[sp][2]<pairDist[sp][bestMd])bestMd=2;

    bool modeChangedByEvidence=false;
    if(bestMd!=preferredMode) {
      float curAvg=pairDist[sp][preferredMode]/nonOffBatches;
      float bestAvg=pairDist[sp][bestMd]/nonOffBatches;
      bool allow=false;

      if(preferredMode==0) {
        // NORMAL ist absichtlich "sticky". Weg davon nur bei sehr klarer
        // Formverbesserung. Das entspricht dem realen Nutzungsprofil.
        allow=(bestAvg<=0.12f &&
               (curAvg-bestAvg)>=0.055f &&
               bestAvg<=(curAvg*0.65f));
      } else if(bestMd==0) {
        // Zurueck zu NORMAL etwas leichter erlauben.
        allow=(bestAvg<=0.16f &&
               (curAvg-bestAvg)>=0.025f &&
               bestAvg<=(curAvg*0.85f));
      } else {
        allow=(bestAvg<=0.12f &&
               (curAvg-bestAvg)>=0.050f &&
               bestAvg<=(curAvg*0.70f));
      }

      if(allow) {
        md=bestMd;
        modeChangedByEvidence=true;

        // Bei echtem Moduswechsel Speed nochmals in diesem Modus bestimmen.
        // Der gecachte Diagnose-Speed gilt nur fuer den bisherigen Web-Modus,
        // daher hier bewusst neu aus den drei Batches entscheiden.
        int spDist2=0;
        if(pairDist[1][md]<pairDist[spDist2][md])spDist2=1;
        if(pairDist[2][md]<pairDist[spDist2][md])spDist2=2;

        int spVote2=0;
        if(speedVotes[md][1]>speedVotes[md][spVote2])spVote2=1;
        if(speedVotes[md][2]>speedVotes[md][spVote2])spVote2=2;

        sp=(speedVotes[md][spVote2]>=2)?spVote2:spDist2;
        speedSource=(speedVotes[md][spVote2]>=2)?"batch_vote":"distance";
      }
    }

    float bestDist=pairDist[sp][md]/nonOffBatches;
    float secondDist=1e30f;
    for(int s=0;s<3;s++)
      if(s!=sp)secondDist=min(secondDist,pairDist[s][md]/nonOffBatches);

    float speedConf=pctShapeConfidence(bestDist,secondDist);
    float speedFit=pctShapeFit(bestDist);
    uint8_t winVotes=speedVotes[md][sp];

    if(strcmp(speedSource,"stable_diag")==0) {
      speedConf=max(speedConf,cachedStableSpeedConf);
    }

    // 3) Drehen: OSC-LINEAR darf den bekannten Webzustand korrigieren, aber
    // bei einem Widerspruch nur mit 3/3-Batchkonsens und klarer Distanz zur
    // Entscheidungsgrenze. So veraendert ein manueller Speedwechsel nicht
    // nebenbei faelschlich Drehen.
    bool candidateOsc=oscOnVotes[sp][md]>=oscOffVotes[sp][md];
    uint8_t oscVotes=candidateOsc?oscOnVotes[sp][md]:oscOffVotes[sp][md];

    float avgOscScore=oscScoreSum[sp][md]/nonOffBatches;
    float z=constrain(avgOscScore,-12.0f,12.0f);
    float p=1.0f/(1.0f+expf(-z));
    float oscConf=fabsf(p-0.5f)*200.0f;

    bool os=candidateOsc;
    bool oscChangedByEvidence=true;
    if(fanIsOn && candidateOsc!=state.osc) {
      bool strong=(oscVotes==nonOffBatches && oscConf>=25.0f);
      if(!strong) {
        os=state.osc;
        oscChangedByEvidence=false;
      }
    }

    // Die explizite Benutzeraktion uebernimmt das Ergebnis auch dann, wenn
    // der alte globale 18P-Qualitaetswert schlecht ist. Speed wird immer aus
    // dem normal-priorisierten Formmodell aktualisiert.
    fanIsOn=true;
    state.speed=sp+1;
    state.mode=md;
    state.osc=os;
    state.lastSpeed=state.speed;
    state.lastMode=state.mode;
    state.lastOsc=state.osc;

    bumpStateRevision();
    saveFanState();
    armLedDiagnostic(state.osc?8000UL:3000UL);

    const char* modeName=(md==0)?"Normal":((md==1)?"Breeze":"Nacht");

    String j="{\"ok\":true,\"detectedOff\":false,\"votes\":"+String(winVotes);
    j+=",\"fit\":"+String(speedFit,1)+",\"confidence\":"+String(speedConf,1);
    j+=",\"speedApplied\":true,\"modeApplied\":true,\"oscApplied\":true";
    j+=",\"modeChangedByEvidence\":"+String(modeChangedByEvidence?"true":"false");
    j+=",\"oscChangedByEvidence\":"+String(oscChangedByEvidence?"true":"false");
    j+=",\"globalSyncSafeDiagnostic\":"+String(comboGlobalSyncSafe?"true":"false");
    j+=",\"speedSource\":\""+String(speedSource)+"\"";
    j+=",\"sampleWindows\":"+String(windows);
    j+=",\"speed\":"+String(state.speed)+",\"mode\":"+String(md);
    j+=",\"modeName\":\""+String(modeName)+"\",\"osc\":"+String(state.osc?"true":"false");
    j+=",\"state\":"+buildStateObjectJson()+"}";
    server.send(200,"application/json",j);
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });

  server.on("/api/power",HTTP_POST,[](){
    if(ana.running){server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");return;}
    togglePower();
    sendOkState();
  });

  server.on("/api/speed",HTTP_POST,[](){
    if(ana.running){server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");return;}
    int target=jsonInt(server.arg("plain"),"target",-1);
    if(target<0||target>3){server.send(400,"text/plain","invalid speed");return;}

    if(target>0 && !fanIsOn){
      server.send(409,"application/json","{\"ok\":false,\"error\":\"power_off\"}");
      return;
    }

    setSpeed((uint8_t)target);
    sendOkState();
  });

  server.on("/api/mode",HTTP_POST,[](){
    if(ana.running){server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");return;}
    int target=jsonInt(server.arg("plain"),"target",-1);
    if(target<0||target>2){server.send(400,"text/plain","invalid mode");return;}
    if(!fanIsOn){server.send(409,"application/json","{\"ok\":false,\"error\":\"power_off\"}");return;}
    setMode((uint8_t)target);
    sendOkState();
  });

  server.on("/api/osc",HTTP_POST,[](){
    if(ana.running){server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");return;}
    if(!fanIsOn){server.send(409,"application/json","{\"ok\":false,\"error\":\"power_off\"}");return;}
    toggleOsc();
    sendOkState();
  });

  server.on("/api/timer",HTTP_POST,[](){
    if(ana.running){server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");return;}
    int minutes=jsonInt(server.arg("plain"),"minutes",0);
    minutes=constrain(minutes,0,240);
    if(minutes>0&&!fanIsOn){server.send(409,"application/json","{\"ok\":false,\"error\":\"power_off\"}");return;}
    state.sleepMin=minutes;
    timerDeadlineMs=minutes>0?millis()+(unsigned long)minutes*60000UL:0;
    bumpStateRevision();
    Serial.printf("[TIMER] auf %d min gesetzt\n",minutes);
    sendOkState();
  });

  // Nur internen Zustand zuruecksetzen. Kein physischer Tastendruck.
  server.on("/api/sync",HTTP_POST,[](){
    if(ana.running){server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");return;}
    state=FanState();
    fanIsOn=false;            // sonst: speed=0 bei fanIsOn=true -> falsche Impulszahl
    timerDeadlineMs=0;
    startupSyncUntilMs=0;
    startupSyncReadyMs=0;
    startupScanDone=false;
    manualLedSyncRequested=false;
    bumpStateRevision();
    saveFanState();
    resetStableDiagnostic();
    Serial.println("[SYNC] Interner Zustand auf AUS gesetzt");
    sendOkState();
  });



  // Nur den internen/Web-Zustand (Speed, Modus und Drehen) an den physisch
  // manuell bedienten Ventilator angleichen. Diese Route erzeugt ABSICHTLICH
  // keinerlei Tastenimpulse.
  server.on("/api/manual-state",HTTP_POST,[](){
    if(ana.running){
      server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");
      return;
    }

    String body=server.arg("plain");
    int targetSpeed=jsonInt(body,"speed",-1);
    int targetMode=jsonInt(body,"mode",-1);
    int targetOsc=jsonInt(body,"osc",-1);

    // "osc" ist ab 6.4.10 Bestandteil des manuellen Web-Status.
    // Fuer alte Clients bleibt die Route rueckwaertskompatibel und behaelt
    // ohne Feld den aktuell bzw. zuletzt gemerkten Drehen-Zustand.
    if(targetOsc<0)targetOsc=fanIsOn?(state.osc?1:0):(state.lastOsc?1:0);

    if(targetSpeed<0||targetSpeed>3||targetMode<0||targetMode>2||
       targetOsc<0||targetOsc>1){
      server.send(400,"application/json","{\"ok\":false,\"error\":\"ungueltiger_status\"}");
      return;
    }

    endStartupScan("Web-Status manuell korrigiert");
    manualLedSyncRequested=false;

    if(targetSpeed==0){
      // Physisch AUS: live ist Drehen immer AUS, aber Modus und Drehen werden
      // als explizit manuell gesetzter letzter Zustand fuer das naechste AN
      // gespeichert. Der letzte Speed bleibt unveraendert.
      fanIsOn=false;
      state.speed=0;
      state.osc=false;
      state.mode=0;
      state.lastMode=(uint8_t)targetMode;
      state.lastOsc=(targetOsc!=0);
      cancelTimer();
    }else{
      // Vollstaendige manuelle Zustandskorrektur: Speed, Modus UND Drehen
      // werden exakt auf die Benutzereingabe gesetzt. Keine Tastenimpulse.
      fanIsOn=true;
      state.speed=(uint8_t)targetSpeed;
      state.mode=(uint8_t)targetMode;
      state.osc=(targetOsc!=0);

      state.lastSpeed=state.speed;
      state.lastMode=state.mode;
      state.lastOsc=state.osc;
    }

#if ENABLE_LED_FEEDBACK
    resetStableDiagnostic();
    armLedDiagnostic(800UL);
#endif

    bumpStateRevision();
    saveFanState();

    Serial.printf("[MANUAL] Webstatus korrigiert: %s / Stufe %u / Modus %u / Drehen %s (keine Tastenimpulse)\n",
                  fanIsOn?"AN":"AUS",state.speed,state.mode,state.osc?"AN":"AUS");
    sendOkState();
  });

  server.on("/api/learn-speed",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    if(ana.running) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");
      return;
    }
    int speed = jsonInt(server.arg("plain"),"speed",-1);
    if(speed<1 || speed>3) {
      server.send(400,"text/plain","invalid speed");
      return;
    }
    if(!fanIsOn || state.speed!=speed) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"webzustand_passt_nicht_zur_speed_lernstufe\"}");
      return;
    }
    learnBusy=true;
    suppressCalibrationSave=true;
    bool ok=learnSpeedSignature(speed);
    suppressCalibrationSave=false;
    if(ok)ok=saveCompactComboCalibration(false);
    learnBusy=false;
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });


  server.on("/api/learn-mode",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    if(ana.running) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"analyse_laeuft\"}");
      return;
    }
    int mode=jsonInt(server.arg("plain"),"mode",-1);
    if(mode<0 || mode>2) {
      server.send(400,"text/plain","invalid mode");
      return;
    }
    if(!fanIsOn || state.mode!=mode) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"webzustand_passt_nicht_zum_modus_lernen\"}");
      return;
    }
    learnBusy=true;
    suppressCalibrationSave=true;
    bool ok=learnModeSignature(mode);
    suppressCalibrationSave=false;
    if(ok)ok=saveCompactComboCalibration(false);
    learnBusy=false;
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });


  server.on("/api/forget-speed-calibration",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    for(int s=0;s<3;s++) {
      memset(&speedSig[s],0,sizeof(speedSig[s]));
      speedSig[s].valid=false;
    }
    bool ok=saveCompactComboCalibration(false);
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });

  server.on("/api/forget-mode-calibration",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    for(int s=0;s<3;s++) {
      memset(&modeSig[s],0,sizeof(modeSig[s]));
      modeSig[s].valid=false;
    }
    bool ok=saveCompactComboCalibration(false);
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });


  server.on("/api/backup/export",HTTP_GET,[](){
    if(!calibrationSamplerCurrent || !comboOscComplete() || !comboComplete() || !offSig.valid || !comboWValid) {
      server.send(409,"text/plain","Finale FINAL_EFFICIENT_BALANCED_V6-Kalibrierung fehlt oder ist unvollstaendig");
      return;
    }
    String j=buildBackupJson();
    server.sendHeader("Content-Disposition","attachment; filename=ventilator-final-osc-linear-v1-backup.json");
    server.send(200,"application/json",j);
  });

  server.on("/api/backup/import",HTTP_POST,[](){
    String body=server.arg("plain");
    String err;
    if(!applyBackupJson(body,err)) {
      server.send(400,"text/plain",err);
      return;
    }
    String j="{\"ok\":true,\"state\":";
    j+=buildStateObjectJson();
    j+="}";
    server.send(200,"application/json",j);
  });

  server.on("/api/analyze/start",HTTP_POST,[](){
    String err;
    if(!analyzeStart(err)) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\""+err+"\"}");
      return;
    }
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json","{\"ok\":true,\"accepted\":true}");
  });

  server.on("/api/analyze/resume",HTTP_POST,[](){
    String err;
    if(!analyzeResume(err)) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\""+err+"\"}");
      return;
    }
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json","{\"ok\":true,\"accepted\":true,\"resumed\":true}");
  });

  server.on("/api/analyze/recovery/discard",HTTP_POST,[](){
    if(ana.running) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"messlauf_laeuft\"}");
      return;
    }
    clearAnalysisCheckpoint();
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/analyze/stop",HTTP_POST,[](){
    if(!ana.running || ana.aborted) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"kein_lauf_aktiv\"}");
      return;
    }
    ana.stopRequested = true;      // wird im naechsten Schritt ausgewertet
    Serial.println("[ANA] Stopp angefordert");
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/analyze",HTTP_GET,[](){
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",analyzeJson());
  });

  server.on("/api/analyze/export",HTTP_GET,[](){
    bool raw=server.arg("raw")=="1";
    sendAnalysisAiExport(raw);
  });

  // Schwellwerte direkt setzen, ohne Neuflashen. Erwartet {"t0":..,"t1":..,...}
  server.on("/api/thresholds",HTTP_POST,[](){
    if(ana.running || learnBusy) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"messung_laeuft\"}");
      return;
    }
    String body = server.arg("plain");
    int neu[5];
    for(int i=0;i<5;i++) {
      String key = "t" + String(i);
      neu[i] = jsonInt(body, key.c_str(), -1);
      if(neu[i] < 0 || neu[i] > 4095) {
        server.send(400,"application/json","{\"ok\":false,\"error\":\"wert_ungueltig\"}");
        return;
      }
    }
    suppressCalibrationSave=true;
    for(int i=0;i<5;i++)fbThresholds[i]=neu[i];

    // Mit neuer Schwelle sind alle dagegen gelernten Prozentprofile ungueltig.
    for(int i=0;i<3;i++){speedSig[i].valid=false;modeSig[i].valid=false;}
    offSig.valid=false;
    invalidateCombinedProfiles();
    suppressCalibrationSave=false;

    bool ok=saveCompactComboCalibration(false); // genau EIN A/B-Schreibvorgang
    Serial.printf("[SCHWELLE] neu: %d %d %d %d %d - Profile verworfen\n",
                  neu[0],neu[1],neu[2],neu[3],neu[4]);
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
  });

  server.on("/api/analyze/apply",HTTP_POST,[](){
    String err;
    if(!analyzeApply(err)) { server.send(409,"application/json","{\"ok\":false,\"error\":\""+err+"\"}"); return; }
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/analyze",HTTP_GET,[](){
    server.sendHeader("Content-Encoding","identity");
    server.send_P(200,"text/html",ANALYZE_PAGE);
  });

  // OTA per Browser - mit Passwortschutz, Messlauf-Sperre und eindeutiger
  // Session-Pruefung. Hintergrund-ADC pausiert waehrend Flash-Schreibzugriffen.
  server.on("/update",HTTP_POST,
    [](){
      if(!server.authenticate(OTA_USER,OTA_PASS)) return server.requestAuthentication();

      bool ok=otaUploadSucceeded && !Update.hasError();
      otaInProgress=false;

      server.sendHeader("Connection","close");
      server.send(ok?200:500,"text/plain",
                  ok ? "OK - Neustart" : "Update fehlgeschlagen - alte Firmware bleibt aktiv");
      Serial.printf("[OTA] Ergebnis: %s\n",ok?"ok":"FEHLER");

      otaUploadSucceeded=false;
      if(ok){delay(400);ESP.restart();}
    },
    [](){
      HTTPUpload& up=server.upload();
      static bool otaAllowed=false;

      if(up.status==UPLOAD_FILE_START) {
        otaUploadSucceeded=false;
        otaAllowed=server.authenticate(OTA_USER,OTA_PASS) && !ana.running && !learnBusy;

        if(!otaAllowed) {
          Serial.println("[OTA] Abgelehnt: Authentifizierung fehlgeschlagen oder Messung laeuft");
          otaInProgress=false;
          return;
        }

        Serial.printf("[OTA] Start: %s\n",up.filename.c_str());
        otaInProgress=true;

        if(!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          otaAllowed=false;
          otaInProgress=false;
        }
      }
      else if(!otaAllowed) {
        return;
      }
      else if(up.status==UPLOAD_FILE_WRITE) {
        if(Update.write(up.buf,up.currentSize)!=up.currentSize) {
          Update.printError(Serial);
          Update.abort();
          otaAllowed=false;
          otaInProgress=false;
          otaUploadSucceeded=false;
        }
      }
      else if(up.status==UPLOAD_FILE_END) {
        otaUploadSucceeded=Update.end(true) && !Update.hasError();
        if(!otaUploadSucceeded) {
          Update.printError(Serial);
        } else {
          Serial.printf("[OTA] %u Bytes geschrieben\n",up.totalSize);
        }
        otaAllowed=false;
        otaInProgress=false;
      }
      else if(up.status==UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaAllowed=false;
        otaInProgress=false;
        otaUploadSucceeded=false;
        Serial.println("[OTA] Abgebrochen");
      }
    }
  );
}

void startNetwork() {
  // Erst nur STA versuchen. Der Dauer-AP kostet Strom, Funkzeit und ist
  // ein offenes Scheunentor - er laeuft deshalb nur noch als Notfalloption.
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID,WIFI_PASS);

  unsigned long start=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-start<15000) delay(250);

  if(WiFi.status()==WL_CONNECTED) {
    MDNS.begin(HOSTNAME);
    Serial.printf("[NET] WLAN verbunden: %s  RSSI %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    // Kein WLAN -> Fallback-AP hochfahren, damit du drankommst.
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID,AP_PASS);
    Serial.printf("[NET] Kein WLAN. Fallback-AP '%s' auf %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
  }
}

// Wird vom Reconnect in loop() benutzt: sobald das WLAN wieder da ist,
// kann der Notfall-AP wieder abgeschaltet werden.
void updateApFallback() {
  static bool apOn = false;
  bool wantAp = (WiFi.status() != WL_CONNECTED);
  if(wantAp == apOn) return;
  apOn = wantAp;
  if(wantAp) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID,AP_PASS);
    Serial.println("[NET] WLAN weg - Fallback-AP an");
  } else {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    MDNS.begin(HOSTNAME);
    Serial.println("[NET] WLAN zurueck - Fallback-AP aus");
  }
}


// -------------------- Hintergrund-LED-Diagnose --------------------
// Lange ADC-Messungen laufen nicht mehr in /api/status. Dadurch kann ein
// alter Statusabruf keinen neueren Web-Befehl blockieren oder danach noch
// veraltete LED-Werte in die Oberfläche schreiben.
void updateLedDiagnosticTask() {
#if ENABLE_LED_FEEDBACK
  if(ana.running || learnBusy || otaInProgress || !fanIsOn)return;
  if(ledDetectReadyMs==0 || (long)(millis()-ledDetectReadyMs)<0)return;

  // Runtime verwendet dieselbe zeitliche Spreizung wie Training/Holdout.
  // Dadurch werden Breeze/Nacht nicht mehr aus einer zufaelligen Kurzphase
  // klassifiziert.
  if(diagNextSampleMs && (long)(millis()-diagNextSampleMs)<0)return;
  diagLastSampleMs=millis();

  FbStats one[5];
  sampleProfileWindow(one);

  if(diagSeriesRng==0)diagSeriesRng=profileNewSeed(0xD1A64C5Fu);
  diagNextSampleMs=millis()+profileSeriesGapMs(diagSeriesRng);

  diagCachePctValid=true;
  for(int ch=0;ch<5;ch++)diagCachePct[ch]=one[ch].abovePct;

  FbStats avg[5];

  // Exakt dieselbe mode-/osc-abhaengige Tiefe wie FINAL_EFFICIENT_BALANCED_V6.
  uint8_t needWindows=anaWindowsFor(state.mode,state.osc);
  if(!addDiagnosticSampleAndAverage(one,avg,needWindows)) {
    diagCacheValid=false;
    diagCacheOff=false;
    return;
  }

  for(int ch=0;ch<5;ch++)diagCachePct[ch]=avg[ch].abovePct;

  // AUS erst aus demselben robusten mode-/osc-abhaengigen Fenstermittel beurteilen.
  // Ein einzelnes Multiplex-/Jitter-Fenster darf die Diagnose nicht auf AUS ziehen.
  bool off=ledsLookOff(avg);
  diagCacheOff=off;

  if(off) {
    diagCacheValid=false;
    diagCacheSpeed=0;diagCacheMode=-1;diagCacheOsc=-1;
    diagCacheSpeedConf=0.0f;diagCacheModeConf=0.0f;diagCacheOscConf=0.0f;
    diagContextFit=0.0f;
    diagWebFit=0.0f;

    diagGlobalSpeed=0;diagGlobalMode=-1;diagGlobalOsc=-1;
    diagGlobalSpeedConf=0.0f;diagGlobalModeConf=0.0f;diagGlobalOscConf=0.0f;
    diagGlobalFit=0.0f;

    diagCacheAtMs=millis();

    diagSpeedCandidate=0;diagSpeedHits=0;
    diagModeCandidate=-1;diagModeHits=0;
    diagOscCandidate=-1;diagOscHits=0;
    stableDiagSpeed=0;stableDiagMode=-1;stableDiagOsc=-1;
    stableDiagSpeedConf=0.0f;stableDiagModeConf=0.0f;stableDiagOscConf=0.0f;
    return;
  }

  int gsp=0,gmd=-1,gos=-1;
  float gsc=0.0f,gmc=0.0f,goc=0.0f;

  if(comboOscComplete()) {
    detectComboOsc(avg,&gsp,&gmd,&gos,&gsc,&gmc,&goc);
  } else if(comboComplete()) {
    detectCombo(avg,&gsp,&gmd,&gsc,&gmc);
  } else {
    const float sw[5]={1.8f,1.8f,1.8f,0.08f,0.08f};
    const float mw[5]={0.08f,0.08f,0.08f,2.2f,2.2f};
    int si=detectProfilesWeightedFromCurrent(speedSig,avg,sw,&gsc);
    int mi=detectProfilesWeightedFromCurrent(modeSig,avg,mw,&gmc);
    gsp=si<0?0:si+1;
    gmd=mi;
  }

  diagGlobalSpeed=gsp;
  diagGlobalMode=gmd;
  diagGlobalOsc=gos;
  diagGlobalSpeedConf=gsc;
  diagGlobalModeConf=gmc;
  diagGlobalOscConf=goc;
  diagGlobalFit=0.0f;

  if(comboOscComplete() && gsp>=1&&gsp<=3&&gmd>=0&&gmd<=2&&gos>=0&&gos<=1)
    diagGlobalFit=comboFitQualityPct(
        comboRichDistanceToState(gsp-1,gmd,gos,avg));

  // Die normalen Diagnosefelder sind eine gezielte LED-Plausibilitaetspruefung
  // des bewusst gesetzten Webzustands. "18P Treffer" bleibt unabhaengig.
  int sp=gsp,md=gmd,os=gos;
  float sc=gsc,mc=gmc,oc=goc;

  if(comboOscComplete() && state.speed>=1&&state.speed<=3&&state.mode<=2)
    detectContextual18(avg,&sp,&sc,&md,&mc,&os,&oc);

  diagCacheSpeed=sp;
  diagCacheMode=md;
  diagCacheOsc=os;
  diagCacheSpeedConf=sc;
  diagCacheModeConf=mc;
  diagCacheOscConf=oc;
  diagContextFit=0.0f;
  diagWebFit=0.0f;

  if(comboOscComplete() && sp>=1&&sp<=3&&md>=0&&md<=2&&os>=0&&os<=1)
    diagContextFit=comboFitQualityPct(
        comboRichDistanceToState(sp-1,md,os,avg));

  if(comboOscComplete() && state.speed>=1&&state.speed<=3&&state.mode<=2) {
    if(state.mode==0) {
      diagWebFit=pctShapeFit(
          comboPctShapeDistanceToSpeedMode(state.speed-1,0,avg));
    } else {
      diagWebFit=comboFitQualityPct(
          comboRichDistanceToState(state.speed-1,state.mode,state.osc?1:0,avg));
    }
  }

  diagCacheValid=(sp>=1&&sp<=3);
  diagCacheOff=false;
  diagCacheAtMs=millis();

  if(diagCacheValid)
    updateStableDiagnostic(sp,sc,md,mc,os,oc);
#endif
}

void setup() {
  // MOSFET-Gates so frueh wie moeglich auf LOW. Das verkuerzt die
  // High-Z-Zeit nach Reset/OTA und reduziert unbeabsichtigte Tastimpulse.
  for(uint8_t p:{PIN_POWER,PIN_SPEED,PIN_OSC,PIN_MODE}) {
    digitalWrite(p,LOW);
    pinMode(p,OUTPUT);
    digitalWrite(p,LOW);
  }

  // Neue Browser-/State-Session bei jedem ESP-Neustart. Damit kann eine
  // noch offene Weboberflaeche eine nach OTA/Software-Reset wieder bei 1
  // beginnende stateRevision sicher erkennen statt neue Werte abzulehnen.
  bootSessionId=profileNewSeed(0xB00751D1u);
  if(bootSessionId==0)bootSessionId=1;

  Serial.begin(115200);
  delay(300);
  Serial.printf("\n\n=== Smart-Ventilator %s ===\nBuild: %s %s\nReset-Grund: %d\n",
                FW_VERSION, __DATE__, __TIME__, (int)esp_reset_reason());

  loadFanState();
  Serial.printf("[BOOT] gemerkt: Stufe %u, Drehen %s, Modus %u\n",
                state.lastSpeed, state.lastOsc?"an":"aus", state.lastMode);

#if ENABLE_LED_FEEDBACK
  // Ab 6.4 wird nur noch der CRC-geschuetzte A/B-Speicher der RICH-CDF-Modellgeneration geladen.
  // Alte Einzel-Key-Generationen werden nicht mehr versehentlich vermischt.
  bool finalCalOk=loadCompactComboCalibration();
  if(!finalCalOk) {
    invalidateRuntimeCalibrationForSampler();
  } else if(calStoreValidSlots<2) {
    // Ein gueltiger Slot reicht zum sicheren Boot. Den fehlenden zweiten Slot
    // sofort aus dem verifizierten RAM-Modell selbstheilend wieder aufbauen.
    if(!saveCompactComboCalibration(false))
      Serial.println("[CAL-NVS] WARNUNG: A/B-Selbstheilung fehlgeschlagen");
  }

  analogReadResolution(12);
  for(uint8_t p:FB_USED_PINS) {
    pinMode(p,INPUT);
    analogSetPinAttenuation(p,ADC_11db);
  }

  // OTA-/Software-Neustart kann bei laufendem Fan stattfinden.
  // Der persistierte Powerzustand wird dann direkt weiter diagnostiziert.
  if(fanIsOn)
    armLedDiagnostic(state.osc?8000UL:LED_DETECT_SETTLE_MS);
#endif

  initAnalysisCheckpointStorage();

  startNetwork();
  setupRoutes();
  server.begin();
}

void loop() {
  server.handleClient();
  analyzeStep();
  updateLedDiagnosticTask();

  // Eigener Timer: Taste4 des Ventilators bleibt komplett unbenutzt.
  if(timerDeadlineMs && (long)(millis()-timerDeadlineMs)>=0) {
    timerDeadlineMs=0;
    state.sleepMin=0;
    Serial.println("[TIMER] abgelaufen - schalte definiert ab");
    powerOffFromTimer();
  }

  if(timerDeadlineMs) {
    long remain=(long)(timerDeadlineMs-millis());
    state.sleepMin=remain>0 ? (remain+59999L)/60000L : 0;
  }

  // WLAN automatisch wieder verbinden; Fallback-AP bleibt dabei erreichbar.
  if(WiFi.status()!=WL_CONNECTED && millis()-lastWifiRetryMs>30000UL) {
    lastWifiRetryMs=millis();
    Serial.println("[NET] Reconnect-Versuch");
    WiFi.reconnect();
  }
  updateApFallback();
}
