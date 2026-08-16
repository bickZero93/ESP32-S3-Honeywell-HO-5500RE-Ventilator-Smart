// Smart-Ventilator / ESP32-S3-Zero
// Public GitHub build - Zugangsdaten vor dem Flashen anpassen
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
// POWER/0: AUS -> AN auf Speed1; AN -> AUS (echter Toggle)
// SPEED: 1 -> 2 -> 3 -> 1
// OSC: Toggle
// MODE: Normal -> Breeze -> Night -> Normal
//
// ACHTUNG:
// Wenn der ESP32 aus der Ventilatorplatine versorgt wird,
// USB NICHT anschliessen solange der Ventilator am Netz haengt.

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <stdarg.h>   // fuer anaSetAction (va_list)
#include <esp_system.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* HOSTNAME  = "ventilator";
const char* FW_VERSION = "5.0.0-FINAL";

// Fallback-AP, damit du dich auch bei WLAN-Problemen noch verbinden kannst.
// Wird NUR gestartet, wenn die WLAN-Verbindung fehlschlaegt (siehe startNetwork).
const char* AP_SSID = "Ventilator-Setup";
const char* AP_PASS = "CHANGE_ME_AP";   // mindestens 8 Zeichen

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

unsigned long timerDeadlineMs = 0;


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
bool startupScanDone = false;
constexpr unsigned long START_SCAN_SETTLE_MS = 3000UL;
constexpr uint8_t START_SCAN_SAMPLES = 3;

constexpr unsigned long STARTUP_SYNC_MS  = 30000UL;

void loadFanState() {
  prefs.begin("fanstate", true);

  // Kalibrierungen bleiben unabhängig davon im NVS erhalten.
  uint8_t savedSpeed = prefs.getUChar("lastSpeed", 1);
  bool savedOsc      = prefs.getBool("lastOsc", false);
  uint8_t savedMode  = prefs.getUChar("lastMode", 0);

  prefs.end();

  if(savedSpeed < 1 || savedSpeed > 3) savedSpeed = 1;
  if(savedMode > 2) savedMode = 0;

  // Der Ventilator verliert bei kompletter Netztrennung seine Bedienzustände.
  // Deshalb ist nach einem echten ESP32-Power-On der physische Fan-Zustand sicher:
  // AUS; und beim nächsten Einschalten: Speed1 / Normal / Drehen AUS.
  esp_reset_reason_t reason = esp_reset_reason();
  bool realPowerCycle =
      (reason == ESP_RST_POWERON) ||
      (reason == ESP_RST_BROWNOUT);

  fanIsOn = false;
  state.speed = 0;
  state.osc = false;
  state.mode = 0;
  state.sleepMin = 0;
  timerDeadlineMs = 0;

  if(realPowerCycle) {
    state.lastSpeed = 1;
    state.lastOsc = false;
    state.lastMode = 0;

    // Auch unsere gemerkten Bedienwerte auf den sicheren Hardware-Default setzen.
    // Speed-/Mode-KALIBRIERUNGEN werden dadurch NICHT gelöscht.
    saveFanState();
  } else {
    // Bei OTA-/Software-Neustart wurde der Ventilator selbst nicht stromlos.
    // Daher die zuletzt gemerkten Werte behalten.
    state.lastSpeed = savedSpeed;
    state.lastOsc = savedOsc;
    state.lastMode = savedMode;
  }
}

void saveFanState() {
  prefs.begin("fanstate", false);
  prefs.putUChar("lastSpeed", state.lastSpeed);
  prefs.putBool("lastOsc", state.lastOsc);
  prefs.putUChar("lastMode", state.lastMode);
  prefs.end();
}

#if ENABLE_LED_FEEDBACK
const uint8_t FB_USED_PINS[5] = {
  PIN_LED1_FB, PIN_LED2_FB, PIN_LED3_FB, PIN_LED8_FB, PIN_LED9_FB
};

struct FbStats {
  int minv;
  int maxv;
  int avg;
  int rangev;
  float abovePct;
  float belowPct;
};

struct LearnedProfile {
  float avgPct[5];
  float avgMean[5];
  bool valid;
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

// Merkmalsvektor: f = pct - shape * Mittelwert(pct). shape=0 laesst den
// Absolutpegel unveraendert, shape=1 zentriert vollstaendig auf die Form.
inline void comboFeature(const float pct[5], float shape, float out[5]) {
  float m = 0.0f;
  for(int i=0;i<5;i++) m += pct[i];
  m /= 5.0f;
  for(int i=0;i<5;i++) out[i] = pct[i] - shape * m;
}




void loadFeedbackThresholds() {
  prefs.begin("fanfb", true);
  for(int i=0;i<5;i++) {
    String key = "th" + String(i);
    fbThresholds[i] = prefs.getInt(key.c_str(), FB_DEFAULT_THR[i]);
  }
  prefs.end();
}

void saveFeedbackThreshold(int index, int value) {
  if(index < 0 || index >= 5) return;
  value = constrain(value, 0, 4095);
  fbThresholds[index] = value;

  prefs.begin("fanfb", false);
  String key = "th" + String(index);
  prefs.putInt(key.c_str(), value);
  prefs.end();
}
#endif

// Explizite Deklarationen verhindern Probleme mit der Arduino-.ino-Autoprototypisierung.
FbStats sampleFeedback(uint8_t pin, int threshold);
bool learnProfile(LearnedProfile& p);
bool learnSpeedSignature(int speedNo);

// true, solange ein manueller Lernvorgang laeuft. Verhindert, dass Analyse und
// Einzel-Lernen gleichzeitig den Ventilator steuern und messen.
bool learnBusy = false;
bool learnModeSignature(int modeNo);
float signatureDistance5(const LearnedProfile& sig, const FbStats current[5]);
void sampleCurrent5(FbStats cur[5]);
int detectProfiles(LearnedProfile profiles[3], float* confidenceOut);
int detectProfilesFromCurrent(LearnedProfile profiles[3], const FbStats cur[5], float* confidenceOut);
float normalizedMean(const float means[5], int ch);
float currentNormalizedMean(const FbStats current[5], int ch);
float signatureDistanceWeighted(const LearnedProfile& sig, const FbStats current[5], const float weights[5]);
int detectProfilesWeightedFromCurrent(LearnedProfile profiles[3], const FbStats cur[5], const float weights[5], float* confidenceOut);
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
 display:flex;flex-direction:column;justify-content:center;padding:24px 22px}
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
.seg{margin-top:10px;display:grid;grid-template-columns:repeat(3,1fr);gap:6px;background:var(--card);
 border:1px solid var(--line);border-radius:20px;padding:6px;transition:opacity .2s}
.seg.off{opacity:.4}
.seg button{border:none;background:none;color:var(--dim);font:600 14px/1 inherit;
 padding:12px 4px;border-radius:15px;cursor:pointer;transition:.18s}
.seg button.on{background:var(--air);color:#06231f}
.seg button:disabled{cursor:default}
.dial{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
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
a.panellink{display:flex;align-items:center;justify-content:space-between;gap:12px;
 background:var(--card);border:1px solid var(--line);border-radius:20px;padding:16px;
 color:var(--txt);text-decoration:none;font-weight:600;font-size:15px}
a.panellink span{color:var(--dim);font-weight:400;font-size:13px;display:block;margin-top:3px}
a.panellink i{color:var(--air);font-style:normal;font-size:20px}
.dial button.on:disabled,.seg button.on:disabled{opacity:1}
.dial button.pending,.seg button.pending,.sw.pending{animation:pend 1.1s ease-in-out infinite}
@keyframes pend{50%{opacity:.55}}
details.panel{background:var(--card);border:1px solid var(--line);border-radius:20px;padding:0 16px}
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

<div class="upload"><button class="mini" id="ledSync" type="button">Status jetzt aus LEDs übernehmen</button></div><div class="note">Der manuelle LED-Abgleich übernimmt Speed nur bei plausibler Erkennung. Breeze/Nacht werden wegen ihrer geringeren Trennschärfe nur bei ausreichend sicherem Treffer übernommen; sonst bleibt der bisherige Modus bestehen.</div>


<details class="panel" style="margin-top:14px">
<summary>ESP32 & OTA</summary>
<div class="panelbody">
 <div class="grid2">
  <div class="kv"><b>IP-Adresse</b><span id="ip">—</span></div>
  <div class="kv"><b>WLAN</b><span id="rssi">—</span></div>
  <div class="kv"><b>Uptime</b><span id="uptime">—</span></div>
  <div class="kv"><b>Build</b><span id="build">—</span></div><div class="kv"><b>Letzte Stufe</b><span id="remembered">—</span></div><div class="kv"><b>Statusquelle</b><span id="stateSource">—</span></div>
 </div>
 <form class="upload" id="otaForm">
  <input type="file" id="otaFile" accept=".bin,application/octet-stream">
  <button class="mini" type="submit">OTA installieren</button>
 </form>
 <div class="note" id="otaMsg">Spätere Firmware-Updates direkt über WLAN. Verbindungsanzeige läuft getrennt von den langsameren LED-Messungen.</div>
</div>
</details>

</section>

<section id="viewCfg" hidden>

<a class="panellink" href="/analyze">
 <div>Trennschärfe-Analyse<span>Misst alle 9 Kombinationen, optimiert Schwellen und übernimmt die Profile</span></div>
 <i>&rarr;</i>
</a>

<details class="panel">
<summary>Speed-Kalibrierung</summary>
<div class="note">Aktive Schwellwerte: <span id="thrNow">—</span><br>
Werden sie geändert, verlieren alle gelernten Profile ihre Gültigkeit und müssen
neu aufgenommen werden — egal ob einzeln oder über die Trennschärfe-Analyse.</div>
<div class="panelbody">
 <div class="note">Ventilator auf die gewünschte Stufe stellen und dann „lernen“ drücken. Die Messung dauert etwa 45–55 Sekunden und wird danach dauerhaft gespeichert. Beim Ein-/Ausschalten drückt der ESP32 nur die Power-Taste; die zuletzt verwendete Stufe stellt der Ventilator selbst wieder her.</div>
 
 <div class="grid2" style="margin-top:10px">
  <div class="kv"><b>Speed 1</b><span id="cal1">nicht gelernt</span></div>
  <div class="kv"><b>Speed 2</b><span id="cal2">nicht gelernt</span></div>
  <div class="kv"><b>Speed 3</b><span id="cal3">nicht gelernt</span></div>
  <div class="kv"><b>Aus-Zustand</b><span id="calOff">nicht gelernt</span></div>
  <div class="kv"><b>Kombiprofile</b><span id="calCombo">nicht gelernt</span></div>
  <div class="kv"><b>Erkannt</b><span id="detSpeed">—</span></div>
  <div class="kv"><b>Sicherheit</b><span id="detConf">—</span></div>
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
 Ventilator wirklich ausschalten, dann lernen — sonst erkennt „Status aus LEDs übernehmen" den
 Aus-Zustand nicht und landet auf Stufe 1.</div>
 <div class="upload">
  <button class="mini" id="learnOff" type="button">Aus-Zustand lernen</button>
 </div>
 
 
 <div class="note" style="margin-top:14px"><b style="color:var(--txt)">Modus lernen</b> — Normal, Breeze und Night werden über LED8/LED9 plus das gemeinsame Multiplexmuster gelernt.</div>
 
 <div class="grid2" style="margin-top:10px">
  <div class="kv"><b>Normal</b><span id="calNormal">nicht gelernt</span></div>
  <div class="kv"><b>Breeze</b><span id="calBreeze">nicht gelernt</span></div>
  <div class="kv"><b>Night</b><span id="calNight">nicht gelernt</span></div>
  <div class="kv"><b>Modus erkannt</b><span id="detMode">—</span></div>
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
 
 <div class="note"><b style="color:var(--txt)">Finale Messlogik:</b> AUS wird separat erkannt. Speed/Modus verwenden bei vollständiger Kalibrierung die 9 Kombinationen aus Stufe × Modus. Unsichere Breeze/Nacht-Treffer überschreiben den bekannten Modus nicht.</div><div class="note"><b style="color:var(--txt)">Startabgleich:</b> Nach Power-EIN werden die ersten 3 Sekunden ignoriert. Danach muss derselbe Speed dreimal konsistent erkannt werden. Der Startscan endet spätestens nach 30 Sekunden und läuft nur einmal. Spätere Messungen ändern die Hauptbedienung nicht automatisch; dafür gibt es den manuellen Status-Abgleich.</div><div class="note"><b style="color:var(--txt)">Erkennung:</b> Speed nutzt hauptsächlich LED1–3, der Modus hauptsächlich LED8/9. Die Kalibrierung arbeitet mit längeren Messfenstern und normalisierten Kanalverhältnissen.</div><div class="note" id="calMsg">Noch nichts gelernt.</div>
</div>
</details>




<details class="panel">
<summary>Backup & Wiederherstellung</summary>
<div class="panelbody">
 <div class="note">Exportiert interne Messparameter, gelernte Speed-/Mode-Profile und die zuletzt gemerkten Einstellungen als JSON-Datei. Nach einem Firmware-Update kannst du die Datei wieder importieren.</div>
 <div class="upload">
  <button class="mini" id="backupExport" type="button">Backup exportieren</button>
  <input type="file" id="backupFile" accept=".json,application/json">
  <button class="mini" id="backupImport" type="button">Backup importieren</button>
 </div>
 <div class="note" id="backupMsg">Noch kein Backup importiert/exportiert.</div>
</div>
</details>


</section>

<div class="warn">GPIO-Belegung &mdash; Tasten: T1/Power = GP6, T2/Speed = GP5, T3/Drehen = GP4, T5/Modus = GP3. LED-Feedback: Speed 1 = GP7, Speed 2 = GP8, Speed 3 = GP9, Breeze = GP10, Nacht = GP1.</div>
</div>
<script>
let S={speed:0,osc:false,sleep:0,mode:0},busy=false,tick=null,restMin=0,online=false,calibrating=false;
const MODE=['Normal','Breeze','Nacht'],MKEY=['normal','breeze','night'];
const $=q=>document.querySelector(q);
const slp=$('#slp'), art=$('#fanart');

 const fmt=(m)=>{
 if(m<=0)return'Aus';
 if(m<60)return m+'<em>min</em>';
 return Math.floor(m/60)+':'+String(m%60).padStart(2,'0')+'<em>h</em>';
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
 const live=S.speed>0;
 $('#tcard').classList.toggle('off',!live);
 slp.disabled=!live||busy;
 $('#tval').classList.toggle('on',S.sleep>0);
 $('#tval').innerHTML=fmt(S.sleep);
 $('#trest').textContent=!live?'Ventilator ist aus':(S.sleep>0?'Schaltet in '+S.sleep+' min ab':'Läuft ohne Abschaltung');
 paintSlider();
}
 const draw=()=>{
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
// Nach einem Befehl braucht die LED-Erkennung einen Moment. Bis dahin darf
// pollStatus den gerade gesetzten Zustand nicht wieder ueberschreiben,
// sonst springt die Anzeige zurueck und wirkt traege.
let holdStateUntil=0;

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
 try{
  const ctl=new AbortController();
  const to=setTimeout(()=>ctl.abort(),12000);
  const r=await fetch('/api/status',{signal:ctl.signal,cache:'no-store'});
  clearTimeout(to);
  if(!r.ok)throw new Error('HTTP '+r.status);
  const x=await r.json();
  if(x.state && Date.now()>=holdStateUntil){
    const powerOn=!!x.state.powerOn;
    S.speed=powerOn?(+x.state.speed||1):0;
    S.osc=powerOn?!!x.state.osc:false;
    S.mode=powerOn?(+x.state.mode||0):0;
    S.sleep=+x.state.sleep||0;
    const rem=$('#remembered'); if(rem) rem.textContent='Speed '+(x.state.lastSpeed||1);
    const ss=$('#stateSource');
    if(ss){
      if(x.state.startupSync){
        if(!x.state.startupReady) ss.textContent='Start · LEDs beruhigen sich';
        else ss.textContent='Einmaliger Startscan '+(x.state.startupSpeedHits||0)+'/3';
      } else ss.textContent='Web / gespeicherter Zustand';
    }
  }
  if(x.thresholds && $('#thrNow')){
    $('#thrNow').textContent=x.thresholds.join(' · ');
  }
  if(x.offLearned!==undefined && $('#calOff')){
    $('#calOff').textContent = x.offLearned ? '✓ gelernt' : 'nicht gelernt';
  }
  if(x.comboLearned!==undefined && $('#calCombo')){
    $('#calCombo').textContent = x.comboLearned ? '✓ 9 von 9 · aktiv' : 'unvollständig';
  }
  if(x.calibration){
    $('#cal1').textContent=x.calibration.s1?'✓ gelernt':'nicht gelernt';
    $('#cal2').textContent=x.calibration.s2?'✓ gelernt':'nicht gelernt';
    $('#cal3').textContent=x.calibration.s3?'✓ gelernt':'nicht gelernt';
    $('#detSpeed').textContent=x.calibration.detected?('Speed '+x.calibration.detected+(S.speed===0?' (bei AUS ignoriert)':'')):'—';
    $('#detConf').textContent=x.calibration.detected?(Number(x.calibration.confidence).toFixed(0)+' %'):'—';
    if(x.calibration.detectedOff){
      $('#detSpeed').textContent='Aus';
      $('#detConf').textContent='LEDs sehr niedrig';
    }
    $('#calNormal').textContent=x.calibration.normal?'✓ gelernt':'nicht gelernt';
    $('#calBreeze').textContent=x.calibration.breeze?'✓ gelernt':'nicht gelernt';
    $('#calNight').textContent=x.calibration.night?'✓ gelernt':'nicht gelernt';
    const mn=['Normal','Breeze','Night'];
    $('#detMode').textContent=x.calibration.detectedMode>=0?mn[x.calibration.detectedMode]:'—';
    $('#detModeConf').textContent=x.calibration.detectedMode>=0?(Number(x.calibration.modeConfidence).toFixed(0)+' %'):'—';
  }
  draw();
 }catch(e){}
}
 const post=async (path,obj)=>{
 return api(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj||{})});
}
let queued=null;
 const run=async (action,optimistic=null)=>{
 // Waehrend ein Befehl laeuft, wird der letzte Klick gemerkt statt verworfen.
 if(busy){ queued={action,optimistic}; return; }
 if(optimistic){
  try{optimistic();}catch(e){}
 }
 busy=true;
 holdStateUntil=Date.now()+4000;
 draw();
 try{
  await action();
 }catch(e){
  console.log(e);
 }finally{
  busy=false;
  // Ab jetzt noch 1,5 s Ruhe, damit die Erkennung nachziehen kann.
  holdStateUntil=Date.now()+1500;
  draw();
  const q=queued; queued=null;
  if(q){ run(q.action,q.optimistic); }
  else { setTimeout(pollStatus,1600); }
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
      }else{
        // Direkt eine plausible Anzeige setzen; der Startscan korrigiert sie,
        // sobald der reale Zustand erkannt wurde.
        const rememberedText=$('#remembered')?.textContent||'';
        const mm=rememberedText.match(/(\d+)/);
        S.speed=mm?Number(mm[1]):1;
        if(S.speed<1||S.speed>3)S.speed=1;
      }
    }
   );
   return;
 }

 if(t===S.speed)return;

 run(
   ()=>post('/api/speed',{target:t}),
   ()=>{S.speed=t;}
 );
});
document.querySelectorAll('#modes button').forEach(b=>b.onclick=()=>{
 const t=+b.dataset.m;
 if(!S.speed||t===S.mode)return;

 run(
   ()=>post('/api/mode',{target:t}),
   ()=>{S.mode=t;}
 );
});
$('#osc').onclick=()=>{if(S.speed)run(()=>post('/api/osc',{}),()=>{S.osc=!S.osc;});};
slp.oninput=()=>{S.sleep=+slp.value;paintTimer();};
slp.onchange=()=>run(()=>post('/api/timer',{minutes:+slp.value}));

$('#ledSync').onclick=async()=>{
 const btn=$('#ledSync');
 const old=btn.textContent;
 btn.textContent='Messe LEDs…';
 try{
  const r=await fetch('/api/sync-from-leds',{
   method:'POST',
   headers:{'Content-Type':'application/json'},
   body:'{}'
  });
  const t=await r.text();
  if(!r.ok) throw new Error(t||('HTTP '+r.status));
  const x=JSON.parse(t);

  // Ergebnis SOFORT in die Hauptoberfläche übernehmen.
  if(x.detectedOff){
    S.speed=0;
    S.osc=false;
    S.mode=0;
    S.sleep=0;
    btn.textContent='Erkannt: Ventilator aus';
  }else{
    if(x.speedApplied!==false) S.speed=Number(x.speed)||S.speed||1;
    if(x.modeApplied===true) S.mode=Number(x.mode)||0;

    if(x.modeApplied===false)
      btn.textContent='Speed übernommen · Modus unsicher';
    else if(x.speedApplied===false)
      btn.textContent='Messung unsicher';
    else
      btn.textContent='Status übernommen';
  }
  draw();

  setTimeout(()=>{btn.textContent=old;},1800);
  setTimeout(pollStatus,300);
 }catch(e){
  btn.textContent='Abgleich fehlgeschlagen';
  setTimeout(()=>{btn.textContent=old;},1800);
 }
};





 const learnSpeed=async (n)=>{
 calibrating=true;
 const msg=$('#calMsg');
 msg.textContent='Lerne Speed '+n+'… ca. 45–55 Sekunden, bitte nicht umschalten.';
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
  b.textContent = r&&r.ok ? 'Aus-Zustand gelernt' : 'Fehlgeschlagen';
  S.speed=0;S.osc=false;S.mode=0;draw();
 }catch(e){ b.textContent='Fehler'; }
 calibrating=false;
 setTimeout(()=>{b.textContent=alt;b.disabled=false;pollStatus();},1800);
};

 const learnMode=async (n,label)=>{
 calibrating=true;
 const msg=$('#calMsg');
 msg.textContent='Lerne '+label+'… ca. 45–55 Sekunden, bitte nichts umschalten.';
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
  a.download='ventilator-backup.json';
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
  msg.textContent='Backup erfolgreich importiert.';
  setTimeout(pollStatus,300);
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
  // Alle Panels zugeklappt zeigen - jedes Mal.
  document.querySelectorAll('#viewCfg details.panel').forEach(d=>d.open=false);
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
setInterval(pollStatus,6000);
setTimeout(pollHealth,100);
setTimeout(pollStatus,500);
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
  if(!fanIsOn) return;

#if ENABLE_LED_FEEDBACK
  // Wurde der Ventilator am Gerät bereits ausgeschaltet, nur den Softwarestatus
  // korrigieren. Ein weiterer Power-Tastendruck würde ihn sonst wieder einschalten.
  FbStats cur[5];
  sampleCurrent5(cur);
  if(ledsLookOff(cur)) {
    Serial.println("[POWER] LEDs sagen: laeuft schon nicht mehr - nur Status korrigiert");
    fanIsOn = false;
    state.speed = 0;
    state.osc = false;
    state.mode = 0;
    cancelTimer();

    startupSyncUntilMs = 0;
    startupSyncReadyMs = 0;
    startupSpeedCandidate = 0;
    startupSpeedHits = 0;
    startupModeCandidate = -1;
    startupModeHits = 0;
    manualLedSyncRequested = false;
    startupScanDone = false;
    return;
  }
#endif

  if(state.speed >= 1 && state.speed <= 3) state.lastSpeed = state.speed;
  state.lastOsc = state.osc;
  state.lastMode = state.mode;
  saveFanState();

  Serial.printf("[POWER] AUS  (gemerkt: Stufe %u, Drehen %s, Modus %u)\n",
                state.lastSpeed, state.lastOsc?"an":"aus", state.lastMode);
  pulsePin(PIN_POWER);

  fanIsOn = false;
  state.speed = 0;
  state.osc = false;
  state.mode = 0;
  cancelTimer();

  startupSyncUntilMs = 0;
  startupSyncReadyMs = 0;
  startupSpeedCandidate = 0;
  startupSpeedHits = 0;
  startupModeCandidate = -1;
  startupModeHits = 0;
  manualLedSyncRequested = false;
  startupScanDone = false;
}

void togglePower() {
  if(!fanIsOn) {
    Serial.printf("[POWER] EIN  (erwartet: Stufe %u, Drehen %s, Modus %u)\n",
                  state.lastSpeed, state.lastOsc?"an":"aus", state.lastMode);
    pulsePin(PIN_POWER);
    fanIsOn = true;

    // Bis der einmalige Scan fertig ist, zeigen wir den zuletzt bekannten Zustand.
    state.speed = state.lastSpeed;
    state.osc   = state.lastOsc;
    state.mode  = state.lastMode;

    startupSyncReadyMs = millis() + START_SCAN_SETTLE_MS;
    startupSyncUntilMs = millis() + STARTUP_SYNC_MS;
    startupScanDone = false;

    startupSpeedCandidate = 0;
    startupSpeedHits = 0;
    startupModeCandidate = -1;
    startupModeHits = 0;
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
  target = constrain(target, 0, 3);

  if(target == 0) {
    powerOff();
    return;
  }

  // Im AUS-Zustand keine Geschwindigkeit direkt wählen.
  // Erst Power einschalten, danach kennt die Software den realen Startzustand
  // und Taste 2 kann korrekt 1 -> 2 -> 3 -> 1 geschaltet werden.
  if(!fanIsOn) {
    return;
  }

  if(target == state.speed) return;

  endStartupScan("Speed ueber Web gewaehlt");
  uint8_t presses = (target + 3 - state.speed) % 3;
  Serial.printf("[SPEED] %u -> %u  (%u Impulse)\n", state.speed, target, presses);
  if(presses) pulsePin(PIN_SPEED, presses);

  state.speed = target;
  state.lastSpeed = target;
  saveFanState();
}

void setMode(uint8_t target) {
  target = constrain(target, 0, 2);
  if(!fanIsOn) return;

  if(target == state.mode) {
    return;
  }

  // Taste 5: Normal -> Breeze -> Night -> Normal
  endStartupScan("Modus ueber Web gewaehlt");
  uint8_t presses = (target + 3 - state.mode) % 3;
  Serial.printf("[MODE] %u -> %u  (%u Impulse)\n", state.mode, target, presses);
  if(presses) pulsePin(PIN_MODE, presses);

  state.mode = target;
  state.lastMode = target;
  saveFanState();

  // Der von uns ausgelöste Tastendruck ist zunächst maßgeblich.
}

void toggleOsc() {
  if(!fanIsOn) return;
  endStartupScan("Drehen ueber Web geschaltet");
  Serial.printf("[OSC] %s -> %s\n", state.osc?"an":"aus", state.osc?"aus":"an");
  pulsePin(PIN_OSC);
  state.osc = !state.osc;
  state.lastOsc = state.osc;
  saveFanState();
}





FbStats sampleFeedback(uint8_t pin, int threshold) {
  const int samples = 480;
  long sum = 0;
  int minv = 4095;
  int maxv = 0;
  int above = 0;
  int below = 0;

  // ~120 ms Fenster. Das ist lang genug, um Multiplex-/PWM-Muster zu erwischen.
  for(int i=0;i<samples;i++) {
    int v = analogRead(pin);
    sum += v;
    if(v < minv) minv = v;
    if(v > maxv) maxv = v;
    if(v > threshold) above++;
    else below++;
    delayMicroseconds(250);
  }

  FbStats s;
  s.minv = minv;
  s.maxv = maxv;
  s.avg = (int)(sum / samples);
  s.rangev = maxv - minv;
  s.abovePct = above * 100.0f / samples;
  s.belowPct = below * 100.0f / samples;
  return s;
}



#if ENABLE_LED_FEEDBACK

void loadCalibration() {
  prefs.begin("fancal2", true);

  for(int s=0;s<3;s++) {
    String vk = "sv" + String(s);
    speedSig[s].valid = prefs.getBool(vk.c_str(), false);
    for(int ch=0;ch<5;ch++) {
      String pk = "sp" + String(s) + String(ch);
      String mk = "sm" + String(s) + String(ch);
      speedSig[s].avgPct[ch] = prefs.getFloat(pk.c_str(), 0.0f);
      speedSig[s].avgMean[ch] = prefs.getFloat(mk.c_str(), 0.0f);
    }
  }

  for(int s=0;s<3;s++) {
    String vk = "mv" + String(s);
    modeSig[s].valid = prefs.getBool(vk.c_str(), false);
    for(int ch=0;ch<5;ch++) {
      String pk = "mp" + String(s) + String(ch);
      String mk = "mm" + String(s) + String(ch);
      modeSig[s].avgPct[ch] = prefs.getFloat(pk.c_str(), 0.0f);
      modeSig[s].avgMean[ch] = prefs.getFloat(mk.c_str(), 0.0f);
    }
  }
  prefs.end();
}

void saveComboCalibration() {
  prefs.begin("fancal3", false);
  prefs.putBool("wv", comboWValid);
  prefs.putFloat("shape", comboShape);
  for(int ch=0; ch<5; ch++) prefs.putFloat(("cw"+String(ch)).c_str(), comboW[ch]);
  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++) {
      String base = String(sp) + String(md);
      prefs.putBool(("cv"+base).c_str(), comboSig[sp][md].valid);
      for(int ch=0; ch<5; ch++) {
        prefs.putFloat(("cp"+base+String(ch)).c_str(), comboSig[sp][md].avgPct[ch]);
        prefs.putFloat(("cm"+base+String(ch)).c_str(), comboSig[sp][md].avgMean[ch]);
      }
    }
  prefs.end();
}

void loadComboCalibration() {
  prefs.begin("fancal3", true);
  comboWValid = prefs.getBool("wv", false);
  comboShape  = prefs.getFloat("shape", 0.0f);
  for(int ch=0; ch<5; ch++) comboW[ch] = prefs.getFloat(("cw"+String(ch)).c_str(), 1.0f);
  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++) {
      String base = String(sp) + String(md);
      comboSig[sp][md].valid = prefs.getBool(("cv"+base).c_str(), false);
      for(int ch=0; ch<5; ch++) {
        comboSig[sp][md].avgPct[ch]  = prefs.getFloat(("cp"+base+String(ch)).c_str(), 0.0f);
        comboSig[sp][md].avgMean[ch] = prefs.getFloat(("cm"+base+String(ch)).c_str(), 0.0f);
      }
    }
  prefs.end();
}

bool comboComplete() {
  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++)
      if(!comboSig[sp][md].valid) return false;
  return true;
}

void saveOffCalibration() {
  prefs.begin("fancal2", false);
  prefs.putBool("ov", offSig.valid);
  for(int ch=0; ch<5; ch++) {
    String pk="op"+String(ch);
    String mk="om"+String(ch);
    prefs.putFloat(pk.c_str(), offSig.avgPct[ch]);
    prefs.putFloat(mk.c_str(), offSig.avgMean[ch]);
  }
  prefs.end();
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
  float sumPct[5]={0,0,0,0,0}, sumMean[5]={0,0,0,0,0};
  const int RUNS = 6;
  for(int r=0; r<RUNS; r++) {
    sampleCurrent5(cur);
    for(int ch=0; ch<5; ch++) { sumPct[ch]+=cur[ch].abovePct; sumMean[ch]+=cur[ch].avg; }
    delay(40);
  }
  for(int ch=0; ch<5; ch++) {
    offSig.avgPct[ch]  = sumPct[ch]  / RUNS;
    offSig.avgMean[ch] = sumMean[ch] / RUNS;
  }
  offSig.valid = true;
  saveOffCalibration();
  Serial.printf("[AUS] Profil gelernt: %.1f %.1f %.1f %.1f %.1f\n",
                offSig.avgPct[0],offSig.avgPct[1],offSig.avgPct[2],
                offSig.avgPct[3],offSig.avgPct[4]);
  return true;
#else
  return false;
#endif
}

void saveSpeedCalibration(int idx) {
  if(idx<0 || idx>2) return;
  prefs.begin("fancal2", false);
  String vk="sv"+String(idx);
  prefs.putBool(vk.c_str(), speedSig[idx].valid);
  for(int ch=0;ch<5;ch++) {
    String pk="sp"+String(idx)+String(ch);
    String mk="sm"+String(idx)+String(ch);
    prefs.putFloat(pk.c_str(), speedSig[idx].avgPct[ch]);
    prefs.putFloat(mk.c_str(), speedSig[idx].avgMean[ch]);
  }
  prefs.end();
}

void saveModeCalibration(int idx) {
  if(idx<0 || idx>2) return;
  prefs.begin("fancal2", false);
  String vk="mv"+String(idx);
  prefs.putBool(vk.c_str(), modeSig[idx].valid);
  for(int ch=0;ch<5;ch++) {
    String pk="mp"+String(idx)+String(ch);
    String mk="mm"+String(idx)+String(ch);
    prefs.putFloat(pk.c_str(), modeSig[idx].avgPct[ch]);
    prefs.putFloat(mk.c_str(), modeSig[idx].avgMean[ch]);
  }
  prefs.end();
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
  // sampleCurrent5() misst bereits drei synchronisierte Durchlaeufe.
  // 36 Fenster ergeben 108 zeitlich verteilte Messfenster pro Kanal.
  constexpr int WINDOWS = 36;

  float pct[5][WINDOWS];
  float meanv[5][WINDOWS];

  for(int w=0; w<WINDOWS; w++) {
    FbStats cur[5];
    sampleCurrent5(cur);
    for(int ch=0; ch<5; ch++) {
      pct[ch][w] = cur[ch].abovePct;
      meanv[ch][w] = (float)cur[ch].avg;
    }
    delay(8);
    yield();
  }

  for(int ch=0; ch<5; ch++) {
    p.avgPct[ch] = robustMedianOfMeans(pct[ch], WINDOWS);
    p.avgMean[ch] = robustMedianOfMeans(meanv[ch], WINDOWS);
  }
  p.valid = true;
  return true;
}

bool learnSpeedSignature(int speedNo) {
  if(speedNo<1 || speedNo>3) return false;
  int idx=speedNo-1;
  bool ok=learnProfile(speedSig[idx]);
  if(ok) saveSpeedCalibration(idx);
  return ok;
}

bool learnModeSignature(int modeNo) {
  if(modeNo<0 || modeNo>2) return false;
  bool ok=learnProfile(modeSig[modeNo]);
  if(ok) saveModeCalibration(modeNo);
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

void sampleCurrent5(FbStats cur[5]) {
  const uint8_t pins[5] = {
    PIN_LED1_FB, PIN_LED2_FB, PIN_LED3_FB, PIN_LED8_FB, PIN_LED9_FB
  };

  // WICHTIG fuer die gemultiplexten LEDs:
  // Alle 5 Kanaele werden innerhalb desselben Zeitfensters verschachtelt
  // gemessen. Die alte Variante hat erst LED1 komplett, dann LED2 usw.
  // gemessen; gemeinsame Pegeldrift konnte dadurch faelschlich wie ein
  // Kanalunterschied aussehen.
  constexpr int CYCLES = 480;
  constexpr int PASSES = 3;

  double sum[5] = {0,0,0,0,0};
  uint32_t above[5] = {0,0,0,0,0};
  uint32_t total[5] = {0,0,0,0,0};
  int minv[5] = {4095,4095,4095,4095,4095};
  int maxv[5] = {0,0,0,0,0};

  for(int pass=0; pass<PASSES; pass++) {
    for(int i=0; i<CYCLES; i++) {
      // Startkanal rotieren: kein Kanal sitzt dauerhaft an derselben
      // Position innerhalb eines Multiplex-Zyklus.
      int first = (i + pass) % 5;
      for(int j=0; j<5; j++) {
        int ch = (first + j) % 5;

        // Nach ADC-MUX-Wechsel eine Probe verwerfen. Das reduziert
        // Kanal-zu-Kanal-Uebersprechen des Sample/Hold-Kondensators.
        (void)analogRead(pins[ch]);
        int v = analogRead(pins[ch]);

        if(v < 0) v = 0;
        if(v > 4095) v = 4095;

        sum[ch] += v;
        total[ch]++;
        if(v > fbThresholds[ch]) above[ch]++;
        if(v < minv[ch]) minv[ch] = v;
        if(v > maxv[ch]) maxv[ch] = v;
      }
      delayMicroseconds(250);
    }
    yield();
  }

  for(int ch=0; ch<5; ch++) {
    const float n = total[ch] ? (float)total[ch] : 1.0f;
    cur[ch].avg = (int)(sum[ch] / n + 0.5);
    cur[ch].abovePct = above[ch] * 100.0f / n;
    cur[ch].belowPct = 100.0f - cur[ch].abovePct;
    cur[ch].minv = minv[ch];
    cur[ch].maxv = maxv[ch];
    cur[ch].rangev = maxv[ch] - minv[ch];
  }
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
  sampleCurrent5(cur);
  bool measuredOff = ledsLookOff(cur);
  const float equalWeights[5] = {1,1,1,1,1};
  return detectProfilesWeightedFromCurrent(profiles,cur,equalWeights,confidenceOut);
}

// Klassifiziert Stufe UND Modus in einem Schritt gegen alle neun Profile.
// Liefert -1, wenn nicht alle neun gelernt sind.
int detectCombo(const FbStats cur[5], int* speedOut, int* modeOut,
                float* speedConfOut, float* modeConfOut) {
  if(speedOut)     *speedOut = 0;
  if(modeOut)      *modeOut  = -1;
  if(speedConfOut) *speedConfOut = 0.0f;
  if(modeConfOut)  *modeConfOut  = 0.0f;
  if(!comboComplete()) return -1;

  float w[5];
  const float fallbackW[5] = {1.10f,0.75f,1.26f,0.76f,1.12f};
  for(int i=0;i<5;i++) w[i] = comboWValid ? comboW[i] : fallbackW[i];

  float curPct[5], curF[5];
  for(int i=0;i<5;i++) curPct[i] = cur[i].abovePct;
  comboFeature(curPct, comboShape, curF);

  // Zweites, weitgehend unabhaengiges Merkmal:
  // normierter ADC-Mittelwert je Kanal. Die Normierung entfernt gemeinsame
  // Versorgungsschwankungen; feine Unterschiede in der Verteilung bleiben.
  float curMeanSum = 0.0f;
  for(int ch=0;ch<5;ch++) curMeanSum += cur[ch].avg;
  if(curMeanSum < 1.0f) curMeanSum = 1.0f;

  const float MEAN_MIX =
      constrain(0.35f * (1.0f - comboShape), 0.0f, 0.35f);
  // Bei comboShape=1.00 ist MEAN_MIX=0: exakt der von der Analyse
  // gefundene reine Form-Klassifikator wird verwendet.

  float dist[3][3];
  float best=1e30f;
  int bs=-1, bm=-1;

  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++) {
      float profF[5];
      comboFeature(comboSig[sp][md].avgPct, comboShape, profF);

      float profMeanSum = 0.0f;
      for(int ch=0;ch<5;ch++) profMeanSum += comboSig[sp][md].avgMean[ch];
      if(profMeanSum < 1.0f) profMeanSum = 1.0f;

      float d = 0.0f;
      for(int ch=0; ch<5; ch++) {
        float ePct = curF[ch] - profF[ch];

        // Prozentualer Anteil am Gesamt-ADC-Pegel, also robust gegen
        // globale Helligkeits-/Versorgungsdrift.
        float cNorm = cur[ch].avg * 100.0f / curMeanSum;
        float pNorm = comboSig[sp][md].avgMean[ch] * 100.0f / profMeanSum;
        float eMean = cNorm - pNorm;

        d += w[ch] * (ePct*ePct + MEAN_MIX*eMean*eMean);
      }
      dist[sp][md] = d;
      if(d < best) { best=d; bs=sp; bm=md; }
    }

  if(bs<0) return -1;

  float bestOtherSpeed = 1e30f, bestOtherMode = 1e30f;
  for(int sp=0; sp<3; sp++)
    for(int md=0; md<3; md++) {
      if(sp != bs && dist[sp][md] < bestOtherSpeed) bestOtherSpeed = dist[sp][md];
      if(md != bm && dist[sp][md] < bestOtherMode)  bestOtherMode  = dist[sp][md];
    }

  float quality = 1.0f / (1.0f + sqrtf(best) / 90.0f);
  auto conf = [&](float other) -> float {
    if(other > 1e29f) return 0.0f;
    float margin = 1.0f - (best / (other + 0.0001f));
    if(margin < 0) margin = 0;
    if(margin > 1) margin = 1;
    float c = margin * quality;
    return constrain(c,0.0f,1.0f) * 100.0f;
  };

  if(speedConfOut) *speedConfOut = conf(bestOtherSpeed);
  if(modeConfOut)  *modeConfOut  = conf(bestOtherMode);
  if(speedOut) *speedOut = bs + 1;
  if(modeOut)  *modeOut  = bm;
  return bs + 1;
}

// Ergebnis der letzten Kombi-Klassifikation, damit Stufe und Modus aus
// DERSELBEN Messung stammen - und die 600 ms Messzeit nicht doppelt anfallen.
struct ComboCache { unsigned long at=0; int speed=0; int mode=-1; float sc=0, mc=0; bool ok=false; };
ComboCache comboCache;

bool detectComboCached(int* sp, int* md, float* sc, float* mc) {
  if(!comboComplete()) return false;
  if(comboCache.ok && (millis() - comboCache.at) < 400UL) {
    if(sp) *sp=comboCache.speed; if(md) *md=comboCache.mode;
    if(sc) *sc=comboCache.sc;    if(mc) *mc=comboCache.mc;
    return true;
  }
  FbStats cur[5];
  sampleCurrent5(cur);
  int s2=0, m2=-1; float c1=0, c2=0;
  int r = detectCombo(cur, &s2, &m2, &c1, &c2);
  if(r <= 0) { comboCache.ok=false; return false; }
  comboCache = { millis(), s2, m2, c1, c2, true };
  if(sp) *sp=s2; if(md) *md=m2; if(sc) *sc=c1; if(mc) *mc=c2;
  return true;
}

int detectLearnedSpeed(float* confidenceOut) {
  FbStats cur[5];
  sampleCurrent5(cur);

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
  sampleCurrent5(cur);

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
  float currentTotal = 0.0f;
  for(int ch=0; ch<5; ch++) currentTotal += cur[ch].avg;

  // Find the weakest valid learned ON/Speed profile.
  float minLearnedOnTotal = 1e30f;
  bool haveSpeedProfile = false;

  for(int s=0; s<3; s++) {
    if(!speedSig[s].valid) continue;
    float total = 0.0f;
    for(int ch=0; ch<5; ch++) total += speedSig[s].avgMean[ch];
    if(total < minLearnedOnTotal) minLearnedOnTotal = total;
    haveSpeedProfile = true;
  }

  // Bevorzugt: gemessenes AUS-Profil. Entscheidet nach Aehnlichkeit statt nach
  // einer festen Prozentgrenze - der einzige Weg, der bei gemultiplexten LEDs
  // zuverlaessig funktioniert.
  if(offSig.valid) {
    // Gleiche Gewichtung wie bei der Klassifikation, damit "aus" und
    // "laeuft" nach demselben Massstab verglichen werden.
    float w[5];
    for(int i=0;i<5;i++) w[i] = comboWValid ? comboW[i] : 1.0f;

    float dOff = 0.0f;
    for(int ch=0; ch<5; ch++) {
      float d = cur[ch].abovePct - offSig.avgPct[ch];
      dOff += w[ch] * d * d;
    }

    float dOn = 1e30f;
    // Kombiprofile zuerst - sie sind die genaueren Ein-Referenzen.
    for(int sp=0; sp<3; sp++)
      for(int md=0; md<3; md++) {
        if(!comboSig[sp][md].valid) continue;
        float d2 = 0.0f;
        for(int ch=0; ch<5; ch++) {
          float d = cur[ch].abovePct - comboSig[sp][md].avgPct[ch];
          d2 += w[ch] * d * d;
        }
        if(d2 < dOn) dOn = d2;
      }
    // Rueckfall auf die alten Stufenprofile
    if(dOn > 1e29f) {
      for(int s=0; s<3; s++) {
        if(!speedSig[s].valid) continue;
        float d2 = 0.0f;
        for(int ch=0; ch<5; ch++) {
          float d = cur[ch].abovePct - speedSig[s].avgPct[ch];
          d2 += w[ch] * d * d;
        }
        if(d2 < dOn) dOn = d2;
      }
    }

    // Kein einziges Ein-Profil gelernt: dann darf das AUS-Profil allein
    // nicht "immer aus" bedeuten. Stattdessen nur bei echter Naehe.
    if(dOn > 1e29f) {
      float mean = 0.0f;
      for(int ch=0; ch<5; ch++) mean += w[ch];
      return dOff < 25.0f * mean;      // ca. 5 Prozentpunkte je Kanal
    }
    return dOff <= dOn;
  }

  // Ohne gelerntes AUS-Profil: alte Faustregel als Rueckfallebene.
  if(haveSpeedProfile) {
    return currentTotal < (minLearnedOnTotal * 0.35f);
  }

  // Fallback before calibration: intentionally conservative.
  return currentTotal < 1200.0f;
#else
  return false;
#endif
}

void applySmartDetectedState(int learnedSpeed, float speedConf,
                             int learnedMode, float modeConf) {
#if ENABLE_LED_FEEDBACK
  if(!fanIsOn) return;

  // Manueller Abgleich: bewusst angefordert, beste gelernte Klassifikation
  // direkt übernehmen.
  if(manualLedSyncRequested) {
    if(learnedSpeed >= 1 && learnedSpeed <= 3) {
      state.speed = learnedSpeed;
      state.lastSpeed = learnedSpeed;
    }
    if(learnedMode >= 0 && learnedMode <= 2) {
      state.mode = learnedMode;
      state.lastMode = learnedMode;
    }
    saveFanState();
    manualLedSyncRequested = false;
    return;
  }

  // Nach abgeschlossenem Startscan verändert laufende LED-Erkennung
  // die Hauptbedienung nicht mehr automatisch.
  if(startupScanDone) return;

  // Zeitfenster abgelaufen: gespeicherten Zustand behalten.
  if(startupSyncUntilMs == 0 || !smartTimeActive(startupSyncUntilMs)) {
    startupScanDone = true;
    startupSyncUntilMs = 0;
    startupSyncReadyMs = 0;
    return;
  }

  // Direkt nach Power-EIN zunächst 3 Sekunden ignorieren.
  if(startupSyncReadyMs == 0 || (long)(millis() - startupSyncReadyMs) < 0) return;

  if(learnedSpeed >= 1 && learnedSpeed <= 3 && speedConf >= 12.0f) {
    if(startupSpeedCandidate == learnedSpeed) {
      if(startupSpeedHits < 20) startupSpeedHits++;
    } else {
      startupSpeedCandidate = learnedSpeed;
      startupSpeedHits = 1;
    }
  } else {
    startupSpeedCandidate = 0;
    startupSpeedHits = 0;
  }

  if(learnedMode >= 0 && learnedMode <= 2 && modeConf >= 20.0f) {
    if(startupModeCandidate == learnedMode) {
      if(startupModeHits < 20) startupModeHits++;
    } else {
      startupModeCandidate = learnedMode;
      startupModeHits = 1;
    }
  } else {
    startupModeCandidate = -1;
    startupModeHits = 0;
  }

  // Mit dem 6-s-Statusintervall sind drei konsistente Messungen im
  // 30-s-Fenster zuverlässig erreichbar.
  if(startupSpeedHits >= START_SCAN_SAMPLES) {
    state.speed = startupSpeedCandidate;
    state.lastSpeed = startupSpeedCandidate;

    if(startupModeHits >= START_SCAN_SAMPLES && startupModeCandidate >= 0) {
      state.mode = startupModeCandidate;
      state.lastMode = startupModeCandidate;
    }

    saveFanState();
    startupScanDone = true;
    startupSyncUntilMs = 0;
    startupSyncReadyMs = 0;
  }
#endif
}

void sendStatus() {
  String ip = WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  int rssi = WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : 0;

  int learnedSpeed = 0;
  int learnedMode = -1;
  float speedConf = 0.0f;
  float modeConf = 0.0f;
  bool measuredOff = false;

#if ENABLE_LED_FEEDBACK
  FbStats cur[5];
  sampleCurrent5(cur);
  measuredOff = ledsLookOff(cur);

  if(comboComplete()) {
    detectCombo(cur,&learnedSpeed,&learnedMode,&speedConf,&modeConf);
  } else {
    const float speedWeights[5] = {1.8f,1.8f,1.8f,0.08f,0.08f};
    const float modeWeights[5]  = {0.08f,0.08f,0.08f,2.2f,2.2f};

    int speedIdx = detectProfilesWeightedFromCurrent(speedSig,cur,speedWeights,&speedConf);
    int modeIdx  = detectProfilesWeightedFromCurrent(modeSig,cur,modeWeights,&modeConf);
    learnedSpeed = speedIdx < 0 ? 0 : speedIdx + 1;
    learnedMode  = modeIdx;
  }

  if(fanIsOn && !measuredOff) {
    applySmartDetectedState(learnedSpeed,speedConf,learnedMode,modeConf);
  } else if(!fanIsOn) {
    state.speed = 0;
    state.osc = false;
    state.mode = 0;
  }
#endif

  String j="{";
  j+="\"version\":\""+String(FW_VERSION)+"\",";
  j+="\"build\":\""+String(__DATE__)+" "+String(__TIME__)+"\",";
  j+="\"ip\":\""+ip+"\",";
  j+="\"rssi\":"+String(rssi)+",";
  j+="\"uptime\":"+String(millis()/1000UL)+",";
  j+="\"offLearned\":"+String(offSig.valid?"true":"false")+",";
  j+="\"comboLearned\":"+String(comboComplete()?"true":"false")+",";

  j+="\"state\":{";
  j+="\"powerOn\":"+String(fanIsOn?"true":"false")+",";
  j+="\"speed\":"+String(state.speed)+",";
  j+="\"osc\":"+String(state.osc?"true":"false")+",";
  j+="\"mode\":"+String(state.mode)+",";
  j+="\"sleep\":"+String(state.sleepMin)+",";
  j+="\"lastSpeed\":"+String(state.lastSpeed)+",";
  j+="\"lastOsc\":"+String(state.lastOsc?"true":"false")+",";
  j+="\"lastMode\":"+String(state.lastMode)+",";
  j+="\"startupSync\":"+String(smartTimeActive(startupSyncUntilMs)?"true":"false")+",";
  j+="\"startupReady\":"+String((startupSyncReadyMs!=0 && (long)(millis()-startupSyncReadyMs)>=0)?"true":"false")+",";
  j+="\"startupSpeedHits\":"+String(startupSpeedHits)+",";
  j+="\"startupModeHits\":"+String(startupModeHits);
  j+="},";

#if ENABLE_LED_FEEDBACK
  j+="\"calibration\":{";
  j+="\"s1\":"+String(speedSig[0].valid?"true":"false")+",";
  j+="\"s2\":"+String(speedSig[1].valid?"true":"false")+",";
  j+="\"s3\":"+String(speedSig[2].valid?"true":"false")+",";
  j+="\"normal\":"+String(modeSig[0].valid?"true":"false")+",";
  j+="\"breeze\":"+String(modeSig[1].valid?"true":"false")+",";
  j+="\"night\":"+String(modeSig[2].valid?"true":"false")+",";
  j+="\"detected\":"+String(learnedSpeed)+",";
  j+="\"confidence\":"+String(speedConf,1)+",";
  j+="\"detectedMode\":"+String(learnedMode)+",";
  j+="\"modeConfidence\":"+String(modeConf,1)+",";
  j+="\"detectedOff\":"+String(measuredOff?"true":"false");
  j+="}";
#else
  j+="\"calibration\":{\"s1\":false,\"s2\":false,\"s3\":false,\"normal\":false,\"breeze\":false,\"night\":false,\"detected\":0,\"confidence\":0,\"detectedMode\":-1,\"modeConfidence\":0,\"detectedOff\":false}";
#endif

  j+="}";
  server.send(200,"application/json",j);
}

String buildBackupJson() {
  String j="{";
  j+="\"format\":1,";
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
  for(int ch=0;ch<5;ch++) { if(ch) j+=","; j+=String(comboW[ch],4); }
  j+="]},";

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
#else
  j+="\"thresholds\":[],\"speedProfiles\":[],\"modeProfiles\":[],\"offProfile\":{},\"comboProfiles\":[]";
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

bool applyBackupJson(const String& body, String& error) {
  int format=0;
  if(!extractJsonInt(body,"format",format) || format!=1) {
    error="Unsupported or missing backup format";
    return false;
  }

  // Fan state
  String fanObj;
  int fanPos=body.indexOf("\"fan\":{");
  if(fanPos>=0) {
    int start=body.indexOf('{',fanPos);
    int depth=0, end=-1;
    for(int i=start;i<body.length();i++) {
      if(body[i]=='{') depth++;
      else if(body[i]=='}') {
        depth--;
        if(depth==0) { end=i; break; }
      }
    }
    if(end>start) fanObj=body.substring(start,end+1);
  }

  if(fanObj.length()) {
    int v;
    bool b;
    if(extractJsonInt(fanObj,"lastSpeed",v)) state.lastSpeed=constrain(v,1,3);
    if(extractJsonBool(fanObj,"lastOsc",b)) state.lastOsc=b;
    if(extractJsonInt(fanObj,"lastMode",v)) state.lastMode=constrain(v,0,2);
    saveFanState();
  }

#if ENABLE_LED_FEEDBACK
  int th[5];
  if(extractIntArray(body,"thresholds",th,5)) {
    for(int i=0;i<5;i++) {
      fbThresholds[i]=constrain(th[i],0,4095);
      saveFeedbackThreshold(i,fbThresholds[i]);
    }
  }

  for(int s=0;s<3;s++) {
    String obj;
    if(extractObjectArrayItem(body,"speedProfiles",s,obj)) {
      bool valid=false;
      extractJsonBool(obj,"valid",valid);
      float pct[5], mean[5];
      if(extractNumberArray(obj,"pct",pct,5) && extractNumberArray(obj,"mean",mean,5)) {
        for(int ch=0;ch<5;ch++) {
          speedSig[s].avgPct[ch]=pct[ch];
          speedSig[s].avgMean[ch]=mean[ch];
        }
        speedSig[s].valid=valid;
        saveSpeedCalibration(s);
      }
    }
  }

  for(int s=0;s<3;s++) {
    String obj;
    if(extractObjectArrayItem(body,"modeProfiles",s,obj)) {
      bool valid=false;
      extractJsonBool(obj,"valid",valid);
      float pct[5], mean[5];
      if(extractNumberArray(obj,"pct",pct,5) && extractNumberArray(obj,"mean",mean,5)) {
        for(int ch=0;ch<5;ch++) {
          modeSig[s].avgPct[ch]=pct[ch];
          modeSig[s].avgMean[ch]=mean[ch];
        }
        modeSig[s].valid=valid;
        saveModeCalibration(s);
      }
    }
  }

  // AUS-Profil
  {
    int p = body.indexOf("\"offProfile\"");
    if(p >= 0) {
      int a = body.indexOf('{', p);
      int b = body.indexOf('}', a);
      if(a > 0 && b > a) {
        String obj = body.substring(a, b+1);
        bool valid=false;
        extractJsonBool(obj,"valid",valid);
        float pct[5], mean[5];
        if(extractNumberArray(obj,"pct",pct,5) && extractNumberArray(obj,"mean",mean,5)) {
          for(int ch=0;ch<5;ch++) { offSig.avgPct[ch]=pct[ch]; offSig.avgMean[ch]=mean[ch]; }
          offSig.valid=valid;
          saveOffCalibration();
        }
      }
    }
  }

  // Kanalgewichte
  {
    int p = body.indexOf("\"comboWeights\"");
    if(p >= 0) {
      int a = body.indexOf('{', p);
      int b = body.indexOf('}', a);
      if(a > 0 && b > a) {
        String obj = body.substring(a, b+1);
        bool valid=false;
        extractJsonBool(obj,"valid",valid);
        float w[5];
        if(extractNumberArray(obj,"w",w,5)) {
          for(int ch=0;ch<5;ch++) comboW[ch]=w[ch];
          comboWValid=valid;
        }
      }
    }
    int q = body.indexOf("\"comboShape\"");
    if(q >= 0) {
      float sh = body.substring(body.indexOf(':', q)+1).toFloat();
      if(sh >= 0.0f && sh <= 1.0f) comboShape = sh;
    }
  }

  // Neun Kombiprofile
  {
    bool any=false;
    for(int i=0;i<9;i++) {
      String obj;
      if(!extractObjectArrayItem(body,"comboProfiles",i,obj)) continue;
      bool valid=false;
      extractJsonBool(obj,"valid",valid);
      float pct[5], mean[5];
      if(extractNumberArray(obj,"pct",pct,5) && extractNumberArray(obj,"mean",mean,5)) {
        int sp=i/3, md=i%3;
        for(int ch=0;ch<5;ch++) {
          comboSig[sp][md].avgPct[ch]=pct[ch];
          comboSig[sp][md].avgMean[ch]=mean[ch];
        }
        comboSig[sp][md].valid=valid;
        any=true;
      }
    }
    if(any) saveComboCalibration();
    else    saveComboCalibration();   // Gewichte in jedem Fall sichern
  }
#endif

  error="";
  return true;
}


static const char ANALYZE_PAGE[] PROGMEM = R"ANA(<!DOCTYPE html><html lang="de"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0d1116"><title>Trennschaerfe</title>
<style>
:root{--bg:#0d1116;--card:#161c24;--line:#2b3644;--txt:#e8edf3;--dim:#8496ab;
--air:#4fd1c5;--bad:#e8624a;--warn:#f6a35c;--good:#5cd67f}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);padding:22px 16px 60px;
font:400 16px/1.5 ui-rounded,-apple-system,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:760px;margin:0 auto}
h1{font-size:20px;margin:0 0 2px}
h2{font-size:12px;letter-spacing:.18em;text-transform:uppercase;color:var(--dim);margin:26px 0 10px;font-weight:600}
p{color:var(--dim);font-size:14px;margin:8px 0}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px}
button{background:var(--air);color:#06231f;border:none;border-radius:12px;padding:12px 18px;font:600 15px inherit}
button:disabled{opacity:.45}
a.back{color:var(--dim);font-size:14px;text-decoration:none}
table{width:100%;border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}
th,td{text-align:right;padding:7px 5px;border-bottom:1px solid var(--line)}
th:first-child,td:first-child{text-align:left;color:var(--dim)}
th{color:var(--dim);font-weight:600;font-size:11px;text-transform:uppercase}
.sd{color:var(--dim);font-size:11px}
.v{border-radius:14px;padding:11px 13px;margin:8px 0;font-size:14px;border:1px solid}
.v b{display:block;font-size:15px;margin-bottom:2px}
.g{border-color:#1f4d33;background:#0f2318;color:#bdf0cf}
.w{border-color:#5a4420;background:#241d10;color:#f2ddb0}
.b{border-color:#5a2320;background:#241110;color:#f2c0b8}
.bar{height:6px;border-radius:3px;background:var(--line);overflow:hidden;margin-top:14px}
.bar i{display:block;height:100%;background:var(--air);transition:width .4s}
.act{background:#0d1116;border:1px solid var(--line);border-left:3px solid var(--air);
 border-radius:10px;padding:10px 12px;color:var(--txt);font-size:14px;margin-top:10px}
.act b{color:var(--air);display:block;font-size:11px;letter-spacing:.14em;
 text-transform:uppercase;margin-bottom:3px}
</style></head><body><div class="wrap">
<a class="back" href="/#einstellungen">&larr; zurueck zur Steuerung</a>
<h1>Trennschaerfe-Analyse</h1>
<p>Startet immer aus dem AUS-Zustand, schaltet ein (Stufe 1 / Normal) und faehrt von dort alle 9 Kombinationen dreimal in fester Reihenfolge an. Stufenwechsel erfolgen immer im Normal-Modus. Am Ende wird Stufe 1 / Normal zur Kontrolle erneut gemessen. Dauer ca. 9 Minuten.</p>
<div class="card">
<button id="go">Messlauf starten</button>
<button id="stop" hidden style="margin-left:8px;background:var(--bad);color:#fff">Kalibrierung stoppen</button>
<button id="apply" hidden style="margin-left:8px">Als Kalibrierung uebernehmen</button>
<div class="bar" id="barwrap" hidden><i id="bar" style="width:0%"></i></div>
<p id="stat"></p>
<p id="act" class="act" hidden></p>
</div>
<p style="font-size:13px">Die Uebernahme mittelt die Stufen-Profile ueber alle Modi und die
Modus-Profile ueber alle Stufen. Vorher lohnt ein Blick auf den Drift-Abschnitt unten:
Driftet eine Modus-Signatur stark, ist Mitteln der falsche Weg.</p>
<div id="out"></div>
<script>
const MODE=['Normal','Breeze','Nacht'];
const CH=['LED1','LED2','LED3','LED8','LED9'];
const $=q=>document.querySelector(q);
let poll=null;

 const mean=(a)=>{return a.reduce((x,y)=>x+y,0)/a.length}
 const sd=(a)=>{const m=mean(a);return Math.sqrt(a.reduce((s,v)=>s+(v-m)*(v-m),0)/a.length)}

$('#stop').onclick=async()=>{
 $('#stop').disabled=true;
 try{ await fetch('/api/analyze/stop',{method:'POST'}); }catch(e){}
};
$('#go').onclick=async()=>{
 $('#go').disabled=true;$('#stop').hidden=false;$('#stop').disabled=false;
 $('#out').innerHTML='';$('#barwrap').hidden=false;
 try{
  const r=await fetch('/api/analyze/start',{method:'POST'});
  if(!r.ok){const e=await r.json();$('#stat').textContent='Fehler: '+(e.error||'unbekannt');$('#go').disabled=false;return;}
 }catch(e){$('#stat').textContent='Keine Verbindung';$('#go').disabled=false;return;}
 poll=setInterval(tick,1200);tick();
};

 const tick=async ()=>{
 let d;try{d=await(await fetch('/api/analyze')).json()}catch(e){return}
 $('#bar').style.width=d.progress+'%';
 $('#stat').textContent=d.running?('Kalibrierung: '+d.progress+' %')
   :(d.aborted?'Kalibrierung abgebrochen':(d.done?'Fertig':''));
 if(d.action){
  $('#act').hidden=false;
  $('#act').innerHTML='<b>Aktuelle Aktion</b>'+d.action;
 }
 if(!d.running){
  clearInterval(poll);poll=null;
  $('#go').disabled=false;$('#stop').hidden=true;
  if(d.aborted){
   $('#barwrap').hidden=true;
   $('#out').innerHTML='<div class="v b"><b>Kalibrierung abgebrochen</b>'+
    'Der Lauf wurde gestoppt, es wurden keine weiteren Befehle an den Ventilator '+
    'gesendet. Die bisherige Kalibrierung ist unveraendert. Ein neuer Messlauf '+
    'kann jederzeit gestartet werden.</div>';
   $('#apply').hidden=true;
  } else if(d.done){
   $('#apply').hidden=false;render(d);
  }
 }
}

$('#apply').onclick=async()=>{
 $('#apply').disabled=true;
 try{
  const r=await fetch('/api/analyze/apply',{method:'POST'});
  const j=await r.json();
  $('#stat').textContent=j.ok?'Kalibrierung uebernommen und gespeichert.':('Fehler: '+(j.error||'?'));
 }catch(e){$('#stat').textContent='Keine Verbindung';}
 $('#apply').disabled=false;
};

 const render=(d)=>{
 const C=d.combos;
 let h='';
 if(d.verifyDone){
  const v=Number(d.verifyDelta), ok=v<1.5;
  h+='<div class="v '+(ok?'g':'b')+'"><b>Schlusspruefung '+(ok?'bestanden':'FEHLGESCHLAGEN')+'</b>'+
     'Stufe 1 / Normal wurde am Ende erneut gemessen. Groesste Abweichung zum '+
     'ersten Durchgang: '+v.toFixed(2)+' Prozentpunkte. '+
     (ok?'Beim Umschalten ist nichts verrutscht, die Zuordnung stimmt.'
        :'Die Zaehlung der Modi oder Stufen ist waehrend des Laufs verrutscht. '+
         'Alle Beschriftungen sind damit unbrauchbar - bitte den Messlauf wiederholen. '+
         'Die Uebernahme ist gesperrt.')+'</div>';
 }
 h+='<h2>Profile je Kombination</h2><div class="card"><table>';
 h+='<tr><th>Zustand</th>'+CH.map(c=>'<th>'+c+'</th>').join('')+'</tr>';
 C.forEach(c=>{
  h+='<tr><td>St.'+c.speed+' / '+MODE[c.mode]+'</td>';
  c.ch.forEach(v=>{h+='<td>'+mean(v).toFixed(1)+' <span class="sd">&plusmn;'+sd(v).toFixed(1)+'</span></td>'});
  h+='</tr>';
 });
 if(d.offMeasured&&d.off){
  h+='<tr style="border-top:2px solid var(--line)"><td style="color:var(--air)">Aus</td>'+
     d.off.map(v=>'<td>'+Number(v).toFixed(1)+'</td>').join('')+'</tr>';
 }
 h+='</table></div>';
 if(d.offMeasured&&d.off){
  // Wie weit liegt AUS vom naechsten Ein-Zustand entfernt?
  let nearest=1e9;
  C.forEach(c=>{
   const dd=Math.max(...c.ch.map((v,i)=>Math.abs(mean(v)-Number(d.off[i]))));
   if(dd<nearest)nearest=dd;
  });
  const cls=nearest>=10?'g':(nearest>=4?'w':'b');
  h+='<div class="v '+cls+'"><b>Aus gegen den naechsten Ein-Zustand</b>'+
     'Abstand '+nearest.toFixed(1)+'. '+
     (nearest>=10?'Der Aus-Zustand ist klar erkennbar.'
      :nearest>=4?'Erkennbar, aber knapp. Nach der Uebernahme pruefen, ob "Status aus LEDs uebernehmen" bei ausgeschaltetem Ventilator wirklich Aus meldet.'
      :'Zu dicht dran - die Aus-Erkennung wird unzuverlaessig sein.')+
     '</div>';
 }

 // Distanz zwischen zwei Kombinationen: groesster Kanalunterschied
 const dist=(a,b)=>Math.max(...a.ch.map((v,i)=>Math.abs(mean(v)-mean(b.ch[i]))));
 const noise=(a,b)=>Math.max(...a.ch.map((v,i)=>Math.max(sd(v),sd(b.ch[i]))),0.1);
 const get=(s,m)=>C.find(c=>c.speed===s&&c.mode===m);

 const verdict=(label,a,b)=>{
  const dd=dist(a,b),nn=noise(a,b),f=dd/nn;
  const cls=f>=3?'g':(f>=1.5?'w':'b');
  const txt=f>=3?'sicher trennbar':(f>=1.5?'Grenzbereich - mehr Messfenster wuerden helfen':'liegt im Rauschen, nicht trennbar');
  return '<div class="v '+cls+'"><b>'+label+'</b>Abstand '+dd.toFixed(1)+
   ' bei Streuung '+nn.toFixed(1)+' &rarr; Faktor '+f.toFixed(1)+'. '+txt+
   '<div class="bar"><i style="width:'+Math.min(f/5*100,100).toFixed(0)+'%;background:var(--'+
   (cls==='g'?'good':cls==='w'?'warn':'bad')+')"></i></div></div>';
 }

 h+='<h2>Stufen trennen sich?</h2>';
 [[1,2],[2,3],[1,3]].forEach(([x,y])=>{
  h+=verdict('Stufe '+x+' gegen Stufe '+y+' (bei Normal)',get(x,0),get(y,0));
 });

 h+='<h2>Modi trennen sich?</h2>';
 [1,2,3].forEach(s=>{
  h+=verdict('Breeze gegen Nacht bei Stufe '+s,get(s,1),get(s,2));
 });
 h+=verdict('Normal gegen Breeze (bei Stufe 2)',get(2,0),get(2,1));

 // ---- Schwellwert-Optimierung (auf dem ESP berechnet) ----
 if(d.optDone&&d.opt){
  h+='<h2>Optimale Schwellwerte</h2>';
  let rows='';
  d.opt.forEach((o,ch)=>{
   const f=x=>Number(x)>=3?'<span style="color:var(--good)">'+Number(x).toFixed(1)+'</span>'
            :Number(x)>=1.5?'<span style="color:var(--warn)">'+Number(x).toFixed(1)+'</span>'
            :'<span style="color:var(--bad)">'+Number(x).toFixed(1)+'</span>';
   rows+='<tr><td>'+CH[ch]+'</td><td>'+o.thr+(o.thr==o.cur?' <span class="sd">aktiv</span>':'')+
     '</td><td>'+f(o.speed)+'</td><td>'+f(o.mode)+'</td><td>'+f(o.off)+
     '</td><td>'+Number(o.lo).toFixed(0)+'&ndash;'+Number(o.hi).toFixed(0)+' %</td></tr>';
  });
  const vals=d.opt.map(o=>o.thr);
  const same=d.opt.every(o=>o.thr==o.cur);
  h+='<div class="card"><table><tr><th>Kanal</th><th>Schwelle</th><th>Stufe</th>'+
     '<th>Modus</th><th>Aus</th><th>Bereich</th></tr>'+rows+'</table>'+
     '<p>Die drei Zahlen sind die erreichbaren Trennschaerfe-Faktoren bei dieser Schwelle. '+
     'LED1&ndash;3 werden auf Stufentrennung optimiert, LED8/9 auf Modustrennung &ndash; '+
     'genau so werden sie auch bei der Erkennung gewichtet.</p>'+
     '<p style="color:var(--txt)"><b>Vorschlag:</b> {'+vals.join(',')+'}</p>'+
     (d.shape!==undefined?'<p><b style="color:var(--txt)">Formanteil:</b> '+
        Number(d.shape).toFixed(2)+' &middot; schlechtestes Paar dann Faktor '+
        Number(d.shapeScore).toFixed(2)+
        '<br><span class="sd">0 = nur Absolutpegel, 1 = nur Kanalform. Breeze und Nacht '+
        'liegen im Pegel gleichauf, unterscheiden sich aber in der Verteilung ueber die '+
        'Kanaele. Da das Rauschen gemultiplexter LEDs weitgehend gleichtaktig ist, faellt '+
        'es beim Zentrieren mit heraus.</span></p>':'')+
     (d.weights?'<p><b style="color:var(--txt)">Gemessene Kanalgewichte:</b> '+
        d.weights.map((w,i)=>CH[i]+' '+Number(w).toFixed(2)).join(' &middot; ')+
        '<br><span class="sd">Signal geteilt durch Rauschen je Kanal. Werden bei der '+
        'Uebernahme mitgespeichert und ersetzen die fest verdrahteten Gewichte.</span></p>':'')+
     (same
      ? '<p style="color:var(--good)">Entspricht den aktiven Werten &ndash; nichts zu tun.</p>'
      : '<button id="setThr" data-v="'+vals.join(',')+'">Diese Schwellwerte uebernehmen</button>'+
        '<p class="sd">Achtung: verwirft alle gelernten Profile. Danach neuer Messlauf noetig.</p>')+
     '<p id="thrMsg"></p></div>';
 }

 h+='<h2>Bleibt die Modus-Signatur ueber die Stufen stabil?</h2>';
 [1,2].forEach(m=>{
  const a=get(1,m),b=get(3,m);
  const dd=dist(a,b),nn=noise(a,b),f=dd/nn;
  const drift=f>=3;
  h+='<div class="v '+(drift?'b':'g')+'"><b>'+MODE[m]+' bei Stufe 1 gegen Stufe 3</b>'+
   (drift
    ?('Das Muster verschiebt sich um '+dd.toFixed(1)+' (Faktor '+f.toFixed(1)+
      '). Die Modus-Signatur haengt von der Stufe ab - du muesstest '+MODE[m]+
      ' bei jeder Stufe separat einlernen.')
    :('Verschiebung nur '+dd.toFixed(1)+'. Signatur ist stufenunabhaengig - Uebernahme durch Mitteln ist hier sicher.'))+
   '</div>';
 });
 $('#out').innerHTML=h;

 const bt=document.getElementById('setThr');
 if(bt) bt.onclick=async()=>{
  const v=bt.dataset.v.split(',').map(Number);
  const body={}; v.forEach((x,i)=>body['t'+i]=x);
  bt.disabled=true;
  try{
   const r=await fetch('/api/thresholds',{method:'POST',
     headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
   const j=await r.json();
   document.getElementById('thrMsg').textContent = j.ok
     ? 'Gespeichert. Die alten Profile wurden verworfen - bitte jetzt einen neuen Messlauf starten.'
     : ('Fehler: '+(j.error||'?'));
  }catch(e){document.getElementById('thrMsg').textContent='Keine Verbindung';}
  bt.disabled=false;
 };
}
</script></div></body></html>)ANA";

// ===================== TRENNSCHAERFE-ANALYSE =====================
// Faehrt alle 9 Kombinationen aus Stufe (1..3) und Modus (Normal/Breeze/Nacht)
// mehrfach an und misst jeweils das 5-Kanal-Profil. Aus Mittelwert und
// Streuung laesst sich beurteilen, ob die Erkennung Reserve hat.
// Beruehrt die normale Erkennung NICHT - reine Zusatzdiagnose.

constexpr uint8_t ANA_COMBOS  = 9;      // 3 Stufen x 3 Modi
constexpr uint8_t ANA_REPEATS = 5;

// Messfenster pro Einzelmessung. learnProfile() nimmt 72 Fenster je Zustand
// auf; mit 8 Fenstern x 9 Messungen kommt ein Profil hier auf denselben Wert.
// Weniger Fenster war der Grund, warum das Einzel-Lernen bisher stabiler war.
constexpr uint8_t ANA_WINDOWS = 6;

// Aufloesung des Histogramms. 64 Klassen a 64 ADC-Zaehler erlauben eine
// Schwellenwahl im 64er-Raster statt im groben 256er-Raster.
// Zustand, mit dem das Geraet aus dem AUS-Zustand hochlaeuft.
// Der gesamte Messlauf haengt daran: Stimmt dieser Ausgangspunkt nicht,
// sind ALLE neun Beschriftungen um denselben Betrag rotiert.
// Die Schlusspruefung am Ende des Laufs deckt genau das auf.
constexpr uint8_t ANA_START_SPEED = 1;
constexpr uint8_t ANA_START_MODE  = 0;   // 0 = Normal

constexpr uint8_t ANA_BINS      = 64;
constexpr uint8_t ANA_BIN_SHIFT = 6;    // 4096 >> 6 = 64

// Wartezeit nach einem Umschaltvorgang. Der Modus-Wechsel braucht spuerbar
// laenger, bis sich das Multiplexmuster beruhigt hat.
constexpr unsigned long ANA_SETTLE_MS      = 2500;
constexpr unsigned long ANA_SETTLE_MODE_MS = 3500;

struct AnalyzeRun {
  bool     running = false;
  bool     done    = false;
  uint8_t  repeat  = 0;                  // 0..ANA_REPEATS-1
  uint8_t  combo   = 0;                  // 0..ANA_COMBOS-1
  unsigned long nextStepMs = 0;
  float    samples[ANA_COMBOS][5][ANA_REPEATS];   // abovePct bei aktueller Schwelle
  float    rawmean[ANA_COMBOS][5][ANA_REPEATS];   // ADC-Mittelwert (fuer avgMean)
  // Verteilung der Rohwerte in 64 Klassen a 64 Zaehler, PRO Wiederholung.
  // Damit laesst sich fuer jede Schwelle nicht nur der abovePct-Wert,
  // sondern auch dessen Streuung ueber die Wiederholungen berechnen -
  // die Voraussetzung fuer eine echte Trennschaerfe-Optimierung.
  // 16 Klassen liessen bei den interessanten Schwellen nur 3 Kandidaten zu.
  uint16_t histC[ANA_COMBOS][5][ANA_REPEATS][ANA_BINS];
  uint16_t histN[ANA_COMBOS][5][ANA_REPEATS];
  uint16_t offHistC[5][ANA_BINS];
  uint16_t offHistN[5];
  // Ergebnis der Optimierung
  int      bestThr[5];
  float    scoreSpeed[5], scoreMode[5], scoreOff[5];
  float    rangeLo[5], rangeHi[5];
  float    weight[5] = {1,1,1,1,1};
  float    shapeBest = 0.0f;
  float    shapeScore = 0.0f;
  bool     optDone = false;
  // Schlusspruefung: Stufe 1 / Normal wird am Ende erneut gemessen.
  // Weicht das Ergebnis vom ersten Durchgang ab, ist beim Umschalten
  // etwas verrutscht und der ganze Lauf ist unbrauchbar.
  float    verifyPct[5];
  float    verifyDelta = -1.0f;
  bool     verifyDone  = false;
  // Live-Anzeige: was der Lauf gerade tut. Wird bei jedem Schritt gesetzt
  // und ueber /api/analyze mitgeliefert, damit man am Geraet gegenpruefen kann.
  char     action[96]   = "";
  // Abbruch: wird von aussen gesetzt, im naechsten Schritt ausgewertet.
  volatile bool stopRequested = false;
  bool     aborted      = false;
  // AUS wird am Ende des Laufs mitgemessen, damit die Erkennung den
  // ausgeschalteten Zustand nicht mehr schaetzen muss.
  float    offPct[5];
  float    offMean[5];
  bool     offMeasured = false;
  // Zustand, der vor dem Lauf aktiv war - wird danach wiederhergestellt
  uint8_t  restoreSpeed = 1;
  uint8_t  restoreMode  = 0;
  bool     restoreOsc   = false;
  bool     wasOff       = true;
  char     note[64]     = "";
} ana;

#if ENABLE_LED_FEEDBACK
// Ein Messdurchgang: 480 Proben a 250 us. Liefert Mittelwert und die
// Verteilung ueber 16 Klassen. Gleiche Dauer wie sampleFeedback().
void anaSampleInto(uint8_t pin, uint16_t bins[ANA_BINS], uint16_t& n) {
  const int N = 480;
  for(int i=0; i<N; i++) {
    int v = analogRead(pin);
    if(v < 0) v = 0;
    if(v > 4095) v = 4095;
    bins[v >> ANA_BIN_SHIFT]++;
    delayMicroseconds(250);
  }
  n += N;
}
#endif

#if ENABLE_LED_FEEDBACK
void anaSampleComboHist(uint8_t combo, uint8_t rep) {
  const uint8_t pins[5] = {PIN_LED1_FB,PIN_LED2_FB,PIN_LED3_FB,PIN_LED8_FB,PIN_LED9_FB};
  constexpr int N = 480;

  for(int i=0;i<N;i++) {
    int first = i % 5;
    for(int j=0;j<5;j++) {
      int ch = (first+j) % 5;
      (void)analogRead(pins[ch]);
      int v = analogRead(pins[ch]);
      if(v<0) v=0; if(v>4095) v=4095;
      ana.histC[combo][ch][rep][v >> ANA_BIN_SHIFT]++;
      ana.histN[combo][ch][rep]++;
    }
    delayMicroseconds(250);
  }
}

void anaSampleOffHist() {
  const uint8_t pins[5] = {PIN_LED1_FB,PIN_LED2_FB,PIN_LED3_FB,PIN_LED8_FB,PIN_LED9_FB};
  constexpr int N = 480;

  for(int i=0;i<N;i++) {
    int first = i % 5;
    for(int j=0;j<5;j++) {
      int ch = (first+j) % 5;
      (void)analogRead(pins[ch]);
      int v = analogRead(pins[ch]);
      if(v<0) v=0; if(v>4095) v=4095;
      ana.offHistC[ch][v >> ANA_BIN_SHIFT]++;
      ana.offHistN[ch]++;
    }
    delayMicroseconds(250);
  }
}
#endif


// Setzt den Live-Text und schreibt ihn zugleich ins Log.
void anaSetAction(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(ana.action, sizeof(ana.action), fmt, ap);
  va_end(ap);
  Serial.printf("[ANA] %s\n", ana.action);
}

inline uint8_t anaSpeedOf(uint8_t combo){ return (combo / 3) + 1; }   // 1,1,1,2,2,2,3,3,3
inline uint8_t anaModeOf (uint8_t combo){ return combo % 3; }        // 0,1,2,0,1,2,...

bool analyzeStart(String& err) {
  if(ana.running) { err = "laeuft bereits"; return false; }
  if(learnBusy)   { err = "manuelles Lernen laeuft gerade"; return false; }
#if !ENABLE_LED_FEEDBACK
  err = "LED-Feedback ist deaktiviert"; return false;
#else
  // ---- Definierte Startsequenz ----
  // 1) Ventilator sicher AUS
  // 2) einschalten
  // 3) Ausgangszustand als bekannt SETZEN statt aus state zu uebernehmen
  ana.wasOff = !fanIsOn;
  ana.restoreSpeed = state.speed >= 1 ? state.speed : 1;
  ana.restoreMode  = state.mode;
  ana.restoreOsc   = state.osc;

  anaSetAction("Startsequenz: bringe den Ventilator in den Aus-Zustand");
  if(fanIsOn) {
    powerOff();
    delay(2500);
  }
  anaSetAction("Startsequenz: schalte ein (erwartet Stufe 1 + Modus Normal)");
  togglePower();
  delay(ANA_SETTLE_MODE_MS);

  // Nicht raten, sondern festlegen: nach dem Einschalten gilt der
  // dokumentierte Startzustand des Geraets.
  fanIsOn     = true;
  state.speed = ANA_START_SPEED;
  state.mode  = ANA_START_MODE;
  state.osc   = false;
  Serial.printf("[ANA] Ausgangszustand gesetzt -> Stufe %u | Modus %u\n",
                state.speed, state.mode);

  memset(ana.samples, 0, sizeof(ana.samples));
  memset(ana.rawmean, 0, sizeof(ana.rawmean));
  memset(ana.histC,    0, sizeof(ana.histC));
  memset(ana.histN,    0, sizeof(ana.histN));
  memset(ana.offHistC, 0, sizeof(ana.offHistC));
  memset(ana.offHistN, 0, sizeof(ana.offHistN));
  ana.optDone = false;
  ana.verifyDone = false;
  ana.verifyDelta = -1.0f;
  memset(ana.verifyPct, 0, sizeof(ana.verifyPct));
  memset(ana.offPct,  0, sizeof(ana.offPct));
  memset(ana.offMean, 0, sizeof(ana.offMean));
  ana.offMeasured = false;
  ana.repeat = 0;
  ana.combo  = 0;
  ana.done   = false;
  ana.running = true;
  ana.nextStepMs = millis();
  ana.note[0] = 0;
  ana.action[0] = 0;
  ana.stopRequested = false;
  ana.aborted = false;
  Serial.println("[ANA] Messlauf gestartet");
  return true;
#endif
}

#if ENABLE_LED_FEEDBACK
void anaOptimize();   // Definition weiter unten
// Sauberer Abbruch: keine weiteren Befehle mehr an den Ventilator.
// Der Ventilator bleibt so stehen, wie er gerade steht - das ist gewollt,
// denn jede Wiederherstellung waere ein weiterer Befehl.
void analyzeAbort(const char* grund) {
  ana.running = false;
  ana.done    = false;
  ana.aborted = true;
  ana.stopRequested = false;
  snprintf(ana.action, sizeof(ana.action), "Abgebrochen: %s", grund);
  strncpy(ana.note, "Kalibrierung abgebrochen", sizeof(ana.note)-1);
  Serial.printf("[ANA] ABBRUCH: %s\n", grund);
}

void analyzeStep() {
  if(!ana.running) return;
  if(ana.stopRequested) { analyzeAbort("vom Benutzer gestoppt"); return; }
  if((long)(millis() - ana.nextStepMs) < 0) return;

  uint8_t sp = anaSpeedOf(ana.combo);
  uint8_t md = anaModeOf(ana.combo);

  // Feste, nachvollziehbare Reihenfolge:
  //   1. Modus zurueck auf Normal  (nur wenn die Stufe wechselt)
  //   2. Stufe setzen
  //   3. Modus setzen
  // Der Nacht-Modus kann die Drehzahl beeinflussen - deshalb wird die Stufe
  // grundsaetzlich im Normal-Modus gewechselt. Jeder Schritt wartet einzeln,
  // es wird nie zweimal hintereinander ohne Beruhigung geschaltet.
  static const char* MODE_NAME[3] = { "Normal", "Breeze", "Nacht" };

  if(state.speed != sp && state.mode != 0) {
    anaSetAction("Schalte Modus %s -> Normal (Vorbereitung fuer Stufenwechsel)", MODE_NAME[state.mode]);
    setMode(0); ana.nextStepMs = millis() + ANA_SETTLE_MODE_MS; return;
  }
  if(state.speed != sp) {
    anaSetAction("Schalte Stufe %u -> Stufe %u", state.speed, sp);
    setSpeed(sp); ana.nextStepMs = millis() + ANA_SETTLE_MS; return;
  }
  if(state.mode != md) {
    anaSetAction("Schalte Modus %s -> %s", MODE_NAME[state.mode], MODE_NAME[md]);
    setMode(md); ana.nextStepMs = millis() + ANA_SETTLE_MODE_MS; return;
  }

  // Sollzustand erreicht - erst jetzt wird gemessen.
  uint16_t nr = ana.repeat * ANA_COMBOS + ana.combo + 1;
  anaSetAction("Messe %u/%u: Stufe %u + Modus %s (Durchgang %u/%u)",
               nr, ANA_COMBOS*ANA_REPEATS, sp, MODE_NAME[md], ana.repeat+1, ANA_REPEATS);

  // Gegenprobe: passt der interne Zustand wirklich zum Soll?
  if(state.speed != sp || state.mode != md) {
    Serial.printf("[ANA] ABBRUCH: Sollzustand %u/%u, tatsaechlich %u/%u\n",
                  sp, md, state.speed, state.mode);
    strncpy(ana.note, "Zustand stimmte nicht - Lauf abgebrochen", sizeof(ana.note)-1);
    ana.running = false;
    return;
  }

  // Ueber mehrere Fenster mitteln statt einer Momentaufnahme.
  FbStats cur[5];
  float accPct[5]  = {0,0,0,0,0};
  float accMean[5] = {0,0,0,0,0};
  for(uint8_t w = 0; w < ANA_WINDOWS; w++) {
    sampleCurrent5(cur);
    for(int ch=0; ch<5; ch++) {
      accPct[ch]  += cur[ch].abovePct;
      accMean[ch] += cur[ch].avg;
    }
    delay(30);
    server.handleClient();      // Oberflaeche waehrend der langen Messung bedienbar halten
    if(ana.stopRequested) { analyzeAbort("vom Benutzer gestoppt"); return; }
  }
  for(int ch=0; ch<5; ch++) {
    ana.samples[ana.combo][ch][ana.repeat] = accPct[ch]  / ANA_WINDOWS;
    ana.rawmean[ana.combo][ch][ana.repeat] = accMean[ch] / ANA_WINDOWS;
  }
  Serial.printf("[ANA]   Stufe: %u | Modus: %s | Messwert: %.1f %.1f %.1f %.1f %.1f\n",
                sp, MODE_NAME[md],
                ana.samples[ana.combo][0][ana.repeat], ana.samples[ana.combo][1][ana.repeat],
                ana.samples[ana.combo][2][ana.repeat], ana.samples[ana.combo][3][ana.repeat],
                ana.samples[ana.combo][4][ana.repeat]);

  // Histogramm ebenfalls zeitgleich ueber alle Kanaele aufnehmen.
  // Drei synchronisierte Histogrammfenster liefern 1440 Werte je Kanal.
  const uint8_t HIST_WINDOWS = 3;
  for(uint8_t w=0; w<HIST_WINDOWS; w++) {
    anaSampleComboHist(ana.combo, ana.repeat);
    server.handleClient();
    if(ana.stopRequested) { analyzeAbort("vom Benutzer gestoppt"); return; }
  }

  Serial.printf("[ANA] r%u  Stufe %u  Modus %u  ->  %.2f %.2f %.2f %.2f %.2f\n",
                ana.repeat+1, sp, md,
                cur[0].abovePct, cur[1].abovePct, cur[2].abovePct,
                cur[3].abovePct, cur[4].abovePct);

  ana.combo++;
  if(ana.combo >= ANA_COMBOS) {
    ana.combo = 0;
    ana.repeat++;
    if(ana.repeat >= ANA_REPEATS) {
      // ---- Schlusspruefung ----
      // Zurueck auf den Ausgangszustand Stufe 1 / Normal und erneut messen.
      // Stimmt das Ergebnis mit dem allerersten Messwert ueberein, ist beim
      // Umschalten nichts verrutscht. Weicht es ab, war die Zaehlung falsch
      // und alle Beschriftungen des Laufs sind unbrauchbar.
      anaSetAction("Schlusspruefung: zurueck auf Stufe 1 + Modus Normal");
      if(state.mode != 0) { setMode(0); delay(ANA_SETTLE_MODE_MS); }
      if(state.speed != 1) { setSpeed(1); delay(ANA_SETTLE_MS); }
      delay(ANA_SETTLE_MODE_MS);

      FbStats vf[5];
      float vacc[5] = {0,0,0,0,0};
      for(uint8_t w=0; w<ANA_WINDOWS; w++) {
        sampleCurrent5(vf);
        for(int ch=0; ch<5; ch++) vacc[ch] += vf[ch].abovePct;
        delay(30);
        server.handleClient();
      }
      float worst = 0.0f;
      for(int ch=0; ch<5; ch++) {
        ana.verifyPct[ch] = vacc[ch] / ANA_WINDOWS;
        // Referenz: Mittel der drei Durchgaenge von Kombination 0 (St.1/Normal)
        float ref = 0.0f;
        for(int r=0; r<ANA_REPEATS; r++) ref += ana.samples[0][ch][r];
        ref /= ANA_REPEATS;
        float d = fabsf(ana.verifyPct[ch] - ref);
        if(d > worst) worst = d;
      }
      ana.verifyDelta = worst;
      ana.verifyDone  = true;
      Serial.printf("[ANA] Schlusspruefung: groesste Abweichung %.2f Prozentpunkte -> %s\n",
                    worst, worst < 1.5f ? "in Ordnung" : "VERDACHT auf Zaehlfehler");

      // AUS-Zustand mitmessen: abschalten, beruhigen lassen, dreimal messen.
      anaSetAction("Messe den Aus-Zustand: schalte ab");
      powerOff();
      delay(3000);
      FbStats off[5];
      for(int r=0; r<8; r++) {
        sampleCurrent5(off);
        for(int ch=0; ch<5; ch++) {
          ana.offPct[ch]  += off[ch].abovePct / 8.0f;
          ana.offMean[ch] += off[ch].avg      / 8.0f;
        }
        delay(60);
      }
      for(uint8_t w=0; w<3; w++) anaSampleOffHist();
      ana.offMeasured = true;
      Serial.printf("[ANA] AUS gemessen: %.1f %.1f %.1f %.1f %.1f\n",
                    ana.offPct[0],ana.offPct[1],ana.offPct[2],ana.offPct[3],ana.offPct[4]);

      anaSetAction("Stelle den Ausgangszustand wieder her");
      // Ausgangszustand wiederherstellen
      if(!ana.wasOff) {
        togglePower();
        delay(1200);
        setSpeed(ana.restoreSpeed);
        setMode(ana.restoreMode);
        if(state.osc != ana.restoreOsc) toggleOsc();
      }
      anaSetAction("Werte aus: Schwellwerte, Kanalgewichte, Formanteil");
      anaOptimize();
      anaSetAction("Fertig");
      ana.running = false;
      ana.done    = true;
      Serial.println("[ANA] Messlauf fertig");
      return;
    }
  }
  ana.nextStepMs = millis() + 250;
}
#else
void analyzeStep() {}
#endif

// Ergebnisse des Messlaufs als Kalibrierung uebernehmen.
// Die Stufen-Signatur wird ueber alle drei Modi gemittelt, die Modus-Signatur
// ueber alle drei Stufen. Dadurch ist jedes Profil unabhaengig von der jeweils
// anderen Dimension - robuster als ein Einlernen an einem einzelnen Betriebspunkt.

// Explizite Vorwaertsdeklarationen:
// analyzeApply() verwendet diese Histogramm-Helfer, bevor ihre Definitionen
// weiter unten im Sketch stehen. Bei .ino-Autoprototypisierung war die
// Reihenfolge nicht verlaesslich, daher deklarieren wir sie bewusst selbst.
float anaAbovePct(uint8_t c, uint8_t ch, uint8_t r, uint8_t t);
float anaOffPctAt(uint8_t ch, uint8_t t);

bool analyzeApply(String& err) {
#if !ENABLE_LED_FEEDBACK
  err = "LED-Feedback ist deaktiviert"; return false;
#else
  if(ana.running) { err = "Messlauf laeuft noch"; return false; }
  if(!ana.done)   { err = "kein abgeschlossener Messlauf vorhanden"; return false; }
  if(ana.verifyDone && ana.verifyDelta >= 1.5f) {
    err = "Schlusspruefung fehlgeschlagen - beim Umschalten ist etwas verrutscht. Bitte Messlauf wiederholen.";
    return false;
  }

  // Die Optimierung hat die besten Schwellen bereits aus den Roh-Histogrammen
  // bestimmt. Sie werden zusammen mit den Profilen atomar uebernommen.
  // Dadurch ist kein zweiter kompletter Messlauf nach einer Schwellwertaenderung
  // mehr notwendig.
  if(ana.optDone) {
    for(int ch=0; ch<5; ch++) {
      fbThresholds[ch] = ana.bestThr[ch];
      saveFeedbackThreshold(ch, fbThresholds[ch]);
    }
  }

  uint8_t thrBin[5];
  for(int ch=0; ch<5; ch++)
    thrBin[ch] = (uint8_t)constrain(fbThresholds[ch] >> ANA_BIN_SHIFT, 1, ANA_BINS-1);

  // Stufenprofile: Prozentwerte direkt aus denselben Roh-Histogrammen bei
  // der jetzt aktiven optimierten Schwelle rekonstruieren.
  for(int sp=1; sp<=3; sp++) {
    int idx = sp-1;
    for(int ch=0; ch<5; ch++) {
      double sp_pct=0, sp_raw=0; int n=0;
      for(int c=0; c<ANA_COMBOS; c++) {
        if(anaSpeedOf(c) != sp) continue;
        for(int r=0; r<ANA_REPEATS; r++) {
          sp_pct += anaAbovePct(c,ch,r,thrBin[ch]);
          sp_raw += ana.rawmean[c][ch][r];
          n++;
        }
      }
      if(!n) { err = "unvollstaendige Messdaten"; return false; }
      speedSig[idx].avgPct[ch]  = (float)(sp_pct / n);
      speedSig[idx].avgMean[ch] = (float)(sp_raw / n);
    }
    speedSig[idx].valid = true;
    saveSpeedCalibration(idx);
  }

  // Modi: weiterhin als Rueckfallprofil vorhanden.
  for(int md=0; md<3; md++) {
    for(int ch=0; ch<5; ch++) {
      double m_pct=0, m_raw=0; int n=0;
      for(int c=0; c<ANA_COMBOS; c++) {
        if(anaModeOf(c) != md) continue;
        for(int r=0; r<ANA_REPEATS; r++) {
          m_pct += anaAbovePct(c,ch,r,thrBin[ch]);
          m_raw += ana.rawmean[c][ch][r];
          n++;
        }
      }
      if(!n) { err = "unvollstaendige Messdaten"; return false; }
      modeSig[md].avgPct[ch]  = (float)(m_pct / n);
      modeSig[md].avgMean[ch] = (float)(m_raw / n);
    }
    modeSig[md].valid = true;
    saveModeCalibration(md);
  }

  // Neun Kombiprofile sind die finale primaere Erkennung, weil die Live-Messung
  // klar zeigt, dass Breeze/Nacht von der jeweiligen Stufe abhaengen.
  for(int c=0; c<ANA_COMBOS; c++) {
    int sp = anaSpeedOf(c)-1;
    int md = anaModeOf(c);
    for(int ch=0; ch<5; ch++) {
      double p=0, m=0;
      for(int r=0; r<ANA_REPEATS; r++) {
        p += anaAbovePct(c,ch,r,thrBin[ch]);
        m += ana.rawmean[c][ch][r];
      }
      comboSig[sp][md].avgPct[ch]  = (float)(p / ANA_REPEATS);
      comboSig[sp][md].avgMean[ch] = (float)(m / ANA_REPEATS);
    }
    comboSig[sp][md].valid = true;
  }

  if(ana.optDone) {
    for(int ch=0; ch<5; ch++) comboW[ch] = ana.weight[ch];
    comboWValid = true;
    comboShape  = ana.shapeBest;
  }
  saveComboCalibration();

  // AUS-Profil ebenfalls an der optimierten Schwelle rekonstruieren.
  if(ana.offMeasured) {
    for(int ch=0; ch<5; ch++) {
      offSig.avgPct[ch]  = anaOffPctAt(ch,thrBin[ch]);
      offSig.avgMean[ch] = ana.offMean[ch];
    }
    offSig.valid = true;
    saveOffCalibration();
  }

  Serial.printf("[ANA] Uebernommen: Schwellen %d %d %d %d %d | Gewichte %.2f %.2f %.2f %.2f %.2f\n",
                fbThresholds[0],fbThresholds[1],fbThresholds[2],fbThresholds[3],fbThresholds[4],
                comboW[0],comboW[1],comboW[2],comboW[3],comboW[4]);
  return true;
#endif
}

// ---------- Schwellwert-Optimierung ----------
// Aus den gespeicherten Verteilungen laesst sich fuer jede Schwelle der
// abovePct-Wert samt Streuung berechnen, ohne neu zu messen.
float anaAbovePct(uint8_t c, uint8_t ch, uint8_t r, uint8_t t) {
  uint32_t sum = 0;
  for(uint8_t b = t; b < ANA_BINS; b++) sum += ana.histC[c][ch][r][b];
  uint16_t n = ana.histN[c][ch][r];
  return n ? (sum * 100.0f / n) : 0.0f;
}

float anaOffPctAt(uint8_t ch, uint8_t t) {
  uint32_t sum = 0;
  for(uint8_t b = t; b < ANA_BINS; b++) sum += ana.offHistC[ch][b];
  uint16_t n = ana.offHistN[ch];
  return n ? (sum * 100.0f / n) : 0.0f;
}

void anaMeanSd(uint8_t c, uint8_t ch, uint8_t t, float& m, float& sd) {
  float v[ANA_REPEATS];
  m = 0.0f;
  for(uint8_t r = 0; r < ANA_REPEATS; r++) { v[r] = anaAbovePct(c, ch, r, t); m += v[r]; }
  m /= ANA_REPEATS;
  float acc = 0.0f;
  for(uint8_t r = 0; r < ANA_REPEATS; r++) acc += (v[r]-m)*(v[r]-m);
  sd = sqrtf(acc / (ANA_REPEATS > 1 ? (ANA_REPEATS - 1) : 1));
}

// Sucht je Kanal die Schwelle mit der besten Trennschaerfe.
// Die Kanaele haben unterschiedliche Aufgaben: LED1..3 tragen die Stufe,
// LED8/9 den Modus. Entsprechend werden sie unterschiedlich gewichtet -
// so bekommt der schwierigste Vergleich (Breeze gegen Nacht) auf den
// Modus-Kanaelen die Schwelle, die ihm am meisten hilft.
void anaOptimize() {
  for(uint8_t ch = 0; ch < 5; ch++) {
    bool isModeCh = (ch >= 3);
    float bestScore = -1.0f;
    int   bestT = -1;

    for(uint8_t t = 1; t < ANA_BINS; t++) {
      float m[ANA_COMBOS], sd[ANA_COMBOS];
      float lo = 1e9f, hi = -1e9f;
      for(uint8_t c = 0; c < ANA_COMBOS; c++) {
        anaMeanSd(c, ch, t, m[c], sd[c]);
        if(m[c] < lo) lo = m[c];
        if(m[c] > hi) hi = m[c];
      }

      // Stufen-Trennschaerfe: Paare gleicher Modi, verschiedener Stufen
      float sSpeed = 1e9f;
      for(uint8_t md = 0; md < 3; md++)
        for(uint8_t a = 0; a < 3; a++)
          for(uint8_t b = a+1; b < 3; b++) {
            uint8_t ca = a*3+md, cb = b*3+md;
            float noise = max(max(sd[ca], sd[cb]), 0.15f);
            float f = fabsf(m[ca]-m[cb]) / noise;
            if(f < sSpeed) sSpeed = f;
          }

      // Modus-Trennschaerfe: Paare gleicher Stufe, verschiedener Modi
      float sMode = 1e9f;
      for(uint8_t sp = 0; sp < 3; sp++)
        for(uint8_t a = 0; a < 3; a++)
          for(uint8_t b = a+1; b < 3; b++) {
            uint8_t ca = sp*3+a, cb = sp*3+b;
            float noise = max(max(sd[ca], sd[cb]), 0.15f);
            float f = fabsf(m[ca]-m[cb]) / noise;
            if(f < sMode) sMode = f;
          }

      // Abstand zum AUS-Zustand
      float sOff = 1e9f;
      if(ana.offMeasured) {
        float mo = anaOffPctAt(ch, t);
        for(uint8_t c = 0; c < ANA_COMBOS; c++) {
          float noise = max(sd[c], 0.15f);
          float f = fabsf(m[c]-mo) / noise;
          if(f < sOff) sOff = f;
        }
      } else sOff = 0.0f;

      // Saettigung bestrafen: liegt alles am Anschlag, ist der Wert wertlos
      float mid = (lo + hi) * 0.5f;
      float pen = (mid > 93.0f || mid < 7.0f) ? 0.25f : 1.0f;

      // Deckelung bei 4: ist ein Vergleich sicher genug, soll der
      // Optimierer den schwaecheren verbessern statt den starken auszureizen.
      float capS = min(sSpeed, 4.0f);
      float capM = min(sMode,  4.0f);
      float capO = min(sOff,   4.0f);
      float wS = isModeCh ? 0.45f : 1.0f;
      float wM = isModeCh ? 1.0f  : 0.45f;
      float score = (wS*capS + wM*capM + 0.4f*capO) * pen;

      if(score > bestScore) {
        bestScore = score;
        bestT = t;
        ana.scoreSpeed[ch] = sSpeed;
        ana.scoreMode[ch]  = sMode;
        ana.scoreOff[ch]   = sOff;
        ana.rangeLo[ch]    = lo;
        ana.rangeHi[ch]    = hi;
      }
    }
    ana.bestThr[ch] = (bestT < 0) ? fbThresholds[ch] : (bestT << ANA_BIN_SHIFT);
    Serial.printf("[OPT] Kanal %d: Schwelle %d  Stufe %.1f  Modus %.1f  Aus %.1f  Bereich %.0f-%.0f\n",
                  ch, ana.bestThr[ch], ana.scoreSpeed[ch], ana.scoreMode[ch],
                  ana.scoreOff[ch], ana.rangeLo[ch], ana.rangeHi[ch]);
  }
  // ---- Kanalgewichte aus der Messung ableiten ----
  // Bewertet wird bei der AKTIVEN Schwelle, denn genau dort entstehen
  // spaeter auch die gelernten Profile.
  {
    float raw[5], sum = 0.0f;
    for(uint8_t ch = 0; ch < 5; ch++) {
      uint8_t t = (uint8_t)constrain(ana.bestThr[ch] >> ANA_BIN_SHIFT, 1, ANA_BINS-1);
      float lo = 1e9f, hi = -1e9f, noise = 0.0f;
      for(uint8_t c = 0; c < ANA_COMBOS; c++) {
        float m, sd;
        anaMeanSd(c, ch, t, m, sd);
        if(m < lo) lo = m;
        if(m > hi) hi = m;
        noise += sd;
      }
      noise /= ANA_COMBOS;
      raw[ch] = (hi - lo) / max(noise, 0.10f);   // Signal durch Rauschen
      sum += raw[ch];
    }
    if(sum > 0.01f) {
      for(uint8_t ch = 0; ch < 5; ch++) {
        ana.weight[ch] = raw[ch] * 5.0f / sum;   // Summe bleibt 5
        // Untergrenze, damit ein schwacher Kanal nicht voellig verstummt
        if(ana.weight[ch] < 0.05f) ana.weight[ch] = 0.05f;
      }
      Serial.printf("[OPT] Kanalgewichte: %.2f %.2f %.2f %.2f %.2f\n",
                    ana.weight[0],ana.weight[1],ana.weight[2],ana.weight[3],ana.weight[4]);
    }
  }

  // ---- Formanteil bestimmen ----
  // Getestet werden mehrere Mischungen aus Absolutpegel und Form. Bewertet
  // wird die SCHLECHTESTE Trennung ueber alle 36 Paare der neun Zustaende -
  // die entscheidet in der Praxis, nicht der Durchschnitt.
  {
    const float cand[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
    float bestWorst = -1.0f, bestK = 0.0f;

    for(uint8_t ki = 0; ki < 5; ki++) {
      float k = cand[ki];

      // Merkmalsvektoren je Kombination und Wiederholung
      float f[ANA_COMBOS][ANA_REPEATS][5];
      for(uint8_t c = 0; c < ANA_COMBOS; c++)
        for(uint8_t r = 0; r < ANA_REPEATS; r++) {
          float pct[5];
          for(int ch=0; ch<5; ch++) pct[ch] = ana.samples[c][ch][r];
          comboFeature(pct, k, f[c][r]);
        }

      // Mittelwert und Streuung je Kombination im Merkmalsraum
      float mu[ANA_COMBOS][5], sg[ANA_COMBOS];
      for(uint8_t c = 0; c < ANA_COMBOS; c++) {
        float var = 0.0f;
        for(int ch=0; ch<5; ch++) {
          float m = 0.0f;
          for(uint8_t r=0; r<ANA_REPEATS; r++) m += f[c][r][ch];
          m /= ANA_REPEATS;
          mu[c][ch] = m;
          for(uint8_t r=0; r<ANA_REPEATS; r++) {
            float d = f[c][r][ch] - m;
            var += ana.weight[ch] * d * d;
          }
        }
        sg[c] = sqrtf(var / (ANA_REPEATS * 5));
      }

      // Schlechtestes Paar
      float worst = 1e30f;
      for(uint8_t a = 0; a < ANA_COMBOS; a++)
        for(uint8_t b = a+1; b < ANA_COMBOS; b++) {
          float d2 = 0.0f;
          for(int ch=0; ch<5; ch++) {
            float e = mu[a][ch] - mu[b][ch];
            d2 += ana.weight[ch] * e * e;
          }
          float sep = sqrtf(d2) / max(max(sg[a], sg[b]), 0.05f);
          if(sep < worst) worst = sep;
        }

      Serial.printf("[SHAPE] Anteil %.2f -> schlechteste Trennung %.2f\n", k, worst);
      if(worst > bestWorst) { bestWorst = worst; bestK = k; }
    }

    ana.shapeBest  = bestK;
    ana.shapeScore = bestWorst;
    Serial.printf("[SHAPE] Gewaehlt: Anteil %.2f, schlechtestes Paar Faktor %.2f\n",
                  bestK, bestWorst);
  }

  ana.optDone = true;
}

String analyzeJson() {
  // Die Antwort ist mit Histogrammen rund 7 kB gross. Ohne reserve() waechst
  // der String in vielen Schritten und fragmentiert den Heap.
  String j;
  j.reserve(8192);
  j = "{";
  j += "\"running\":" + String(ana.running ? "true" : "false") + ",";
  j += "\"done\":"    + String(ana.done ? "true" : "false") + ",";
  j += "\"repeats\":" + String(ANA_REPEATS) + ",";
  int total = ANA_COMBOS * ANA_REPEATS;
  int steps = ana.repeat * ANA_COMBOS + ana.combo;
  j += "\"progress\":" + String(ana.done ? 100 : (total ? steps * 100 / total : 0)) + ",";
  j += "\"note\":\"" + String(ana.note) + "\",";
  j += "\"action\":\"" + String(ana.action) + "\",";
  j += "\"aborted\":" + String(ana.aborted ? "true" : "false") + ",";
  j += "\"offMeasured\":" + String(ana.offMeasured ? "true" : "false") + ",";
  j += "\"off\":[";
  for(int ch=0; ch<5; ch++) { if(ch) j += ","; j += String(ana.offPct[ch], 1); }
  j += "],";

  j += "\"combos\":[";
  for(int c=0; c<ANA_COMBOS; c++) {
    if(c) j += ",";
    j += "{\"speed\":" + String(anaSpeedOf(c)) + ",\"mode\":" + String(anaModeOf(c)) + ",\"ch\":[";
    for(int ch=0; ch<5; ch++) {
      if(ch) j += ",";
      j += "[";
      for(int r=0; r<ANA_REPEATS; r++) {
        if(r) j += ",";
        j += String(ana.samples[c][ch][r], 1);
      }
      j += "]";
    }
    j += "]}";
  }
  j += "],";

  // Ergebnis der Schwellwert-Optimierung
  j += "\"verifyDone\":" + String(ana.verifyDone ? "true" : "false") + ",";
  j += "\"verifyDelta\":" + String(ana.verifyDelta, 2) + ",";
  j += "\"shape\":" + String(ana.shapeBest, 2) + ",";
  j += "\"shapeScore\":" + String(ana.shapeScore, 2) + ",";
  j += "\"optDone\":" + String(ana.optDone ? "true" : "false") + ",";
  j += "\"weights\":[";
  for(int ch=0; ch<5; ch++) { if(ch) j += ","; j += String(ana.weight[ch], 2); }
  j += "],";
  j += "\"opt\":[";
  for(int ch=0; ch<5; ch++) {
    if(ch) j += ",";
    j += "{\"thr\":"   + String(ana.bestThr[ch]);
    j += ",\"cur\":"   + String(fbThresholds[ch]);
    j += ",\"speed\":" + String(ana.scoreSpeed[ch], 1);
    j += ",\"mode\":"  + String(ana.scoreMode[ch], 1);
    j += ",\"off\":"   + String(ana.scoreOff[ch], 1);
    j += ",\"lo\":"    + String(ana.rangeLo[ch], 1);
    j += ",\"hi\":"    + String(ana.rangeHi[ch], 1);
    j += "}";
  }
  j += "]}";
  return j;
}
// =================== ENDE TRENNSCHAERFE-ANALYSE ===================

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
    j+="\"version\":\""+String(FW_VERSION)+"\",";
    j+="\"build\":\""+String(__DATE__)+" "+String(__TIME__)+"\",";
    j+="\"ip\":\""+ip+"\",";
    j+="\"rssi\":"+String(rssi)+",";
    j+="\"uptime\":"+String(millis()/1000UL);
    j+="}";
    server.send(200,"application/json",j);
  });

  server.on("/api/status",HTTP_GET,sendStatus);

  // AUS-Zustand lernen: Ventilator muss dabei wirklich aus sein.
  server.on("/api/learn-off",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    if(ana.running || learnBusy) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"messung_laeuft\"}");
      return;
    }
    learnBusy = true;
    bool ok = learnOffSignature();
    learnBusy = false;
    // Nach dem Lernen gilt: der Ventilator ist aus.
    if(ok) {
      fanIsOn = false;
      state.speed = 0; state.osc = false; state.mode = 0;
      cancelTimer();
      startupScanDone = true;
      startupSyncUntilMs = 0;
      startupSyncReadyMs = 0;
      saveFanState();
    }
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });

  server.on("/api/sync-from-leds",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    FbStats cur[5];
    sampleCurrent5(cur);

    // 1) AUS-Erkennung hat Vorrang.
    if(ledsLookOff(cur)) {
      fanIsOn = false;
      state.speed = 0;
      state.osc = false;
      state.mode = 0;
      cancelTimer();
      startupSpeedCandidate = 0;
      startupSpeedHits = 0;
      startupModeCandidate = -1;
      startupModeHits = 0;

      // Manueller Sync beendet jeden laufenden Startscan.
      startupScanDone = true;
      startupSyncUntilMs = 0;
      startupSyncReadyMs = 0;
      manualLedSyncRequested = false;

      String j="{";
      j+="\"ok\":true,";
      j+="\"powerOn\":false,";
      j+="\"speed\":0,";
      j+="\"mode\":0,";
      j+="\"detectedOff\":true";
      j+="}";
      server.send(200,"application/json",j);
      return;
    }

    // 2) Ventilator läuft: gelernte Profile EINMAL messen und direkt übernehmen.
    // Dieser Button ist bewusst ein "mach die Anzeige jetzt nach Messwerten"-Befehl.
    fanIsOn = true;

    float speedConf=0.0f, modeConf=0.0f;
    const float speedWeights[5] = {1.8f,1.8f,1.8f,0.08f,0.08f};
    const float modeWeights[5]  = {0.08f,0.08f,0.08f,2.2f,2.2f};

    int learnedSpeed, learnedMode;
    if(comboComplete()) {
      int sp=0, md=-1;
      detectCombo(cur, &sp, &md, &speedConf, &modeConf);
      learnedSpeed = sp;
      learnedMode  = md;
      Serial.printf("[SYNC] Kombiprofil: Stufe %d, Modus %d (%.0f %%)\n",
                    learnedSpeed, learnedMode, speedConf);
    } else {
      int speedIdx=detectProfilesWeightedFromCurrent(speedSig,cur,speedWeights,&speedConf);
      int modeIdx =detectProfilesWeightedFromCurrent(modeSig,cur,modeWeights,&modeConf);
      learnedSpeed = speedIdx<0 ? 0 : speedIdx+1;
      learnedMode  = modeIdx;
    }

    // Manueller Abgleich soll zuverlaessig sein, nicht nur irgendeinen
    // "besten" Treffer erzwingen. Deine Live-Daten zeigen:
    // Speed ist klar trennbar, Breeze gegen Nacht liegt dagegen teilweise
    // innerhalb der Streuung. Deshalb Speed mit niedriger, Modus mit
    // strengerer Mindestkonfidenz uebernehmen.
    bool speedApplied = false;
    bool modeApplied  = false;

    if(learnedSpeed>=1 && learnedSpeed<=3 && speedConf>=8.0f) {
      state.speed = learnedSpeed;
      state.lastSpeed = learnedSpeed;
      speedApplied = true;
    }

    if(learnedMode>=0 && learnedMode<=2 && modeConf>=20.0f) {
      state.mode = learnedMode;
      state.lastMode = learnedMode;
      modeApplied = true;
    }

    saveFanState();

    // Danach keinerlei automatisches Nachkorrigieren durch den Startscan.
    startupScanDone = true;
    startupSyncUntilMs = 0;
    startupSyncReadyMs = 0;
    manualLedSyncRequested = false;

    String j="{";
    j+="\"ok\":true,";
    j+="\"powerOn\":true,";
    j+="\"speed\":"+String(state.speed)+",";
    j+="\"mode\":"+String(state.mode)+",";
    j+="\"detectedOff\":false,";
    j+="\"speedConfidence\":"+String(speedConf,1)+",";
    j+="\"modeConfidence\":"+String(modeConf,1)+",";
    j+="\"speedApplied\":"+String(speedApplied?"true":"false")+",";
    j+="\"modeApplied\":"+String(modeApplied?"true":"false");
    j+="}";
    server.send(200,"application/json",j);
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });

  server.on("/api/power",HTTP_POST,[](){
    togglePower();
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/speed",HTTP_POST,[](){
    int target=jsonInt(server.arg("plain"),"target",-1);
    if(target<0||target>3){server.send(400,"text/plain","invalid speed");return;}

    if(target>0 && !fanIsOn){
      server.send(409,"application/json","{\"ok\":false,\"error\":\"power_off\"}");
      return;
    }

    setSpeed((uint8_t)target);
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/mode",HTTP_POST,[](){
    int target=jsonInt(server.arg("plain"),"target",-1);
    if(target<0||target>2){server.send(400,"text/plain","invalid mode");return;}
    setMode((uint8_t)target);
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/osc",HTTP_POST,[](){
    toggleOsc();
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/timer",HTTP_POST,[](){
    int minutes=jsonInt(server.arg("plain"),"minutes",0);
    minutes=constrain(minutes,0,240);
    state.sleepMin=minutes;
    timerDeadlineMs=minutes>0 ? millis()+(unsigned long)minutes*60000UL : 0;
    Serial.printf("[TIMER] auf %d min gesetzt\n", minutes);
    server.send(200,"application/json","{\"ok\":true}");
  });

  // Nur internen Zustand zuruecksetzen. Kein physischer Tastendruck.
  server.on("/api/sync",HTTP_POST,[](){
    state=FanState();
    fanIsOn=false;            // sonst: speed=0 bei fanIsOn=true -> falsche Impulszahl
    timerDeadlineMs=0;
    startupSyncUntilMs=0;
    startupSyncReadyMs=0;
    startupScanDone=false;
    manualLedSyncRequested=false;
    saveFanState();
    Serial.println("[SYNC] Interner Zustand auf AUS gesetzt");
    server.send(200,"application/json","{\"ok\":true}");
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
    learnBusy = true;
    bool ok = learnSpeedSignature(speed);
    learnBusy = false;
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
    learnBusy = true;
    bool ok=learnModeSignature(mode);
    learnBusy = false;
    server.send(ok?200:500,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });


  server.on("/api/forget-speed-calibration",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    // Nur Speed 1..3 löschen
    prefs.begin("fancal2", false);
    for(int s=0;s<3;s++) {
      String vk="sv"+String(s);
      prefs.remove(vk.c_str());
      for(int ch=0;ch<5;ch++) {
        String pk="sp"+String(s)+String(ch);
        String mk="sm"+String(s)+String(ch);
        prefs.remove(pk.c_str());
        prefs.remove(mk.c_str());
      }
      speedSig[s].valid=false;
      for(int ch=0;ch<5;ch++) {
        speedSig[s].avgPct[ch]=0.0f;
        speedSig[s].avgMean[ch]=0.0f;
      }
    }
    prefs.end();


    server.send(200,"application/json","{\"ok\":true}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });

  server.on("/api/forget-mode-calibration",HTTP_POST,[](){
#if ENABLE_LED_FEEDBACK
    // Nur Normal / Breeze / Night löschen
    prefs.begin("fancal2", false);
    for(int s=0;s<3;s++) {
      String vk="mv"+String(s);
      prefs.remove(vk.c_str());
      for(int ch=0;ch<5;ch++) {
        String pk="mp"+String(s)+String(ch);
        String mk="mm"+String(s)+String(ch);
        prefs.remove(pk.c_str());
        prefs.remove(mk.c_str());
      }
      modeSig[s].valid=false;
      for(int ch=0;ch<5;ch++) {
        modeSig[s].avgPct[ch]=0.0f;
        modeSig[s].avgMean[ch]=0.0f;
      }
    }
    prefs.end();


    server.send(200,"application/json","{\"ok\":true}");
#else
    server.send(503,"text/plain","feedback disabled");
#endif
  });


  server.on("/api/backup/export",HTTP_GET,[](){
    String j=buildBackupJson();
    server.sendHeader("Content-Disposition","attachment; filename=ventilator-backup.json");
    server.send(200,"application/json",j);
  });

  server.on("/api/backup/import",HTTP_POST,[](){
    String body=server.arg("plain");
    String err;
    if(!applyBackupJson(body,err)) {
      server.send(400,"text/plain",err);
      return;
    }
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/analyze/start",HTTP_POST,[](){
    String err;
    if(!analyzeStart(err)) { server.send(409,"application/json","{\"ok\":false,\"error\":\""+err+"\"}"); return; }
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/analyze/stop",HTTP_POST,[](){
    if(!ana.running) {
      server.send(409,"application/json","{\"ok\":false,\"error\":\"kein_lauf_aktiv\"}");
      return;
    }
    ana.stopRequested = true;      // wird im naechsten Schritt ausgewertet
    Serial.println("[ANA] Stopp angefordert");
    server.send(200,"application/json","{\"ok\":true}");
  });

  server.on("/api/analyze",HTTP_GET,[](){
    server.send(200,"application/json",analyzeJson());
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
    for(int i=0;i<5;i++) saveFeedbackThreshold(i, neu[i]);

    // Mit neuer Schwelle sind die alten Profile bedeutungslos - sie wurden
    // gegen einen anderen Vergleichspunkt gelernt. Deshalb verwerfen.
    for(int i=0;i<3;i++) { speedSig[i].valid = false; saveSpeedCalibration(i); }
    for(int i=0;i<3;i++) { modeSig[i].valid  = false; saveModeCalibration(i); }
    for(int sp=0;sp<3;sp++) for(int md=0;md<3;md++) comboSig[sp][md].valid = false;
    comboWValid = false;
    comboShape  = 0.0f;
    for(int ch=0;ch<5;ch++) comboW[ch] = 1.0f;
    saveComboCalibration();
    // Auch das AUS-Profil wurde gegen die alte Schwelle gemessen.
    offSig.valid = false;
    saveOffCalibration();

    Serial.printf("[SCHWELLE] neu: %d %d %d %d %d - Profile verworfen\n",
                  neu[0],neu[1],neu[2],neu[3],neu[4]);
    server.send(200,"application/json","{\"ok\":true}");
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

  // OTA per Browser - mit Passwortschutz und echter Fehlerpruefung.
  server.on("/update",HTTP_POST,
    [](){
      if(!server.authenticate(OTA_USER,OTA_PASS)) return server.requestAuthentication();
      bool ok = !Update.hasError();
      server.sendHeader("Connection","close");
      server.send(ok?200:500,"text/plain",
                  ok ? "OK - Neustart" : "Update fehlgeschlagen - alte Firmware bleibt aktiv");
      Serial.printf("[OTA] Ergebnis: %s\n", ok ? "ok" : "FEHLER");
      if(ok){ delay(400); ESP.restart(); }
    },
    [](){
      HTTPUpload& up = server.upload();

      static bool otaAllowed = false;

      if(up.status==UPLOAD_FILE_START) {
        otaAllowed = server.authenticate(OTA_USER,OTA_PASS);
        if(!otaAllowed) { Serial.println("[OTA] Abgelehnt: falsche Zugangsdaten"); return; }
        Serial.printf("[OTA] Start: %s\n", up.filename.c_str());
        if(!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          otaAllowed = false;
        }
      }
      else if(!otaAllowed) {
        return;               // ohne gueltigen Start wird nichts geschrieben
      }
      else if(up.status==UPLOAD_FILE_WRITE) {
        // Kurzer Write bedeutet: Flash voll oder Fehler. Dann sofort abbrechen,
        // sonst landet ein halbes Image in der Partition.
        if(Update.write(up.buf,up.currentSize) != up.currentSize) {
          Update.printError(Serial);
          Update.abort();
        }
      }
      else if(up.status==UPLOAD_FILE_END) {
        if(!Update.end(true)) {
          Update.printError(Serial);
        } else {
          Serial.printf("[OTA] %u Bytes geschrieben\n", up.totalSize);
        }
        otaAllowed = false;
      }
      else if(up.status==UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaAllowed = false;
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n\n=== Smart-Ventilator %s ===\nBuild: %s %s\nReset-Grund: %d\n",
                FW_VERSION, __DATE__, __TIME__, (int)esp_reset_reason());

  loadFanState();
  Serial.printf("[BOOT] gemerkt: Stufe %u, Drehen %s, Modus %u\n",
                state.lastSpeed, state.lastOsc?"an":"aus", state.lastMode);

  for(uint8_t p:{PIN_POWER,PIN_SPEED,PIN_OSC,PIN_MODE}) {
    pinMode(p,OUTPUT);
    digitalWrite(p,LOW);
  }

#if ENABLE_LED_FEEDBACK
  loadFeedbackThresholds();
  loadCalibration();
  loadOffCalibration();
  loadComboCalibration();
  analogReadResolution(12);

  // WICHTIG fuer die gelernten Profile: Aufloesung UND Messbereich explizit
  // festnageln. Aendert ein kuenftiges Core-Update den Default, verschieben
  // sich sonst alle ADC-Werte und die Kalibrierung ist wertlos.
  for(uint8_t p : FB_USED_PINS) {
    pinMode(p, INPUT);
    analogSetPinAttenuation(p, ADC_11db);   // ca. 0 .. 3,1 V
  }
#endif

  startNetwork();
  setupRoutes();
  server.begin();
}

void loop() {
  server.handleClient();
  analyzeStep();

  // Eigener Timer: Taste4 des Ventilators bleibt komplett unbenutzt.
  if(timerDeadlineMs && (long)(millis()-timerDeadlineMs)>=0) {
    timerDeadlineMs=0;
    state.sleepMin=0;
    Serial.println("[TIMER] abgelaufen - schalte ab");
    powerOff();
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
