/*
 * File:        controller.ino
 *
 * Author: Rambod Taherian, Kourosh Ghahramani
 * Date: 2025-08-22
 *
 * Description:
 *   Controller firmware for a dual-joystick + LCD UI that drives a servo-mask over ESP-NOW.
 *   Left stick (NAV) navigates menus/FSM. LEFT push-button: Hold toggles Game/Free
 *   mode; Single-tap triggers Random (Free Play) or starts game/selects difficulty (Game mode).
 *   RIGHT push-button: Double-tap toggles the persistent Preview overlay.
 *
 *   Joystick orientations are hardware-rotated: NAV is rotated 90° CW; CTRL is
 *   rotated 90° CCW. Code maps raw ADCs accordingly. LCD provides status/HUD.
 *   ESP-NOW packets carry MOVE, MODE, and GAME control with CRC16-LE integrity.
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_crc.h>
#include <string.h>
#include <limits.h>

/* =============================== Macros / Constants =============================== */

#define N_SERVO 14

/* ---- Mask MAC (peer) ---- */
static const uint8_t MASK_MAC[6] = {0xFC,0x01,0x2C,0xDB,0xEB,0x88};

/* ---- ADC Pins (note: sticks are physically swapped/rotated) ---- */
// LEFT  stick = CTRL (movement)
#define CTRL_X_PIN  5
#define CTRL_Y_PIN  4
// RIGHT stick = NAV  (FSM/menu)
#define NAV_X_PIN  14
#define NAV_Y_PIN  13

/* ---- Button Pins (active-LOW) ---- */
#define MODE_BTN_PIN     12  // LEFT stick push: Hold=Mode toggle, single-tap=Random (Free Play)
#define PREVIEW_BTN_PIN   6  // RIGHT stick push: Double-tap toggles Preview overlay

/* ---- Buzzer (LEDC PWM) ---- */
#define BUZZER_PIN 10
#define BUZZ_FREQ  4000
#define BUZZ_RES   10

LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ============================== Protocol Definitions ============================== */

/* Message families */
enum : uint8_t { MSG_MOVE=1, MSG_SYNC_REQ=2, MSG_SYNC_RESP=3, MSG_MODE=4, MSG_RANDOM=5, MSG_GAME=6 };

/* Mode commands */
enum : uint8_t { MODE_TOGGLE=1, MODE_SET_FREE=2, MODE_SET_GAME=3 };

/* Game commands */
enum : uint8_t {
  GAME_CFG=1, GAME_START=2, GAME_ABORT=3, GAME_STATE=4, GAME_FACE_DONE=5, GAME_RESULT=6,
  GAME_PREVIEW=7
};

/* Preview modes (applied by mask side) */
enum : uint8_t { PREV_MID=0, PREV_MIN=1, PREV_MAX=2 };

/* ---- Packed wire formats (CRC16-LE at end) ---- */
struct __attribute__((packed)) PktMove    { uint8_t type, idx; int16_t us, xOff; uint16_t crc; };
struct __attribute__((packed)) PktSyncReq { uint8_t type; uint16_t crc; };
struct __attribute__((packed)) PktSyncResp{ uint8_t type; int16_t pos[N_SERVO]; uint16_t crc; };
struct __attribute__((packed)) PktMode    { uint8_t type, cmd, arg; uint16_t crc; };
struct __attribute__((packed)) PktRandom  { uint8_t type; uint16_t crc; };
struct __attribute__((packed)) PktGame    { uint8_t type, cmd, a, b; uint16_t crc; };

/* ============================== Servo Limits / Directions ============================== */

struct Limits { int16_t minUs, maxUs; };
static Limits lim[N_SERVO] = {
  { 960,1065},{1041,2248},{2015,2160},{1006,2111},
  {1039,1600},{ 890,1510},{ 981,2200},{1557,2180},
  {1280,1871},{1139,2400},{1216,1750},{ 660,2400},
  { 500,2060},{ 500,2400}
};

/* DIR: +1 means increasing microseconds moves positive UI direction; −1 inverts. */
static const int8_t DIR[N_SERVO] = {
  +1,+1,-1,-1, +1,-1, +1,+1,+1,-1, +1,+1,-1,+1
};

/* ==================================== FSM (Free Play) ==================================== */

/* States correspond to controllable features; NAV stick moves between them. */
enum State : uint8_t {
  S_L_BROW_V=0, S_L_BROW_A, S_R_BROW_V, S_R_BROW_A,
  S_L_EYELID,  S_R_EYELID,  S_L_EYE,     S_R_EYE,
  S_L_MOUTH,   S_R_MOUTH,   S_JAW,
  S_COUNT
};

/* Short names for LCD/HUD */
static const char* STATE_NAME[S_COUNT] = {
  "L eyebrow V","L eyebrow A","R eyebrow V","R eyebrow A",
  "L eyelid","R eyelid","L eye","R eye","L mouth","R mouth","Jaw"
};

/* Mapping of single-channel states to servo index; eyes use a pair (handled separately). */
static const int8_t STATE_CH_SINGLE[S_COUNT] = {
  0,1,2,3, 9,6, -1,-1, 12,11, 10
};

/* NAV directions emitted on edge (not continuous) */
enum NavDir : uint8_t { ND_NONE=0, ND_UP, ND_DOWN, ND_LEFT, ND_RIGHT };

/* ============================== Shared State / Tunables ============================== */

static volatile bool gSynced=false;    // true after first valid SYNC_RESP
static volatile bool gGame=false;      // false on boot → Free Play
static int16_t  gPos[N_SERVO];         // last-known microsecond targets (guarded by posMtx)
static State    gState = S_L_BROW_V;   // current Free Play selection

/* Joystick dead zones and motion scaling */
const int   DEAD_CTRL = 600;
const int   DEAD_NAV  = 600;
const float KMAX      = 0.005f;        // max step gain per 20 ms tick (scaled by |offset|^2)

/* RX queue + mutex (mutex used; queue placeholder for ISR-safe path if needed) */
struct RxMsg { uint8_t len; uint8_t bytes[sizeof(PktSyncResp)]; };
static QueueHandle_t rxq;
static SemaphoreHandle_t posMtx;

/* ============================== Game / Preview Overlay State ============================== */

enum UiStage : uint8_t { UI_FREEHUD=0, UI_GAME_MENU, UI_GAME_ARM, UI_GAME_RUN, UI_GAME_END, UI_PREVIEW };
static volatile UiStage uiStage = UI_FREEHUD;

enum Diff : uint8_t { DIFF_EASY=0, DIFF_MED=1, DIFF_HARD=2 };
struct DiffCfg { uint8_t faces; uint8_t secPerFace; char tag; };
static const DiffCfg DIFFS[] = {
  {3,60,'E'},  // Easy
  {4,40,'M'},  // Medium
  {5,20,'H'}   // Hard
};
static volatile Diff diffSel = DIFF_EASY;
static volatile uint8_t facesDone=0, facesTotal=0;
static volatile uint8_t timeRemain=0;
static volatile uint8_t faceIndex=0;
static volatile bool    gameWon=false, gameLost=false;
static volatile bool    gameEndLatched=false;

static uint32_t toastUntilMs = 0;  // brief “toast” on mode/preview toggles

/* Preview selection persists across toggles (sticky) */
static volatile uint8_t previewFace = 0;
static volatile uint8_t previewMode = PREV_MID;
static const uint8_t NUM_FACES_CTRL = 5;
static const char* FACE_NAME_CTRL[NUM_FACES_CTRL] = {
  "Poker","Happy","Angry","Sad","Shocked"
};

/* ============================== Forward Declarations ============================== */

static NavDir readNavDirOnce();
static State  fsmNext(State s, NavDir d);

static inline int16_t clampUs(uint8_t i, int16_t us);
static inline void   sendMoveTo(uint8_t idx, int16_t us);
static inline void   sendSyncReq();
static inline void   sendModeToggle();
static inline void   sendRandom();
static inline void   sendGame(uint8_t cmd, uint8_t a=0, uint8_t b=0);

static void onRx(const esp_now_recv_info_t*, const uint8_t* d, int len);

static void TaskRx(void*);
static void TaskSync(void*);
static void TaskControl(void*);
static void TaskLCD(void*);

static void lcdWriteRow(uint8_t row, const char* text);
static void lcdWriteRowLR(uint8_t row, const char* left, const char* right);

/* ============================== Helpers: Packets / Bounds ============================== */

/** clampUs
 *  Clamp a target microsecond value to that servo’s allowed [min,max] range.
 *  param i:  Servo index.
 *  param us: Desired microseconds.
 *  return:   Clamped microseconds.
 */
static inline int16_t clampUs(uint8_t i, int16_t us){
  return constrain(us, lim[i].minUs, lim[i].maxUs);
}

/** sendMoveTo
 *  Send a MOVE packet to set servo idx to absolute microseconds.
 *  xOff is reserved (kept 0 here; could be used for velocity/offset features later).
 */
static inline void sendMoveTo(uint8_t idx, int16_t us){
  PktMove p{}; p.type=MSG_MOVE; p.idx=idx; p.us=us; p.xOff=0;
  p.crc=esp_crc16_le(0,(uint8_t*)&p,sizeof(p)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&p,sizeof(p));
}

/** sendSyncReq
 *  Request a SYNC_RESP containing current servo microseconds (mask → controller).
 */
static inline void sendSyncReq(){
  PktSyncReq r{}; r.type=MSG_SYNC_REQ;
  r.crc=esp_crc16_le(0,(uint8_t*)&r,sizeof(r)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&r,sizeof(r));
}

/** sendModeToggle
 *  Toggle mask mode (Free/Game). Controller also mirrors local gGame and HUD.
 */
static inline void sendModeToggle(){
  PktMode m{}; m.type=MSG_MODE; m.cmd=MODE_TOGGLE; m.arg=0;
  m.crc = esp_crc16_le(0,(uint8_t*)&m,sizeof(m)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&m,sizeof(m));
}

/** sendRandom
 *  Ask mask to randomize channels (Free Play single-tap behavior).
 */
static inline void sendRandom(){
  PktRandom m{}; m.type=MSG_RANDOM;
  m.crc = esp_crc16_le(0,(uint8_t*)&m,sizeof(m)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&m,sizeof(m));
}

/** sendGame
 *  Send a generic GAME command (config/start/abort/state/preview/etc).
 *  param cmd: GAME_* enumerant.
 *  param a:   Command-specific first byte.
 *  param b:   Command-specific second byte.
 */
static inline void sendGame(uint8_t cmd, uint8_t a, uint8_t b){
  PktGame g{}; g.type=MSG_GAME; g.cmd=cmd; g.a=a; g.b=b;
  g.crc=esp_crc16_le(0,(uint8_t*)&g,sizeof(g)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&g,sizeof(g));
}

/* ============================== ESP-NOW Receive Callback ============================== */

/** onRx
 *  Process inbound ESP-NOW frames. Validates CRC and updates shared state.
 *  - SYNC_RESP: updates gPos[] under mutex and sets gSynced.
 *  - GAME_*: updates HUD counters; latches result once per round (buzzer feedback).
 */
static void onRx(const esp_now_recv_info_t*, const uint8_t* d, int len){
  if(len<=0 || len>(int)sizeof(((RxMsg*)0)->bytes)) return;

  if (len==(int)sizeof(PktSyncResp) && d[0]==MSG_SYNC_RESP){
    const PktSyncResp* r=(const PktSyncResp*)d;
    if (esp_crc16_le(0,(uint8_t*)r,sizeof(PktSyncResp)-2)==r->crc){
      if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(5))==pdTRUE){
        for(int i=0;i<N_SERVO;++i) gPos[i]=clampUs(i, r->pos[i]);
        xSemaphoreGive(posMtx);
      }
      gSynced=true;
    }
    return;
  }

  if (len>=(int)sizeof(PktGame) && d[0]==MSG_GAME){
    const PktGame* g=(const PktGame*)d;
    if (esp_crc16_le(0,(uint8_t*)g,sizeof(PktGame)-2)!=g->crc) return;

    if (g->cmd==GAME_STATE){ timeRemain=g->a; faceIndex=g->b; }
    else if (g->cmd==GAME_FACE_DONE){
      facesDone=g->a; facesTotal=g->b;
      ledcWrite(BUZZER_PIN, 512); delay(120); ledcWrite(BUZZER_PIN,0);
    } else if (g->cmd==GAME_RESULT){
      if (gameEndLatched) return;  // only first result counts
      gameEndLatched=true;
      if (g->a==1){                 // win
        gameWon=true; gameLost=false;
        for(int i=0;i<3;i++){ ledcWrite(BUZZER_PIN,512); delay(120); ledcWrite(BUZZER_PIN,0); delay(80); }
      } else {                      // loss
        gameWon=false; gameLost=true;
        ledcWrite(BUZZER_PIN,512); delay(300); ledcWrite(BUZZER_PIN,0);
      }
      uiStage=UI_GAME_END;
    }
  }
}

/* ============================== NAV stick (RIGHT, +90° CW) ============================== */

/** readNavDirOnce
 *  Read NAV stick and emit a direction ONLY on edge (press → hold suppressed) to
 *  avoid repeated transitions while held. Accounts for +90° CW rotation:
 *    navX = +rawY, navY = -rawX (here we derive x,y directly to pick dominant axis).
 *  return: ND_* direction or ND_NONE if no new edge.
 */
static NavDir readNavDirOnce(){
  static NavDir prev = ND_NONE;
  int16_t rawX = analogRead(NAV_X_PIN) - 2048;
  int16_t rawY = analogRead(NAV_Y_PIN) - 2048;

  // +90° CW mapping: x=rawY, y=rawX (signs chosen so up/right feel natural)
  int16_t x =  rawY;
  int16_t y =  rawX;

  NavDir dir = ND_NONE;
  int ax = abs(x), ay = abs(y);

  // choose dominant axis above dead zone
  if (ay >= DEAD_NAV || ax >= DEAD_NAV){
    dir = (ay >= ax) ? ((y>0)?ND_UP:ND_DOWN) : ((x>0)?ND_RIGHT:ND_LEFT);
  }

  // Edge emit: only when coming from NONE
  NavDir emit = ND_NONE;
  if (dir != ND_NONE && prev == ND_NONE) emit = dir;
  prev = (dir==ND_NONE)?ND_NONE:dir;
  return emit;
}

/* ============================== FSM Transitions ============================== */

/** fsmNext
 *  Given current Free Play state and a NAV edge direction, compute the next state.
 *  Custom hops: Brows UP from any eyebrow goes to JAW; mouth toward center goes JAW.
 *  param s: Current state.
 *  param d: NAV edge direction.
 *  return:  Next state (or same if ND_NONE).
 */
static State fsmNext(State s, NavDir d){
  if (d==ND_NONE) return s;
  switch (s){
    /* Eyebrows: UP → JAW  */
    case S_L_BROW_V: if(d==ND_UP)return S_JAW; if(d==ND_LEFT)return S_L_BROW_A; if(d==ND_RIGHT)return S_R_BROW_V; if(d==ND_DOWN)return S_L_EYELID; return s;
    case S_L_BROW_A: if(d==ND_UP)return S_JAW; if(d==ND_RIGHT)return S_L_BROW_V; if(d==ND_DOWN)return S_L_EYELID; if(d==ND_LEFT)return S_R_BROW_A; return s;
    case S_R_BROW_V: if(d==ND_UP)return S_JAW; if(d==ND_LEFT)return S_L_BROW_V; if(d==ND_RIGHT)return S_R_BROW_A; if(d==ND_DOWN)return S_R_EYELID; return s;
    case S_R_BROW_A: if(d==ND_UP)return S_JAW; if(d==ND_LEFT)return S_R_BROW_V; if(d==ND_DOWN)return S_R_EYELID; if(d==ND_RIGHT)return S_L_BROW_A; return s;

    case S_L_EYELID: if(d==ND_UP)return S_L_BROW_V; if(d==ND_RIGHT)return S_R_EYELID; if(d==ND_DOWN)return S_L_EYE; if(d==ND_LEFT)return S_R_EYELID; return s;
    case S_R_EYELID: if(d==ND_UP)return S_R_BROW_V; if(d==ND_DOWN)return S_R_EYE; if(d==ND_LEFT)return S_L_EYELID; if(d==ND_RIGHT)return S_L_EYELID; return s;
    case S_L_EYE:    if(d==ND_UP)return S_L_EYELID; if(d==ND_DOWN)return S_L_MOUTH; if(d==ND_RIGHT)return S_R_EYE; if(d==ND_LEFT)return S_R_EYE; return s;
    case S_R_EYE:    if(d==ND_UP)return S_R_EYELID; if(d==ND_DOWN)return S_R_MOUTH; if(d==ND_LEFT)return S_L_EYE; if(d==ND_RIGHT)return S_L_EYE; return s;

    /* Mouth rules: toward center goes to JAW */
    case S_L_MOUTH:
      if(d==ND_UP)return S_L_EYE;
      if(d==ND_DOWN)return S_JAW;
      if(d==ND_RIGHT)return S_JAW;      // to center
      if(d==ND_LEFT)return S_R_MOUTH;   // outward hop preserved
      return s;
    case S_R_MOUTH:
      if(d==ND_UP)return S_R_EYE;
      if(d==ND_DOWN)return S_JAW;
      if(d==ND_LEFT)return S_JAW;       // to center
      if(d==ND_RIGHT)return S_L_MOUTH;  // outward hop preserved
      return s;

    /* Jaw rules including UP→Left Mouth */
    case S_JAW:
      if(d==ND_UP)   return S_L_MOUTH;
      if(d==ND_DOWN) return S_L_BROW_V;
      if(d==ND_LEFT) return S_L_MOUTH;
      if(d==ND_RIGHT)return S_R_MOUTH;
      return s;

    default: return s;
  }
}

/* ============================== FreeRTOS Tasks ============================== */

/** TaskRx
 *  Placeholder task to process queued RX frames if onRx() is converted to enqueue.
 */
static void TaskRx(void*){
  RxMsg m;
  for(;;){
    if (xQueueReceive(rxq,&m,portMAX_DELAY)!=pdTRUE) continue;
    onRx(nullptr, m.bytes, m.len);
  }
}

/** TaskSync
 *  Periodically requests SYNC until first valid response (sets gSynced).
 */
static void TaskSync(void*){
  const TickType_t period=pdMS_TO_TICKS(250);
  for(;;){ if(!gSynced) sendSyncReq(); vTaskDelay(period); }
}

/** TaskControl
 *  Main input loop at 50 Hz (20 ms tick). Handles:
 *    - LEFT button (MODE/RANDOM): debounce, hold (>=300 ms) toggles mode, tap acts
 *      depending on uiStage (Random in Free; arm/start progression in Game).
 *    - RIGHT button (PREVIEW): double-tap (<=350 ms between releases) toggles overlay.
 *    - RIGHT stick (NAV): emits edge directions to drive FSM or difficulty selection.
 *    - LEFT stick (CTRL): moves channels (or browses Preview face/mode when overlay on).
 *
 *  Movement uses a squared magnitude scaling (|offset|^2) and per-servo DIR[] to
 *  keep fine control near center and faster far from center, clamped to Limits.
 */
static void TaskControl(void*){
  const TickType_t period=pdMS_TO_TICKS(16);
  TickType_t next=xTaskGetTickCount();

  /* LEFT (MODE/Random) button debounce/timing */
  static bool rawMode=false, debMode=false, prevMode=false;
  static uint32_t modeLastChg=0, modePressStart=0;
  static bool modeSentThisHold=false;
  const uint32_t DEBOUNCE_MS=30, HOLD_MS=300;

  /* RIGHT (PREVIEW) button debounce/timing (double-tap window) */
  static bool rawPrev=false, debPrev=false, prevPrev=false;
  static uint32_t prevLastChg=0, prevReleaseMs=0;
  const uint32_t DOUBLE_TAP_MS=350;

  for(;;){
    /* ===== LEFT button (MODE/Random) ===== */
    bool rMode = (digitalRead(MODE_BTN_PIN) == LOW);
    if (rMode != rawMode){ rawMode=rMode; modeLastChg=millis(); }
    if (millis()-modeLastChg >= DEBOUNCE_MS) debMode = rawMode;

    // Press start edge
    if (!prevMode && debMode){ modePressStart=millis(); modeSentThisHold=false; }

    // Hold action: toggle Game/Free once per hold
    if (debMode && !modeSentThisHold && (millis()-modePressStart)>=HOLD_MS){
      gGame = !gGame; sendModeToggle(); modeSentThisHold = true;
      toastUntilMs = millis() + 2000;
      uiStage = gGame ? UI_GAME_MENU : UI_FREEHUD;
      gameEndLatched=false; gameWon=false; gameLost=false;
    }

    // Release edge: tap actions (depends on current stage)
    if (prevMode && !debMode){
      uint32_t dur = millis()-modePressStart;
      if (dur < HOLD_MS && !modeSentThisHold){
        if (!gGame){
          // Free Play: single-tap triggers Random on the mask
          sendRandom();
        } else {
          // Game mode tap progression: MENU → ARM → RUN; END → MENU
          if (uiStage==UI_GAME_MENU){
            uiStage = UI_GAME_ARM;
          } else if (uiStage==UI_GAME_ARM){
            const DiffCfg& d = DIFFS[diffSel];
            facesDone=0; facesTotal=d.faces; timeRemain=d.secPerFace; faceIndex=0;
            gameEndLatched=false; gameWon=false; gameLost=false;
            sendGame(GAME_CFG, d.faces, d.secPerFace);
            sendGame(GAME_START, 0, 0);
            uiStage = UI_GAME_RUN;
          } else if (uiStage==UI_GAME_END){
            uiStage = UI_GAME_MENU;
          }
        }
      }
    }
    prevMode = debMode;

    /* ===== NAV stick (RIGHT) → FSM/menu ===== */
    NavDir nav = readNavDirOnce();
    if (!gGame || uiStage==UI_GAME_RUN){
      if (nav != ND_NONE) gState = fsmNext(gState, nav);
    } else if (uiStage==UI_GAME_MENU){
      // Difficulty carousel on left/right
      if (nav==ND_LEFT){
        diffSel = (diffSel==DIFF_EASY)? DIFF_HARD : (Diff)(diffSel-1);
      } else if (nav==ND_RIGHT){
        diffSel = (diffSel==DIFF_HARD)? DIFF_EASY : (Diff)(diffSel+1);
      }
    }

    /* ===== RIGHT button (PREVIEW) double-tap ===== */
    bool rPrev = (digitalRead(PREVIEW_BTN_PIN) == LOW);
    if (rPrev != rawPrev){ rawPrev=rPrev; prevLastChg=millis(); }
    if (millis()-prevLastChg >= DEBOUNCE_MS) debPrev = rawPrev;

    if (prevPrev && !debPrev){
      // release → check double-tap window
      uint32_t now = millis();
      bool isDouble = (now - prevReleaseMs) <= DOUBLE_TAP_MS;

      if (!gGame){ // Preview only in Free Play contexts
        if (isDouble){
          // Toggle Preview WITHOUT resetting sticky face/mode
          if (uiStage != UI_PREVIEW){
            uiStage = UI_PREVIEW;
            sendGame(GAME_PREVIEW, previewFace, previewMode);
            ledcWrite(BUZZER_PIN, 300); delay(60); ledcWrite(BUZZER_PIN,0);
            toastUntilMs = now + 1200;
          } else {
            uiStage = UI_FREEHUD;
            ledcWrite(BUZZER_PIN, 200); delay(40); ledcWrite(BUZZER_PIN,0);
            toastUntilMs = now + 800;
          }
        }
      }
      prevReleaseMs = now;
    }
    prevPrev = debPrev;

    /* ===== CTRL stick (LEFT) → either Preview browsing OR servo movement ===== */

    // Raw ADC centered at ~2048 (12-bit). Apply 90° CCW mapping:
    //   ctrlX = -rawCY, ctrlY = +rawCX
    int16_t rawCX = analogRead(CTRL_X_PIN) - 2048;
    int16_t rawCY = analogRead(CTRL_Y_PIN) - 2048;
    int16_t xOff = -rawCY;
    int16_t yOff =  rawCX;

    // In PREVIEW: browse faces (X) and preview mode (Y) with repeat delay
    if (uiStage == UI_PREVIEW){
      static uint32_t lastMoveMs = 0;
      const uint32_t REPEAT_MS = 210;
      uint32_t now = millis();
      bool changed = false;

      if (abs(xOff) >= DEAD_CTRL && (now - lastMoveMs) >= REPEAT_MS){
        if (xOff > 0) previewFace = (previewFace + 1) % NUM_FACES_CTRL;
        else          previewFace = (previewFace == 0) ? (NUM_FACES_CTRL-1) : (previewFace-1);
        changed = true; lastMoveMs = now;
        ledcWrite(BUZZER_PIN, 180); delay(25); ledcWrite(BUZZER_PIN,0);
      }
      if (abs(yOff) >= DEAD_CTRL && (now - lastMoveMs) >= REPEAT_MS){
        if (yOff > 0) previewMode = (previewMode + 1) % 3;
        else          previewMode = (previewMode == 0) ? 2 : (previewMode - 1);
        changed = true; lastMoveMs = now;
        ledcWrite(BUZZER_PIN, 240); delay(25); ledcWrite(BUZZER_PIN,0);
      }
      if (changed){
        sendGame(GAME_PREVIEW, previewFace, previewMode);
      }
      vTaskDelayUntil(&next, period);
      continue; // skip movement while overlay is active
    }

    // Normal movement path (Free HUD or Game RUN): eyes move 2 axes; others single
    if (gState==S_L_EYE || gState==S_R_EYE){
      if (abs(xOff)>=DEAD_CTRL || abs(yOff)>=DEAD_CTRL){
        float gx = KMAX * sq(abs(xOff)/2048.0f);
        float gy = KMAX * sq(abs(yOff)/2048.0f);
        uint8_t chX = (gState==S_R_EYE)?4:7;
        uint8_t chY = (gState==S_R_EYE)?5:8;

        if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(2))==pdTRUE){
          if (abs(xOff)>=DEAD_CTRL){
            int16_t tmpX = gPos[chX] + int16_t(DIR[chX] * gx * xOff);
            gPos[chX] = clampUs(chX, tmpX);
            sendMoveTo(chX, gPos[chX]);
          }
          if (abs(yOff)>=DEAD_CTRL){
            int16_t tmpY = gPos[chY] + int16_t(DIR[chY] * gy * yOff);
            gPos[chY] = clampUs(chY, tmpY);
            sendMoveTo(chY, gPos[chY]);
          }
          xSemaphoreGive(posMtx);
        }
      }
    } else {
      int8_t ch = STATE_CH_SINGLE[gState];
      if (ch >= 0 && abs(yOff)>=DEAD_CTRL){
        float gain = KMAX * sq(abs(yOff)/2048.0f);
        if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(2))==pdTRUE){
          int16_t tmp = gPos[ch] + int16_t(DIR[ch] * gain * yOff);
          gPos[ch] = clampUs(ch, tmp);
          int16_t us = gPos[ch];
          xSemaphoreGive(posMtx);
          sendMoveTo(ch, us);
        }
      }
    }

    vTaskDelayUntil(&next, period);
  }
}

/* ============================== LCD Helpers / HUD Task ============================== */

/** lcdWriteRow
 *  Write a left-justified string to a 16-char row; blanks pad the remainder.
 */
static void lcdWriteRow(uint8_t row, const char* text){
  char buf[17]; memset(buf,' ',16); buf[16]='\0';
  size_t n = strlen(text); if (n>16) n=16;
  memcpy(buf, text, n);
  lcd.setCursor(0,row); lcd.print(buf);
}

/** lcdWriteRowLR
 *  Write "left ... right" into a 16-char row (left-justified left, right-justified right).
 */
static void lcdWriteRowLR(uint8_t row, const char* left, const char* right){
  char buf[17]; memset(buf,' ',16); buf[16]='\0';
  size_t ll = strlen(left); if (ll>16) ll=16;
  memcpy(buf, left, ll);
  size_t rl = strlen(right); if (rl>16) rl=16;
  memcpy(buf + (16-rl), right, rl);
  lcd.setCursor(0,row); lcd.print(buf);
}

/** TaskLCD
 *  Renders HUD:
 *    - Toasts during brief transitions (Preview toggle, Mode toggle).
 *    - "Syncing..." until first SYNC_RESP is processed.
 *    - Free HUD: shows current state and its microseconds (or X/Y for eyes).
 *    - Game MENU/ARM/RUN/END screens with difficulty selection and progress.
 *    - Preview overlay: shows face index/name and preview mode (MIN/MID/MAX).
 */
static void TaskLCD(void*){
  State prevState = (State)255;
  bool  prevGame  = false;
  Diff  prevDiff  = DIFF_EASY;
  UiStage prevStage = UI_FREEHUD;

  for(;;){
    // Toast banner (short-lived)
    if (toastUntilMs && millis() < toastUntilMs){
      lcdWriteRow(0, (uiStage==UI_PREVIEW) ? "Preview Mode" : (gGame ? "Game Mode" : "Free Play"));
      lcdWriteRow(1, "                ");
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    } else if (toastUntilMs && millis() >= toastUntilMs){ toastUntilMs = 0; }

    // Waiting for first sync
    if (!gSynced){
      lcdWriteRow(0, "Syncing...");
      lcdWriteRow(1, "                ");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Preview overlay HUD
    if (uiStage == UI_PREVIEW){
      const char* modeName = (previewMode==PREV_MID)?"MID":(previewMode==PREV_MIN)?"MIN":"MAX";
      char top[17];  snprintf(top, sizeof(top), "Preview %u/%u", (unsigned)(previewFace+1), (unsigned)NUM_FACES_CTRL);
      lcdWriteRow(0, top);
      const char* fname = FACE_NAME_CTRL[previewFace];
      char shortName[10]; strncpy(shortName, fname, 9); shortName[9]='\0';
      char bot[17]; snprintf(bot, sizeof(bot), "%-9s %s", shortName, modeName);
      lcdWriteRow(1, bot);
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // Free HUD
    if (!gGame || uiStage==UI_FREEHUD){
      if (gState != prevState || gGame != prevGame || uiStage!=prevStage){
        char tag[2] = {'F','\0'};
        lcdWriteRowLR(0, STATE_NAME[gState], tag);
        prevState=gState; prevGame=gGame; prevStage=uiStage;
      }
      if (gState==S_L_EYE || gState==S_R_EYE){
        int16_t ux=0, uy=0;
        if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(2))==pdTRUE){
          ux = gPos[(gState==S_R_EYE)?4:7];
          uy = gPos[(gState==S_R_EYE)?5:8];
          xSemaphoreGive(posMtx);
        }
        char row1[17]; snprintf(row1,sizeof(row1),"X:%4d Y:%4d", ux, uy);
        lcdWriteRow(1, row1);
      } else {
        int8_t ch = STATE_CH_SINGLE[gState];
        int16_t us=0; if (ch>=0 && xSemaphoreTake(posMtx,pdMS_TO_TICKS(2))==pdTRUE){ us=gPos[ch]; xSemaphoreGive(posMtx); }
        char row1[17]; snprintf(row1,sizeof(row1),"us:%4d", us);
        lcdWriteRow(1, row1);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Game: difficulty menu
    if (uiStage==UI_GAME_MENU){
      const char* name = (diffSel==DIFF_EASY)?"Easy":(diffSel==DIFF_MED)?"Medium":"Hard";
      lcdWriteRow(0, "Select Difficulty");
      char row1[17]; snprintf(row1,sizeof(row1),"< %s >", name);
      lcdWriteRow(1, row1);
      prevStage=uiStage; prevDiff=diffSel;
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // Game: arm screen
    if (uiStage==UI_GAME_ARM){
      lcdWriteRow(0, "Press btn to start");
      const DiffCfg& d = DIFFS[diffSel];
      char r1[17]; snprintf(r1,sizeof(r1),"%u faces %us", d.faces, d.secPerFace);
      lcdWriteRow(1, r1);
      prevStage=uiStage; prevDiff=diffSel;
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // Game: run screen (shows timer and flips between face label and progress)
    if (uiStage==UI_GAME_RUN){
      char tbuf[4]; snprintf(tbuf,sizeof(tbuf),"%02u",(unsigned)timeRemain);
      lcdWriteRowLR(0, STATE_NAME[gState], tbuf);
      static uint32_t flipT=0; static bool flip=false;
      if (millis()-flipT>700){ flip=!flip; flipT=millis(); }
      if (flip){
        static const char* FACE_NAME[]={"Poker","Happy","Angry","Sad","Shocked"};
        char b[17]; snprintf(b,sizeof(b),"Face %s", FACE_NAME[faceIndex % 7]);
        lcdWriteRow(1, b);
      } else {
        char pb[17]; snprintf(pb,sizeof(pb),"Done %u/%u",(unsigned)facesDone,(unsigned)facesTotal);
        lcdWriteRow(1, pb);
      }
      vTaskDelay(pdMS_TO_TICKS(80));
      continue;
    }

    // Game: end screen
    if (uiStage==UI_GAME_END){
      lcdWriteRow(0, gameWon ? "GAME WON!" : "GAME LOST!");
      lcdWriteRow(1, "Press btn restart");
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }
  }
}

/* ================================= Arduino setup/loop ================================= */

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(MODE_BTN_PIN,    INPUT_PULLUP);
  pinMode(PREVIEW_BTN_PIN, INPUT_PULLUP);

  Wire.begin(20, 21);
  lcd.init(); lcd.backlight();
  lcdWriteRow(0, "Syncing...");
  lcdWriteRow(1, "                ");

  // Buzzer PWM
  ledcAttach(BUZZER_PIN, BUZZ_FREQ, BUZZ_RES);
  ledcWrite(BUZZER_PIN, 0);

  // ESP-NOW init & peer
  WiFi.mode(WIFI_STA);
  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init fail"); for(;;); }
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr,MASK_MAC,6);
  peer.ifidx=WIFI_IF_STA; peer.channel=0; peer.encrypt=false;
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onRx);

  // Concurrency primitives
  rxq   = xQueueCreate(8,sizeof(RxMsg));
  posMtx= xSemaphoreCreateMutex();

  // Initialize positions safely within limits
  for(int i=0;i<N_SERVO;++i) gPos[i] = constrain(1500, lim[i].minUs, lim[i].maxUs);
  gState = S_L_BROW_V;
  gGame  = false;
  uiStage= UI_FREEHUD;

  sendSyncReq();

  // Tasks pinned to core 1
  xTaskCreatePinnedToCore(TaskRx,      "rx",      4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskSync,    "sync",    2048, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskControl, "control", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskLCD,     "lcd",     3072, nullptr, 1, nullptr, 1);

  Serial.println("Controller ready: LEFT stick moves (rotated CCW); RIGHT stick navigates (rotated CW).");
  Serial.println("Buttons: LEFT push=Hold:Mode / Tap:Random, RIGHT push=Double-tap:Preview.");
}

void loop(){}  // not used, all work is in FreeRTOS tasks
