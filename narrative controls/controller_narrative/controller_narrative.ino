/* -----------------------------------------------------------------------------
  File:        controller_narrative.ino
 *
 * Author:Rambod Taherian, Kourosh Ghahramani
 * Date:        2025-08-23
 *
 * Description:
 * This program runs on the ESP32-S3 handheld controller for the expressive
 * robotic mask. It manages dual joysticks, buttons, LCD feedback, and wireless
 * communication with the mask over ESP-NOW. The controller has been extended
 * with Narration Control, allowing real-time toggling of the mask’s jaw
 * animation (driven by TaskNarrate on the mask side) without disrupting Free
 * Play, Game, or Preview flows.
 *
 * Key Narration Features:
 *   - MSG_NARRATE packet (NARR_START / NARR_STOP / NARR_TOGGLE)
 *   - RIGHT stick push (GPIO6):
 *       * Single‑tap  -> Narration toggle
 *       * Long‑press  -> Blink action (eyes)
 *       * Double‑tap  -> Preview toggle (Face/Mode browser)
 *   - LCD hint “R:Talk  Hold:Blink” to surface the new behavior.
 *
 * Other Functions:
 *   - Dual joysticks (rotated): LEFT = control groups; RIGHT = nav/FSM
 *   - Game UI layered over Free Play
 *   - Face Preview (min/mid/max pose) browsing and send-to-mask
 *   - Randomizer, difficulty selection, buzzer feedback
 *
 * ----------------------------------------------------------------------------- */


#include <Arduino.h>                 // Arduino core
#include <Wire.h>                    // I2C for LCD
#include <LiquidCrystal_I2C.h>       // 16x2 I2C LCD driver
#include <WiFi.h>                    // Needed for ESP-NOW
#include <esp_now.h>                 // ESP-NOW transport
#include <esp_crc.h>                 // CRC16 helpers
#include <string.h>
#include <limits.h>

#define N_SERVO 14                   // Total channels in the mask

/* ---- Mask MAC ---- */
// Hard-coded peer MAC (mask unit) for ESP-NOW send
static const uint8_t MASK_MAC[6] = {0xFC,0x01,0x2C,0xDB,0xEB,0x88};

/* ---- Pins (SWAPPED sticks kept) ---- */
// LEFT stick = CTRL (move)
#define CTRL_X_PIN  5                // LEFT stick X (after rotation math)
#define CTRL_Y_PIN  4                // LEFT stick Y
// RIGHT stick = NAV (FSM/menu)
#define NAV_X_PIN  14                // RIGHT stick X (after rotation math)
#define NAV_Y_PIN  13                // RIGHT stick Y
// Buttons (active-LOW)
#define MODE_BTN_PIN   12            // LEFT push: hold=mode toggle, tap=random
#define PREVIEW_BTN_PIN 6            // RIGHT push: double-tap=Preview, single=Talk, long=Blink

#define BUZZER_PIN 10                // PWM channel index for buzzer (logical)
#define BUZZ_FREQ  4000              // Buzzer tone frequency
#define BUZZ_RES   10                // PWM resolution bits

LiquidCrystal_I2C lcd(0x27, 16, 2);  // 16x2 LCD at I2C addr 0x27

/* ---- Protocol ---- */
// Message and command enums match the mask firmware
enum : uint8_t { MSG_MOVE=1, MSG_SYNC_REQ=2, MSG_SYNC_RESP=3, MSG_MODE=4, MSG_RANDOM=5, MSG_GAME=6 };
enum : uint8_t { MODE_TOGGLE=1, MODE_SET_FREE=2, MODE_SET_GAME=3 };
enum : uint8_t {
  GAME_CFG=1, GAME_START=2, GAME_ABORT=3, GAME_STATE=4, GAME_FACE_DONE=5, GAME_RESULT=6,
  GAME_PREVIEW=7
};
enum : uint8_t { PREV_MID=0, PREV_MIN=1, PREV_MAX=2 };

// New narration channel (mirrors mask)
enum : uint8_t { MSG_NARRATE=7 };
enum : uint8_t { NARR_START=1, NARR_STOP=2, NARR_TOGGLE=3 };

// Packet layouts (packed to avoid padding mismatch)
struct __attribute__((packed)) PktNarrate { uint8_t type, cmd; uint16_t crc; };
struct __attribute__((packed)) PktMove    { uint8_t type, idx; int16_t us, xOff; uint16_t crc; };
struct __attribute__((packed)) PktSyncReq { uint8_t type; uint16_t crc; };
struct __attribute__((packed)) PktSyncResp{ uint8_t type; int16_t pos[N_SERVO]; uint16_t crc; };
struct __attribute__((packed)) PktMode    { uint8_t type, cmd, arg; uint16_t crc; };
struct __attribute__((packed)) PktRandom  { uint8_t type; uint16_t crc; };
struct __attribute__((packed)) PktGame    { uint8_t type, cmd, a, b; uint16_t crc; };

// ---------narrate helper-------------------
// Sends a narration control message to the mask
static inline void sendNarrate(uint8_t cmd){
  PktNarrate p{}; p.type=MSG_NARRATE; p.cmd=cmd;
  p.crc = esp_crc16_le(0,(uint8_t*)&p,sizeof(p)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&p,sizeof(p));
}

/* ---- Limits / DIR ---- */
// Per-channel safe µs limits (mirrors mask limits if possible)
struct Limits { int16_t minUs, maxUs; };
static Limits lim[N_SERVO] = {
  { 960,1065},{1041,2248},{2015,2160},{1006,2111},
  {1039,1600},{ 890,1510},{ 981,2300},{1557,2180},
  {1280,1871},{1139,2400},{1216,1750},{ 660,2400},
  { 500,2060},{ 500,2400}
};

/* --------- Neutral helpers for mirroring --------- */
// DIR: direction sign applied to stick motions per channel
static const int8_t DIR[N_SERVO] = {
  +1,+1,-1,-1, +1,-1, +1,+1,+1,-1, +1,+1,-1,+1
};

/* ========================= FSM ========================= */
/* ========================= FSM (Group Puppet States) ========================= */
// Groups multiple channels under a single stick control mode
enum State : uint8_t {
  S_EYEBROW_V=0,   // both vertical brows (CH0 + CH2)
  S_EYEBROW_A,     // both angled brows   (CH1 + CH3)
  S_EYELIDS,       // both eyelids        (CH6 + CH9)
  S_EYES,          // both eyes X/Y       (CH4/5 + CH7/8) -> X mirrored, Y synced
  S_MOUTH,         // both mouth corners  (CH11 + CH12) -> mirrored
  S_COUNT
};

static const char* STATE_NAME[S_COUNT] = {
  "Brows Vert","Brows Angle","Eyelids","Eyes XY","Mouth"
};

/* ---- NAV directions ---- */
// Discrete directions derived from RIGHT stick (with deadband)
enum NavDir : uint8_t { ND_NONE=0, ND_UP, ND_DOWN, ND_LEFT, ND_RIGHT };

/* ---- Prototypes ---- */
static NavDir readNavDirOnce();           // Sample NAV stick and emit single-step direction
static State  fsmNext(State s, NavDir d); // Compute next FSM state from direction

/* ===================== Shared state ==================== */
static volatile bool gSynced=false;       // True after first SYNC_RESP
static volatile bool gGame=false;         // Free on boot; toggled by long-press LEFT push
static int16_t  gPos[N_SERVO];            // Last known positions (from SYNC)
static State    gState = S_EYEBROW_V;     // Current control group

const int   DEAD_CTRL = 600;              // Deadband for LEFT stick
const int   DEAD_NAV  = 600;              // Deadband for RIGHT stick
const float KMAX      = 0.015f;           // Stick gain scaling

/* ==== RX queue & mutex ==== */
// Minimal RX path: queue message then parse on TaskRx
struct RxMsg { uint8_t len; uint8_t bytes[sizeof(PktSyncResp)]; };
static QueueHandle_t rxq;
static SemaphoreHandle_t posMtx;          // Protects gPos

/* ===================== Game/Preview overlay ==================== */
// UI stages overlayed on top of control
enum UiStage : uint8_t { UI_FREEHUD=0, UI_GAME_MENU, UI_GAME_ARM, UI_GAME_RUN, UI_GAME_END, UI_PREVIEW };
static volatile UiStage uiStage = UI_FREEHUD;

enum Diff : uint8_t { DIFF_EASY=0, DIFF_MED=1, DIFF_HARD=2 };
// Faces per round + seconds/face + letter tag
struct DiffCfg { uint8_t faces; uint8_t secPerFace; char tag; };
static const DiffCfg DIFFS[] = {
  {3,60,'E'},  // Easy: 3 faces, 60s/face
  {4,40,'M'},  // Medium: 4 faces, 40s/face
  {5,20,'H'}   // Hard: 5 faces, 20s/face
};
static volatile Diff diffSel = DIFF_EASY;

static volatile uint8_t facesDone=0, facesTotal=0;  // Progress counters
static volatile uint8_t timeRemain=0;               // Remaining seconds for current face
static volatile uint8_t faceIndex=0;                // Current face index (from mask)
static volatile bool    gameWon=false, gameLost=false;
static volatile bool    gameEndLatched=false;       // Prevent duplicate result handling

static uint32_t toastUntilMs = 0;                   // Short-lived banner timer

// PREVIEW state persists across toggles (controller remembers last selection)
static volatile uint8_t previewFace = 0;
static volatile uint8_t previewMode = PREV_MID;
static const uint8_t NUM_FACES_CTRL = 5;            // Controller’s view of face count
static const char* FACE_NAME_CTRL[NUM_FACES_CTRL] = {
  "Poker","Happy","Angry","Sad","Shocked"
};

/* ================== ESP-NOW helpers ==================== */
// Constrain a target value within per-channel limits
static inline int16_t clampUs(uint8_t i, int16_t us){
  return constrain(us, lim[i].minUs, lim[i].maxUs);
}
// Send a MOVE command to the mask (single channel)
static inline void sendMoveTo(uint8_t idx, int16_t us){
  PktMove p{}; p.type=MSG_MOVE; p.idx=idx; p.us=us; p.xOff=0;
  p.crc=esp_crc16_le(0,(uint8_t*)&p,sizeof(p)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&p,sizeof(p));
}
// Ask mask for current positions (fills gPos[] via onRx)
static inline void sendSyncReq(){
  PktSyncReq r{}; r.type=MSG_SYNC_REQ;
  r.crc=esp_crc16_le(0,(uint8_t*)&r,sizeof(r)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&r,sizeof(r));
}
// Toggle mask mode (Free/Game) via long-press LEFT push
static inline void sendModeToggle(){
  PktMode m{}; m.type=MSG_MODE; m.cmd=MODE_TOGGLE; m.arg=0;
  m.crc = esp_crc16_le(0,(uint8_t*)&m,sizeof(m)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&m,sizeof(m));
}
// Trigger mask randomizer (Free Play single-tap LEFT push)
static inline void sendRandom(){
  PktRandom m{}; m.type=MSG_RANDOM;
  m.crc = esp_crc16_le(0,(uint8_t*)&m,sizeof(m)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&m,sizeof(m));
}
// Send a compact GAME packet (CFG/START/STATE/etc.)
static inline void sendGame(uint8_t cmd, uint8_t a=0, uint8_t b=0){
  PktGame g{}; g.type=MSG_GAME; g.cmd=cmd; g.a=a; g.b=b;
  g.crc=esp_crc16_le(0,(uint8_t*)&g,sizeof(g)-2);
  esp_now_send(MASK_MAC,(uint8_t*)&g,sizeof(g));
}

// ESP-NOW receive callback: handles SYNC_RESP and GAME events
static void onRx(const esp_now_recv_info_t*, const uint8_t* d, int len){
  if(len<=0 || len>(int)sizeof(((RxMsg*)0)->bytes)) return;

  if (len==(int)sizeof(PktSyncResp) && d[0]==MSG_SYNC_RESP){
    const PktSyncResp* r=(const PktSyncResp*)d;
    if (esp_crc16_le(0,(uint8_t*)r,sizeof(PktSyncResp)-2)==r->crc){
      if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(5))==pdTRUE){
        for(int i=0;i<N_SERVO;++i) gPos[i]=clampUs(i, r->pos[i]);
        xSemaphoreGive(posMtx);
      }
      gSynced=true;                   // We’re ready to drive after first sync
    }
    return;
  }

  if (len>=(int)sizeof(PktGame) && d[0]==MSG_GAME){
    const PktGame* g=(const PktGame*)d;
    if (esp_crc16_le(0,(uint8_t*)g,sizeof(PktGame)-2)!=g->crc) return;

    if (g->cmd==GAME_STATE){ timeRemain=g->a; faceIndex=g->b; }
    else if (g->cmd==GAME_FACE_DONE){
      facesDone=g->a; facesTotal=g->b;
      ledcWrite(BUZZER_PIN, 512); delay(120); ledcWrite(BUZZER_PIN,0); // chirp confirm
    } else if (g->cmd==GAME_RESULT){
      if (gameEndLatched) return;    // ignore duplicates
      gameEndLatched=true;
      if (g->a==1){                   // WIN
        gameWon=true; gameLost=false;
        for(int i=0;i<3;i++){ ledcWrite(BUZZER_PIN,512); delay(120); ledcWrite(BUZZER_PIN,0); delay(80); }
      } else {                        // LOSE
        gameWon=false; gameLost=true;
        ledcWrite(BUZZER_PIN,512); delay(300); ledcWrite(BUZZER_PIN,0);
      }
      uiStage=UI_GAME_END;
    }
  }
}

/* ===== NAV stick (RIGHT) with 90° CW rotation ===== */
// Converts analog NAV stick into one-shot discrete direction (with deadband)
static NavDir readNavDirOnce(){
  static NavDir prev = ND_NONE;
  int16_t rawX = analogRead(NAV_X_PIN) - 2048;
  int16_t rawY = analogRead(NAV_Y_PIN) - 2048;
  int16_t x =  rawY;   // 90° CW rotation: x'<-y
  int16_t y =  rawX;   // y'<-x (note: sign rules in comment header)
  NavDir dir = ND_NONE;
  int ax = abs(x), ay = abs(y);
  if (ay >= DEAD_NAV || ax >= DEAD_NAV){
    dir = (ay >= ax) ? ((y>0)?ND_UP:ND_DOWN) : ((x>0)?ND_RIGHT:ND_LEFT);
  }
  NavDir emit = ND_NONE;
  if (dir != ND_NONE && prev == ND_NONE) emit = dir;  // emit only on edge
  prev = (dir==ND_NONE)?ND_NONE:dir;
  return emit;
}

/* ==================== FSM transitions (with your custom hops) ================== */
// Left/Right cycle through groups; Up/Down reserved for future
static State fsmNext(State s, NavDir d){
  if (d==ND_NONE) return s;
  switch (d){
    case ND_LEFT:  return (s==0)? (State)(S_COUNT-1) : (State)(s-1);
    case ND_RIGHT: return (State)((s+1)%S_COUNT);
    case ND_UP:    return s; // reserved
    case ND_DOWN:  return s; // reserved
    default: return s;
  }
}

/* ======================== Tasks ======================== */
// TaskRx: pulls messages from rxq and feeds onRx parser
static void TaskRx(void*){
  RxMsg m;
  for(;;){
    if (xQueueReceive(rxq,&m,portMAX_DELAY)!=pdTRUE) continue;
    onRx(nullptr, m.bytes, m.len);
  }
}

// TaskSync: polls for SYNC until gSynced becomes true
static void TaskSync(void*){
  const TickType_t period=pdMS_TO_TICKS(250);
  for(;;){ if(!gSynced) sendSyncReq(); vTaskDelay(period); }
}

const uint32_t BLINK_HOLD_MS = 280;   // press >= this = blink

// --- mirror helpers (place near top, after lim[] / DIR[]) ---
// Midpoint of a channel’s legal range (used for symmetric mouth mirroring)
static inline int16_t midUs(uint8_t ch){
  return (int16_t)((lim[ch].minUs + lim[ch].maxUs)/2);
}
// Generic “mirror around midpoint” helper
static inline int16_t mirrorAround(int16_t mid, int16_t v){
  long dv = (long)v - (long)mid;
  return (int16_t)(mid - dv);
}

// --- quick blink action (tweak BLINK_DELTA if needed) ---
static const int      BLINK_DELTA   = 280;  // how far to push lids (unused in calibrated path)

// Eyelid conventions noted from hardware:
//   R eyelid: open=min, closed=max
//   L eyelid: open=max, closed=min
static const int16_t R_LID_OPEN_RAW   = lim[6].minUs;  // ≈ 981
static const int16_t R_LID_CLOSED_RAW = lim[6].maxUs;  // ≈ 2200
static const int16_t L_LID_OPEN_RAW   = lim[9].maxUs;  // ≈ 2400
static const int16_t L_LID_CLOSED_RAW = lim[9].minUs;  // ≈ 1139

// Soft margins to avoid mechanical binding
static const int16_t R_OPEN_SOFT   = (int16_t)constrain(R_LID_OPEN_RAW   + 20, lim[6].minUs, lim[6].maxUs);
static const int16_t R_CLOSED_SOFT = (int16_t)constrain(R_LID_CLOSED_RAW - 20, lim[6].minUs, lim[6].maxUs);
static const int16_t L_OPEN_SOFT   = (int16_t)constrain(L_LID_OPEN_RAW   - 20, lim[9].minUs, lim[9].maxUs);
static const int16_t L_CLOSED_SOFT = (int16_t)constrain(L_LID_CLOSED_RAW + 20, lim[9].minUs, lim[9].maxUs);

// Quick synchronous blink that restores original eyelid positions after
static void doBlink(){
  // Snapshot current so user puppeteering continues smoothly after blink
  int16_t r0=0, l0=0;
  if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(2))==pdTRUE){
    r0 = gPos[6]; l0 = gPos[9];
    xSemaphoreGive(posMtx);
  }

  const uint32_t tClose = 350;  // close time (ms)
  const uint32_t tHold  = 35;   // hold closed
  const uint32_t tOpen  = 120;  // reopen time

  // Close to calibrated closed targets
  sendMoveTo(6, R_CLOSED_SOFT);
  sendMoveTo(9, L_CLOSED_SOFT);
  delay(tClose);

  delay(tHold);

  // Re-open to where they were (feels more “alive,” works during narration)
  sendMoveTo(6, r0);
  sendMoveTo(9, l0);
  delay(tOpen);
}

// TaskControl: core UI loop (sticks + buttons + overlay logic)
static void TaskControl(void*){
  const TickType_t period = pdMS_TO_TICKS(20);
  TickType_t next = xTaskGetTickCount();

  /* LEFT (MODE/Random) button debounce/timing — unchanged */
  static bool rawMode=false, debMode=false, prevMode=false;
  static uint32_t modeLastChg=0, modePressStart=0;
  static bool modeSentThisHold=false;
  const uint32_t DEBOUNCE_MS=30, HOLD_MS=300;

  /* RIGHT (PREVIEW/NARR/BLINK) button debounce/timing */
  static bool rawPrev=false, debPrev=false, prevPrev=false;
  static uint32_t prevLastChg=0, prevReleaseMs=0, prevPressMs=0;
  const uint32_t DOUBLE_TAP_MS=350;
  const uint32_t BLINK_HOLD_MS=280;   // >= this = blink on long-press

  for(;;){
    /* ===== LEFT button (MODE toggle / Random) ===== */
    bool rMode = (digitalRead(MODE_BTN_PIN) == LOW);
    if (rMode != rawMode){ rawMode=rMode; modeLastChg=millis(); }
    if (millis()-modeLastChg >= DEBOUNCE_MS) debMode = rawMode;

    // Press edge
    if (!prevMode && debMode){ modePressStart=millis(); modeSentThisHold=false; }
    // Long-hold: toggle Free/Game once
    if (debMode && !modeSentThisHold && (millis()-modePressStart)>=HOLD_MS){
      gGame = !gGame; sendModeToggle(); modeSentThisHold = true;
      toastUntilMs = millis() + 2000;
      uiStage = gGame ? UI_GAME_MENU : UI_FREEHUD;
      gameEndLatched=false; gameWon=false; gameLost=false;
    }
    // Release edge: if short tap, do Random or advance Game flow
    if (prevMode && !debMode){
      uint32_t dur = millis()-modePressStart;
      if (dur < HOLD_MS && !modeSentThisHold){
        if (!gGame){
          sendRandom();                    // single-tap in Free Play = Random
        } else {
          // Minimal game flow (arm/start/end cycling)
          if (uiStage==UI_GAME_MENU) uiStage = UI_GAME_ARM;
          else if (uiStage==UI_GAME_ARM){
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

    /* ===== RIGHT button (PREVIEW/double, NARR/single, BLINK/long) ===== */
    bool rPrev = (digitalRead(PREVIEW_BTN_PIN) == LOW);
    if (rPrev != rawPrev){ rawPrev=rPrev; prevLastChg=millis(); }
    if (millis()-prevLastChg >= DEBOUNCE_MS) debPrev = rPrev;

    if (!prevPrev && debPrev){        // press edge
      prevPressMs = millis();
    }

    if (prevPrev && !debPrev){        // release edge
      uint32_t now = millis();
      uint32_t dur = now - prevPressMs;
      bool isLong  = (dur >= BLINK_HOLD_MS);
      bool isDouble= (now - prevReleaseMs) <= DOUBLE_TAP_MS;
      prevReleaseMs = now;

      if (!gGame){ // Only in Free Play contexts (keeps Game UX clean)
        if (isLong){
          // LONG -> Blink
          doBlink();
          ledcWrite(BUZZER_PIN, 320); delay(40); ledcWrite(BUZZER_PIN,0);
          toastUntilMs = now + 600;

        } else if (isDouble){
          // DOUBLE -> Preview toggle
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

        } else {
          // SINGLE -> Narration toggle (NEW)
          sendNarrate(NARR_TOGGLE);
          ledcWrite(BUZZER_PIN, 220); delay(30); ledcWrite(BUZZER_PIN,0);
          toastUntilMs = now + 600;
        }
      }
    }
    prevPrev = debPrev;

    /* ===== Dedicated Blink button on GPIO19 (active-LOW) ===== */
    static bool prevBlinkBtn=false;
    bool blinkBtn = (digitalRead(19) == LOW);
    if (blinkBtn && !prevBlinkBtn){   // press edge
      doBlink();
    }
    prevBlinkBtn = blinkBtn;

    /* ===== NAV stick (RIGHT) → FSM/group selection OR game menus ===== */
    NavDir nav = readNavDirOnce();
    if (!gGame || uiStage==UI_GAME_RUN){
      if (nav != ND_NONE) gState = fsmNext(gState, nav);  // browse groups
    } else if (uiStage==UI_GAME_MENU){
      if (nav==ND_LEFT){
        diffSel = (diffSel==DIFF_EASY)? DIFF_HARD : (Diff)(diffSel-1);
      } else if (nav==ND_RIGHT){
        diffSel = (diffSel==DIFF_HARD)? DIFF_EASY : (Diff)(diffSel+1);
      }
    }

    /* ===== PREVIEW browsing (unchanged) ===== */
    // LEFT stick with 90° CCW rotation
    int16_t rawCX = analogRead(CTRL_X_PIN) - 2048;
    int16_t rawCY = analogRead(CTRL_Y_PIN) - 2048;
    int16_t xOff = -rawCY;   // 90° CCW: x'<- -y
    int16_t yOff =  rawCX;   // y'<-  +x

    if (uiStage == UI_PREVIEW){
      static uint32_t lastMoveMs = 0;
      const uint32_t REPEAT_MS = 210;        // key‑repeat feel
      uint32_t now = millis();
      bool changed = false;

      if (abs(xOff) >= DEAD_CTRL && (now - lastMoveMs) >= REPEAT_MS){
        previewFace = (xOff > 0) ? (uint8_t)((previewFace + 1) % NUM_FACES_CTRL)
                                 : (uint8_t)((previewFace == 0) ? (NUM_FACES_CTRL-1) : (previewFace-1));
        changed = true; lastMoveMs = now;
        ledcWrite(BUZZER_PIN, 180); delay(25); ledcWrite(BUZZER_PIN,0);
      }
      if (abs(yOff) >= DEAD_CTRL && (now - lastMoveMs) >= REPEAT_MS){
        previewMode = (yOff > 0) ? (uint8_t)((previewMode + 1) % 3)
                                 : (uint8_t)((previewMode == 0) ? 2 : (previewMode - 1));
        changed = true; lastMoveMs = now;
        ledcWrite(BUZZER_PIN, 240); delay(25); ledcWrite(BUZZER_PIN,0);
      }
      if (changed) sendGame(GAME_PREVIEW, previewFace, previewMode);
      vTaskDelayUntil(&next, period);
      continue;
    }

    /* ===== GROUP PUPPET CONTROL (CTRL stick) ===== */
    // Drive grouped channels with LEFT stick (scaled by KMAX, with DIR signs)
    if (abs(xOff) >= DEAD_CTRL || abs(yOff) >= DEAD_CTRL){
      float gx = KMAX * sq(abs(xOff)/2048.0f);
      float gy = KMAX * sq(abs(yOff)/2048.0f);

      if (xSemaphoreTake(posMtx,pdMS_TO_TICKS(2))==pdTRUE){

        if (gState == S_EYEBROW_V){
          int16_t n0 = clampUs(0, gPos[0] + int16_t(DIR[0]*gy*yOff));
          int16_t n2 = clampUs(2, gPos[2] + int16_t(DIR[2]*gy*yOff));
          gPos[0]=n0; gPos[2]=n2;
          xSemaphoreGive(posMtx);
          sendMoveTo(0,n0); sendMoveTo(2,n2);

        } else if (gState == S_EYEBROW_A){
          int16_t n1 = clampUs(1, gPos[1] + int16_t(DIR[1]*gy*yOff));
          int16_t n3 = clampUs(3, gPos[3] + int16_t(DIR[3]*gy*yOff));
          gPos[1]=n1; gPos[3]=n3;
          xSemaphoreGive(posMtx);
          sendMoveTo(1,n1); sendMoveTo(3,n3);

        } else if (gState == S_EYELIDS){
          int16_t n6 = clampUs(6, gPos[6] + int16_t(DIR[6]*gy*yOff));
          int16_t n9 = clampUs(9, gPos[9] + int16_t(DIR[9]*gy*yOff));
          gPos[6]=n6; gPos[9]=n9;
          xSemaphoreGive(posMtx);
          sendMoveTo(6,n6); sendMoveTo(9,n9);

        } else if (gState == S_EYES){
           // X: use per-channel DIR so you can flip either eye with DIR[4]/DIR[7]
  int16_t nRx = clampUs(4, gPos[4] + int16_t(DIR[4]*gx*xOff));
  int16_t nLx = clampUs(7, gPos[7] + int16_t(DIR[7]*gx*xOff));

  // Y: move both the same (sync)
  int16_t nRy = clampUs(5, gPos[5] + int16_t(DIR[5]*gy*yOff));
  int16_t nLy = clampUs(8, gPos[8] + int16_t(DIR[8]*gy*yOff));

  gPos[4]=nRx; gPos[7]=nLx; gPos[5]=nRy; gPos[8]=nLy;
  xSemaphoreGive(posMtx);
  sendMoveTo(4,nRx); sendMoveTo(7,nLx);
  sendMoveTo(5,nRy); sendMoveTo(8,nLy);

        } else if (gState == S_MOUTH){
          // Mirror left mouth around midpoint to match right mouth offset
          int16_t midR = midUs(11), midL = midUs(12);
          int16_t nR   = clampUs(11, gPos[11] + int16_t(DIR[11]*gy*yOff));
          int16_t offR = nR - midR;
          int16_t nL   = clampUs(12, midL - offR);
          gPos[11]=nR; gPos[12]=nL;
          xSemaphoreGive(posMtx);
          sendMoveTo(11,nR); sendMoveTo(12,nL);

        } else {
          xSemaphoreGive(posMtx);
        }
      }
    }

    vTaskDelayUntil(&next, period);
  }
}

/* ===== LCD helpers / TaskLCD ===== */
// Print fixed-length row (pads/truncates to 16)
static void lcdWriteRow(uint8_t row, const char* text){
  char buf[17]; memset(buf,' ',16); buf[16]='\0';
  size_t n = strlen(text); if (n>16) n=16;
  memcpy(buf, text, n);
  lcd.setCursor(0,row); lcd.print(buf);
}
// Print left/right-aligned pair on a single row
static void lcdWriteRowLR(uint8_t row, const char* left, const char* right){
  char buf[17]; memset(buf,' ',16); buf[16]='\0';
  size_t ll = strlen(left); if (ll>16) ll=16;
  memcpy(buf, left, ll);
  size_t rl = strlen(right); if (rl>16) rl=16;
  memcpy(buf + (16-rl), right, rl);
  lcd.setCursor(0,row); lcd.print(buf);
}

// TaskLCD: renders HUD, Preview, and Game screens with small “toast” overlay
static void TaskLCD(void*){
  State   prevState = (State)255;
  bool    prevGame  = false;
  Diff    prevDiff  = DIFF_EASY;
  UiStage prevStage = UI_FREEHUD;

  for(;;){
    // --- Toast overlay (mode/preview notifications) ---
    if (toastUntilMs && millis() < toastUntilMs){
      lcdWriteRow(0, (uiStage==UI_PREVIEW) ? "Preview Mode" : (gGame ? "Game Mode" : "Free Play"));
      lcdWriteRow(1, "                ");
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    } else if (toastUntilMs && millis() >= toastUntilMs){
      toastUntilMs = 0;
    }

    // --- Sync gate: wait for positions before showing normal UI ---
    if (!gSynced){
      lcdWriteRow(0, "Syncing...");
      lcdWriteRow(1, "                ");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // ===================== PREVIEW =====================
    if (uiStage == UI_PREVIEW){
      const char* modeName = (previewMode==PREV_MID) ? "MID"
                             : (previewMode==PREV_MIN) ? "MIN" : "MAX";
      char top[17];  snprintf(top, sizeof(top), "Preview %u/%u",
                              (unsigned)(previewFace+1), (unsigned)NUM_FACES_CTRL);
      lcdWriteRow(0, top);

      const char* fname = FACE_NAME_CTRL[previewFace];
      char shortName[10]; strncpy(shortName, fname, 9); shortName[9]='\0';
      char bot[17]; snprintf(bot, sizeof(bot), "%-9s %s", shortName, modeName);
      lcdWriteRow(1, bot);

      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // ===================== FREE PLAY HUD =====================
    if (!gGame || uiStage==UI_FREEHUD){
      if (gState != prevState || gGame != prevGame || uiStage != prevStage){
        lcdWriteRow(0, STATE_NAME[gState]);               // group name
        prevState = gState; prevGame = gGame; prevStage = uiStage;
      }
      lcdWriteRow(1, "R:Talk  Hold:Blink");              // narration/blink hint
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // ===================== GAME: MENU =====================
    if (uiStage==UI_GAME_MENU){
      const char* name = (diffSel==DIFF_EASY) ? "Easy"
                         : (diffSel==DIFF_MED) ? "Medium" : "Hard";
      lcdWriteRow(0, "Select Difficulty");
      char row1[17]; snprintf(row1, sizeof(row1), "< %s >", name);
      lcdWriteRow(1, row1);
      prevStage=uiStage; prevDiff=diffSel;
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // ===================== GAME: ARM =====================
    if (uiStage==UI_GAME_ARM){
      lcdWriteRow(0, "Press btn to start");
      const DiffCfg& d = DIFFS[diffSel];
      char r1[17]; snprintf(r1, sizeof(r1), "%u faces %us",
                            (unsigned)d.faces, (unsigned)d.secPerFace);
      lcdWriteRow(1, r1);
      prevStage=uiStage; prevDiff=diffSel;
      vTaskDelay(pdMS_TO_TICKS(120));
      continue;
    }

    // ===================== GAME: RUN =====================
    if (uiStage==UI_GAME_RUN){
      // Top row: current group left, time right
      char tbuf[4]; snprintf(tbuf, sizeof(tbuf), "%02u", (unsigned)timeRemain);
      lcdWriteRowLR(0, STATE_NAME[gState], tbuf);

      // Bottom row: flip between current face name and progress
      static uint32_t flipT=0; static bool flip=false;
      if (millis()-flipT>700){ flip=!flip; flipT=millis(); }
      if (flip){
        static const char* FACE_NAME[] = {"Poker","Happy","Angry","Sad","Shocked"};
        char b[17]; snprintf(b, sizeof(b), "Face %s",
                             FACE_NAME[faceIndex % (sizeof(FACE_NAME)/sizeof(FACE_NAME[0]))]);
        lcdWriteRow(1, b);
      } else {
        char pb[17]; snprintf(pb, sizeof(pb), "Done %u/%u",
                              (unsigned)facesDone, (unsigned)facesTotal);
        lcdWriteRow(1, pb);
      }
      vTaskDelay(pdMS_TO_TICKS(80));
      continue;
    }

    // ===================== GAME: END =====================
    if (uiStage==UI_GAME_END){
      lcdWriteRow(0, gameWon ? "GAME WON!" : "GAME LOST!");
      lcdWriteRow(1, "Press btn restart");
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }
  }
}

/* ===== setup / loop ===== */
// Hardware bring-up, ESP‑NOW peer add, tasks spawn
void setup(){
  Serial.begin(115200);
  analogReadResolution(12);                 // 12-bit ADC (0..4095)

  pinMode(MODE_BTN_PIN,    INPUT_PULLUP);   // active-LOW buttons
  pinMode(PREVIEW_BTN_PIN, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);                // dedicated blink button

  Wire.begin(20, 21);                       // LCD I2C pins (SDA=20, SCL=21)
  lcd.init(); lcd.backlight();
  lcdWriteRow(0, "Syncing...");
  lcdWriteRow(1, "                ");

  ledcAttach(BUZZER_PIN, BUZZ_FREQ, BUZZ_RES);
  ledcWrite(BUZZER_PIN, 0);                 // buzzer off

  WiFi.mode(WIFI_STA);
  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init fail"); for(;;); }
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr,MASK_MAC,6);
  peer.ifidx=WIFI_IF_STA; peer.channel=0; peer.encrypt=false;
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onRx);

  rxq   = xQueueCreate(8,sizeof(RxMsg));    // RX queue
  posMtx= xSemaphoreCreateMutex();          // position mutex

  for(int i=0;i<N_SERVO;++i) gPos[i] = constrain(1500, lim[i].minUs, lim[i].maxUs);
  gState = S_EYEBROW_V;                     // start on first group
  gGame  = false;                           // Free on boot
  uiStage= UI_FREEHUD;

  sendSyncReq();                            // kick off sync early

  // Task pinning mirrors the mask’s layout: higher prio for RX/control
  xTaskCreatePinnedToCore(TaskRx,      "rx",      4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskSync,    "sync",    2048, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskControl, "control", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskLCD,     "lcd",     3072, nullptr, 1, nullptr, 1);

  Serial.println("Controller ready: LEFT stick moves (rotated CCW); RIGHT stick navigates (rotated CW).");
  Serial.println("Buttons: LEFT push=Hold:Mode / Tap:Random, RIGHT push=Double-tap:Preview.");
}
void loop(){}  // All logic runs in FreeRTOS tasks
