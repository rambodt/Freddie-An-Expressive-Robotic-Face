/*
 * File:        mask.ino
 *
 * Author:Rambod Taherian, Kourosh Ghahramani
 * Date:        2025-08-22
 *
 * Description:
 *   Mask-side firmware driving 13 servos via PCA9685. Supports:
 *     • Free Play: absolute servo control via MOVE packets with CRC16 validation.
 *     • Game Mode: sequence matching of predefined faces with timer and results.
 *     • Preview: GAME_PREVIEW(faceIdx, mode[0=MID,1=MIN,2=MAX]) smoothly slews
 *       targets to a face’s MIN/MID/MAX ranges (eyes/eyelids/jaw/mouth, etc.).
 *
 *   Concurrency:
 *     • TaskRx    — parses ESP-NOW packets (controller → mask) and updates state.
 *     • TaskSlew  — every 20 ms, moves posUs toward targetUs (clamped to limits).
 *     • TaskGame  — checks face match, pushes GAME_STATE/RESULT/FACE_DONE.
 *     • TaskRandom— optional simultaneous randomizer across channels (Free only).
 *
 *   Non-obvious details:
 *     • CRC16-LE covers each packet except the trailing crc field.
 *     • us2cnt() converts microseconds to PCA9685 counts at 50 Hz.
 *     • NVS (Preferences) persists last positions; restore on boot if available.
 *     • previewActive applies goals immediately, but motion is handled by TaskSlew.
 *     • gameActive gates Preview and Random so they don't fight a running game.
 *
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_crc.h>
#include <limits.h>

#define N_SERVO        14

/* ================================ I2C / PCA9685 ================================= */
#define SDA_PIN        5
#define SCL_PIN        4
#define SERVO_FREQ     50
#define MIN_US_GLOBAL  500
#define MAX_US_GLOBAL  2400
#define MAX_STEP_US    20   // max microsecond change per 20 ms tick (TaskSlew)

/* Optional Output Enable pin for PCA9685 (active-LOW when driving) */
#define USE_OE         1
#if USE_OE
#define OE_PIN        18
#endif

/* ================================ Peering / MACs ================================= */
/* Controller MAC */
static const uint8_t CONTROLLER_MAC[6] = {0xFC,0x01,0x2C,0xDB,0xF4,0x00};
static uint8_t ctlMac[6] = {0};
static bool    ctlMacKnown = false;

/* ================================ Protocol ======================================= */
enum : uint8_t { MSG_MOVE=1, MSG_SYNC_REQ=2, MSG_SYNC_RESP=3, MSG_MODE=4, MSG_RANDOM=5, MSG_GAME=6 };
enum : uint8_t { MODE_TOGGLE=1, MODE_SET_FREE=2, MODE_SET_GAME=3 };
enum : uint8_t {
  GAME_CFG=1, GAME_START=2, GAME_ABORT=3, GAME_STATE=4, GAME_FACE_DONE=5, GAME_RESULT=6,
  GAME_PREVIEW=7
};
enum : uint8_t { PREV_MID=0, PREV_MIN=1, PREV_MAX=2 };

/* ================================ Limits / Names ================================= */
/* Per-servo limits (absolute mechanical safe bounds) */
struct Limits { int16_t minUs, maxUs; };
static Limits lim[N_SERVO] = {
  { 960,1065} /*0  L brow V*/,
  {1041,2248} /*1  L brow A*/,
  {2015,2160} /*2  R brow V*/,
  {1006,2111} /*3  R brow A*/,
  {1039,1600} /*4  R eye X*/,
  { 890,1510} /*5  R eye Y*/,
  { 981,2200} /*6  R eyelid*/,
  {1557,2180} /*7  L eye X*/,
  {1280,1871} /*8  L eye Y*/,
  {1139,2400} /*9  L eyelid*/,
  {1216,1750} /*10 Jaw*/,
  { 660,2400} /*11 R mouth*/,
  { 500,2060} /*12 L mouth*/,
  { 500,2400} /*13 Extra*/
};

/* Optional names for debug prints */
static const char* CH_NAME[N_SERVO] = {
  "L brow V","L brow A","R brow V","R brow A",
  "R eye X","R eye Y","R eyelid","L eye X","L eye Y","L eyelid",
  "Jaw","R mouth","L mouth","Extra"
};

/* ================================ Faces / Ranges ================================= */
/* Range [min,max] in microseconds; IGN marks channel ignored for a face. */
struct Range { int16_t minUs, maxUs; };
struct Face  { const char* name; Range ch[N_SERVO]; };
#define IGN {INT16_MAX, INT16_MIN}
#define U(us,tol) { int16_t((us)-(tol)), int16_t((us)+(tol)) }

/* Face definitions — kept within your channel limits; tune U() tolerances as needed. */
static Face faces[] = {

/* 0 */
{ "Poker", {
  U(1045,55)  /*0 L brow V*/,
  U(1420,100) /*1 L brow A*/, 
  U(2045,35)  /*2 R brow V*/, 
  U(1510,80) /*3 R brow A*/,
  U(1290,230)  /*4 R eye X*/, 
  U(1210,170) /*5 R eye Y*/, 
  U(1560,300) /*6 R eyelid*/, 
  U(1880,230) /*7 L eye X*/,
  U(1620,170) /*8 L eye Y*/, 
  U(1670,300) /*9 L eyelid*/, 
  U(1260,60)  /*10 Jaw*/, 
  U(1310,150) /*11 R mouth*/,
  U(1290,150) /*12 L mouth*/, 
  IGN         /*13 Extra*/
}},

/* 1 */
{ "Happy", {
  U(1045,55)  /*0 L brow V*/, 
  U(1600,200) /*1 L brow A*/, 
  U(2045,35)  /*2 R brow V*/, 
  U(1345,200) /*3 R brow A*/,
  IGN /*4 R eye X*/, 
  IGN /*5 R eye Y*/, 
  U(1560,300) /*6 R eyelid*/,
  IGN /*7 L eye X*/,
  IGN /*8 L eye Y*/, 
  U(1670,300) /*9 L eyelid*/, 
  IGN /*10 Jaw*/, 
  U( 922,330) /*11 R mouth*/,
  U(1730,330) /*12 L mouth*/, 
  IGN         /*13 Extra*/
}},

/* 2 */
{ "Angry", {
  U(1045,55)  /*0 L brow V*/, 
  U(1158,117) /*1 L brow A*/, 
  U(2045,35)  /*2 R brow V*/, 
  U(1871,240) /*3 R brow A*/,
  IGN /*4 R eye X*/, 
  IGN /*5 R eye Y*/, 
  U(1560,300) /*6 R eyelid*/, 
  IGN /*7 L eye X*/,
  IGN /*8 L eye Y*/, 
  U(1670,300) /*9 L eyelid*/, 
  IGN /*10 Jaw*/, 
  U( 1890,510) /*11 R mouth*/,
  U(855,355) /*12 L mouth*/, 
  IGN         /*13 Extra*/
}},

/* 3 */
{ "Sad", {
  U(1045,55)  /*0 L brow V*/, 
  U(1600,200) /*1 L brow A*/, 
  U(2045,35)  /*2 R brow V*/, 
  U(1345,200) /*3 R brow A*/,
  IGN /*4 R eye X*/, 
  IGN /*5 R eye Y*/, 
  U(1560,300) /*6 R eyelid*/, 
  IGN /*7 L eye X*/,
  IGN /*8 L eye Y*/, 
  U(1670,300) /*9 L eyelid*/, 
  U(1260,60) /*10 Jaw*/, 
  U( 1890,510) /*11 R mouth*/,
  U(855,355) /*12 L mouth*/, 
  IGN         /*13 Extra*/
}},

/* 5 */
{ "Shocked", {
  U(983,25)  /*0 L brow V*/, 
  U(1420,80) /*1 L brow A*/, 
  U(2115,45)  /*2 R brow V*/, 
  U(1510,80) /*3 R brow A*/,
  U(1290,230)  /*4 R eye X*/, 
  U(1210,170) /*5 R eye Y*/,
  U( 1192,211) /*6 R eyelid*/, 
  U(1880,230) /*7 L eye X*/,
  U(1620,170) /*8 L eye Y*/,  
  U(2127,273) /*9 L eyelid*/, 
  U(1700,50)  /*10 Jaw*/, 
  U(1310,250) /*11 R mouth*/,
  U(995,300) /*12 L mouth*/, 
  IGN         /*13 Extra*/
}},
};

static const int NUM_FACES = sizeof(faces)/sizeof(faces[0]);

/* ================================ Globals ====================================== */
Adafruit_PWMServoDriver pwm(0x40);
Preferences prefs;

struct Msg { uint8_t len; uint8_t src[6]; uint8_t bytes[64]; };
static QueueHandle_t q;

static volatile int16_t targetUs[N_SERVO];   // where we want to go
static int16_t posUs[N_SERVO];               // what we have commanded (tracked)
static int16_t lastPrintedUs[N_SERVO];       // last printed pos (debug throttle)

static volatile bool saveDirty=false;
static uint32_t lastSaveMs=0;

/* Game state */
static bool     inGameMode = false;
static bool     gameActive = false;
static bool     gameResultSent = false;

static uint8_t  orderIdx = 0;
static uint8_t  order[32];
static uint32_t faceStartMs = 0;
static uint16_t stableCount = 0;

// per round:
static uint8_t  gameFaces = 3;       // number of faces this round
static uint32_t faceMs    = 30000;   // ms allowed per face

#define STABLE_MATCH_FRAMES   5
#define GAME_CHECK_PERIOD_MS  20

/* RANDOM trigger from controller */
static volatile bool randomTrigger = false;

/* Preview state (controller-driven) */
static volatile bool previewActive = false;
static volatile uint8_t previewFace = 0;
static volatile uint8_t previewMode = PREV_MID;

/* ================================ Helpers ====================================== */
/** us2cnt
 *  Convert microseconds to PCA9685 12-bit count at SERVO_FREQ.
 */
static inline uint16_t us2cnt(int us){
  return (uint16_t)(us / (1e6f / (SERVO_FREQ * 4096.0f)));
}

/** clampServoUs
 *  Clamp desired microseconds to both per-channel limits and global guard rails.
 */
static inline int16_t clampServoUs(uint8_t i, int16_t us){
  int16_t a = constrain(us, lim[i].minUs, lim[i].maxUs);
  return constrain(a, MIN_US_GLOBAL, MAX_US_GLOBAL);
}

/** ensurePeer
 *  Add peer if missing (idempotent). Required before esp_now_send().
 */
static void ensurePeer(const uint8_t mac[6]){
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, mac, 6);
    peer.ifidx=WIFI_IF_STA; peer.channel=0; peer.encrypt=false;
    esp_now_add_peer(&peer);
  }
}

/** savePositionsIfNeeded
 *  Debounced NVS write of current posUs[] to limit flash wear.
 */
static void savePositionsIfNeeded(){
  uint32_t now=millis();
  if (saveDirty && (now-lastSaveMs)>800){
    prefs.putBytes("pos", posUs, sizeof(posUs));
    lastSaveMs=now; saveDirty=false;
  }
}

/** randIn
 *  Inclusive random in [mn,mx] using esp_random().
 */
static inline int16_t randIn(int16_t mn, int16_t mx){
  if (mx < mn) return mn;
  uint32_t r = esp_random();
  return (int16_t)(mn + (r % (uint32_t)(mx - mn + 1)));
}

/** shuffleOrder
 *  Prepare randomized per-round face order (values are face indices).
 */
static void shuffleOrder(uint8_t* arr,int n){
  for(int i=0;i<n;i++) arr[i]=i % NUM_FACES;
  for(int i=n-1;i>0;--i){ uint32_t r=esp_random(); int j=r%(i+1); uint8_t t=arr[i]; arr[i]=arr[j]; arr[j]=t; }
}

/** faceMatched
 *  Returns true when all non-IGN channels fall within the face’s [min,max] windows.
 */
static bool faceMatched(const Face& f){
  for(uint8_t i=0;i<N_SERVO;++i){
    int16_t mn=f.ch[i].minUs, mx=f.ch[i].maxUs;
    if (mn<=mx){
      int16_t v=posUs[i];
      if (v<mn || v>mx) return false;
    }
  }
  return true;
}

/* ================================ GAME send helper ============================== */
/** sendGame
 *  Send a GAME packet back to controller (requires known ctlMac).
 */
static void sendGame(uint8_t cmd, uint8_t a=0, uint8_t b=0){
  if (!ctlMacKnown) return;
  ensurePeer(ctlMac);
  struct __attribute__((packed)) PktGame { uint8_t type, cmd, a, b; uint16_t crc; };
  PktGame g{}; g.type=MSG_GAME; g.cmd=cmd; g.a=a; g.b=b;
  g.crc=esp_crc16_le(0,(uint8_t*)&g,sizeof(g)-2);
  esp_now_send(ctlMac,(uint8_t*)&g,sizeof(g));
}

/* ================================ GAME control ================================= */
/** gameStart
 *  Initialize a new round using current gameFaces and faceMs.
 *  Push initial GAME_STATE with seconds and first face index.
 */
static void gameStart(){
  uint8_t n = gameFaces;
  if (n > NUM_FACES) n = NUM_FACES;
  if (n < 1) n = 1;

  shuffleOrder(order, n);
  orderIdx=0; faceStartMs=millis(); stableCount=0;
  gameActive=true; gameResultSent=false;

  uint32_t s0 = faceMs / 1000U;
  if (s0 > 255) s0 = 255;
  sendGame(GAME_STATE, (uint8_t)s0, order[orderIdx]);
}

/** gameStop
 *  End the round, send one-shot GAME_RESULT (win=1/lose=0) if not already sent.
 */
static void gameStop(bool won){
  if (gameActive) gameActive=false;
  if (!gameResultSent){
    sendGame(GAME_RESULT, won?1:0, 0);
    gameResultSent=true;
  }
}

/* ================================ Preview apply ================================= */
/** applyPreview
 *  Compute and set targetUs[] for a given face and preview mode:
 *    • PREV_MIN → per-channel minUs, PREV_MAX → maxUs, PREV_MID → midpoint.
 *  Motion is performed by TaskSlew at a fixed MAX_STEP_US rate.
 */
static void applyPreview(uint8_t idx, uint8_t mode){
  if (idx >= NUM_FACES) idx = NUM_FACES - 1;
  const Face& F = faces[idx];

  for (uint8_t i=0;i<N_SERVO;++i){
    int16_t mn = F.ch[i].minUs;
    int16_t mx = F.ch[i].maxUs;

    if (mn > mx){
      // IGN: leave target unchanged
      continue;
    }
    int32_t mid = (int32_t)mn + (int32_t)mx;
    mid /= 2;

    int16_t goal = (mode==PREV_MIN) ? mn : (mode==PREV_MAX) ? mx : (int16_t)mid;
    goal = clampServoUs(i, goal);
    targetUs[i] = goal;  // Slew loop will glide
  }
}

/* ================================ ESP-NOW RX (ISR → queue) ===================== */
/** onRx
 *  Lightweight ISR-safe handler: copies frame into queue for TaskRx to parse.
 */
static void onRx(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  if (!info || len<=0 || len>(int)sizeof(((Msg*)0)->bytes)) return;
  Msg m; m.len=(uint8_t)len; memcpy(m.src, info->src_addr, 6); memcpy(m.bytes, data, len);
  BaseType_t hpw=pdFALSE; xQueueSendFromISR(q,&m,&hpw); if(hpw) portYIELD_FROM_ISR();
}

/* ================================ Tasks ========================================= */
/** TaskRx
 *  Parses queued ESP-NOW packets and updates targets/mode/game/preview:
 *   - MOVE: updates targetUs[idx] (absolute microseconds).
 *   - SYNC_REQ: replies with SYNC_RESP(posUs[]).
 *   - MODE: toggles/sets inGameMode; stops game if leaving Game.
 *   - GAME: CFG/START/ABORT/PREVIEW (PREVIEW ignored while gameActive).
 *   - RANDOM: triggers simultaneous randomizer (Free only).
 */
static void TaskRx(void*){
  Msg m;
  for(;;){
    if (xQueueReceive(q,&m,portMAX_DELAY)!=pdTRUE) continue;
    memcpy(ctlMac, m.src, 6);
    ctlMacKnown = true;
    ensurePeer(ctlMac);

    if (m.len<1) continue;
    uint8_t type=m.bytes[0];

    if (type==MSG_MOVE){
      if (m.len < (int)sizeof(uint8_t)+ (int)sizeof(uint8_t)+ (int)sizeof(int16_t)*2 + (int)sizeof(uint16_t)) continue;
      struct __attribute__((packed)) PktMove { uint8_t type, idx; int16_t us, xOff; uint16_t crc; };
      const PktMove* p=(const PktMove*)m.bytes;
      bool ok = (p->idx < N_SERVO) &&
                (esp_crc16_le(0,(uint8_t*)p,sizeof(PktMove)-2)==p->crc);
      if (ok && p->us!=-1){
        targetUs[p->idx]=clampServoUs(p->idx, p->us);
      }

    } else if (type==MSG_SYNC_REQ){
      if (m.len != (int)sizeof(uint8_t)+ (int)sizeof(uint16_t)) continue;
      struct __attribute__((packed)) PktSyncReq { uint8_t type; uint16_t crc; };
      const PktSyncReq* rq=(const PktSyncReq*)m.bytes;
      bool ok=(esp_crc16_le(0,(uint8_t*)rq,sizeof(PktSyncReq)-2)==rq->crc);
      if (ok){
        ensurePeer(m.src);
        struct __attribute__((packed)) PktSyncResp{ uint8_t type; int16_t pos[N_SERVO]; uint16_t crc; };
        PktSyncResp r{}; r.type=MSG_SYNC_RESP;
        for(int i=0;i<N_SERVO;++i) r.pos[i]=posUs[i];
        r.crc=esp_crc16_le(0,(uint8_t*)&r,sizeof(r)-2);
        esp_now_send(m.src,(uint8_t*)&r,sizeof(r));
      }

    } else if (type==MSG_MODE){
      if (m.len != (int)sizeof(uint8_t)*3 + (int)sizeof(uint16_t)) continue;
      struct __attribute__((packed)) PktMode { uint8_t type, cmd, arg; uint16_t crc; };
      const PktMode* pm = (const PktMode*)m.bytes;
      bool ok = (esp_crc16_le(0,(uint8_t*)pm,sizeof(PktMode)-2)==pm->crc);
      if (ok){
        if (pm->cmd==MODE_SET_FREE){
          inGameMode=false;
          gameStop(false);
        } else if (pm->cmd==MODE_SET_GAME){
          inGameMode=true;
          gameStop(false);
        } else if (pm->cmd==MODE_TOGGLE){
          inGameMode = !inGameMode;
          if (!inGameMode) gameStop(false);
        }
      }

    } else if (type==MSG_GAME){
      if (m.len < (int)sizeof(uint8_t)*4 + (int)sizeof(uint16_t)) continue;
      struct __attribute__((packed)) PktGame { uint8_t type, cmd, a, b; uint16_t crc; };
      const PktGame* g = (const PktGame*)m.bytes;
      bool ok = (esp_crc16_le(0,(uint8_t*)g,sizeof(PktGame)-2)==g->crc);
      if (!ok) continue;

      if (g->cmd==GAME_CFG){
        uint8_t wantFaces = g->a;
        uint8_t wantSec   = g->b;
        if (wantFaces < 1) wantFaces = 1;
        if (wantFaces > NUM_FACES) wantFaces = NUM_FACES;
        if (wantSec   < 5) wantSec   = 5;
        if (wantSec   > 240) wantSec = 240;
        gameFaces = wantFaces;
        faceMs    = (uint32_t)wantSec * 1000U;

      } else if (g->cmd==GAME_START){
        if (inGameMode) gameStart();

      } else if (g->cmd==GAME_ABORT){
        gameStop(false);

      } else if (g->cmd==GAME_PREVIEW){
        if (!gameActive){ // do not fight the running game
          previewActive = true;
          previewFace = g->a;
          previewMode = g->b;  // 0=MID,1=MIN,2=MAX
          applyPreview(previewFace, previewMode);
          Serial.printf("Preview: face=%u mode=%u\n", (unsigned)previewFace, (unsigned)previewMode);
        }
      }

    } else if (type==MSG_RANDOM){
      if (m.len != (int)sizeof(uint8_t) + (int)sizeof(uint16_t)) continue;
      struct __attribute__((packed)) PktRandom { uint8_t type; uint16_t crc; };
      const PktRandom* pr = (const PktRandom*)m.bytes;
      bool ok = (esp_crc16_le(0,(uint8_t*)pr,sizeof(PktRandom)-2)==pr->crc);
      if (ok){
        randomTrigger = true;
      }
    }
  }
}

/** TaskSlew
 *  50 Hz position controller: move posUs toward targetUs with MAX_STEP_US limit,
 *  clamp within channel/global bounds, write to PCA9685, and mark saveDirty for NVS.
 */
static void TaskSlew(void*){
  const TickType_t period=pdMS_TO_TICKS(20);
  TickType_t next=xTaskGetTickCount();
  for(;;){
    bool changed=false;
    for(uint8_t i=0;i<N_SERVO;++i){
      int16_t t=targetUs[i], p=posUs[i], d=t-p;
      if (d){
        int16_t step=d;
        if(step> MAX_STEP_US) step= MAX_STEP_US;
        if(step<-MAX_STEP_US) step=-MAX_STEP_US;
        p=clampServoUs(i, (int16_t)(p+step));
        posUs[i]=p;
        pwm.setPWM(i,0,us2cnt(p));
        changed=true;
        if (p!=lastPrintedUs[i]){
          // Uncomment for debug spam:
          // Serial.printf("CH%u  pos=%4d us  target=%4d us  diff=%+4d us\n", i, p, t, (int)d);
          lastPrintedUs[i]=p;
        }
      }
    }
    if (changed) saveDirty=true;
    savePositionsIfNeeded();
    vTaskDelayUntil(&next, period);
  }
}

/** TaskGame
 *  If a game is active, checks for face match stability and timeouts:
 *    - On stable match (STABLE_MATCH_FRAMES), send GAME_FACE_DONE and move on.
 *    - On last face matched → gameStop(true).
 *    - On timeout (elapsed ≥ faceMs) → gameStop(false).
 *    - Push GAME_STATE once per second with remaining seconds and current face idx.
 */
static void TaskGame(void*){
  Serial.printf("\n>>> GAME task ready: %d faces\n", NUM_FACES);
  const TickType_t period=pdMS_TO_TICKS(GAME_CHECK_PERIOD_MS);
  TickType_t next=xTaskGetTickCount();
  uint32_t lastStatePush=0;

  for(;;){
    if(!gameActive){ vTaskDelayUntil(&next,period); continue; }
    uint32_t now=millis();

    const Face& F=faces[order[orderIdx]];
    uint32_t elapsed=now-faceStartMs;

    if (faceMatched(F)){
      if(++stableCount>=STABLE_MATCH_FRAMES){
        stableCount=0;
        orderIdx++;
        sendGame(GAME_FACE_DONE, orderIdx, gameFaces);
        if (orderIdx >= gameFaces){
          gameStop(true); // WIN
        } else {
          faceStartMs=now;
          uint8_t sec = (uint8_t)min<uint32_t>(255, faceMs/1000U);
          sendGame(GAME_STATE, sec, order[orderIdx]);
        }
      }
    } else {
      stableCount=0;
    }

    if (elapsed >= faceMs && gameActive){
      gameStop(false); // LOSE (timeout)
    }

    // 1 Hz STATE push
    if (gameActive && (now - lastStatePush) >= 1000) {
      uint32_t remain = (elapsed < faceMs) ? (faceMs - elapsed) : 0;
      uint32_t rs = remain / 1000U;
      if (rs > 255) rs = 255;
      sendGame(GAME_STATE, (uint8_t)rs, order[orderIdx]);
      lastStatePush = now;
    }

    vTaskDelayUntil(&next, period);
  }
}

/* ---------- RANDOMIZER (SIMULTANEOUS) ---------- */
/** TaskRandom
 *  Optional simultaneous random target assignment:
 *   - Skips if gameActive (guarded by RAND_REQUIRE_FREE).
 *   - Waits for settle (|pos - target| ≤ RAND_SETTLE_US) or timeout.
 */
#define RAND_SETTLE_US        10
#define RAND_TIMEOUT_MS     3000
#define RAND_REQUIRE_FREE      1
#define RAND_WAIT_FOR_SETTLE   1

static void TaskRandom(void*)
{
  for(;;){
    if (!randomTrigger){ vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    randomTrigger = false;

    if (RAND_REQUIRE_FREE && gameActive){
      Serial.println(">>> RANDOMIZER ignored (Game running).");
      continue;
    }

    Serial.println(">>> RANDOMIZER (SIMULTANEOUS): start");

    int16_t rndTarget[N_SERVO];
    for (uint8_t i=0;i<N_SERVO;++i){
      rndTarget[i] = clampServoUs(i, randIn(lim[i].minUs, lim[i].maxUs));
    }
    for (uint8_t i=0;i<N_SERVO;++i){
      targetUs[i] = rndTarget[i];
    }

#if RAND_WAIT_FOR_SETTLE
    uint32_t t0 = millis();
    for(;;){
      if (RAND_REQUIRE_FREE && gameActive){
        Serial.println(">>> RANDOMIZER aborted (Game turned ON).");
        break;
      }
      bool allSet = true;
      for (uint8_t i=0;i<N_SERVO;++i){
        int16_t p = posUs[i];
        if (abs(p - rndTarget[i]) > RAND_SETTLE_US){ allSet=false; break; }
      }
      if (allSet) break;
      if ((millis() - t0) > RAND_TIMEOUT_MS){
        Serial.println(">>> RANDOMIZER: timeout before all settled.");
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
#endif
    Serial.println(">>> RANDOMIZER (SIMULTANEOUS): done");
  }
}

/* ================================ Setup / Loop ================================= */
/** setup
 *  Initializes I2C/PCA9685, restores NVS positions (if present), configures ESP-NOW,
 *  creates tasks, and enables outputs (OE low). Free Play on boot.
 */
void setup(){
  Serial.begin(115200);
  for(int i=0;i<N_SERVO;++i){
    int16_t n = clampServoUs(i,1500);
    targetUs[i]=posUs[i]=lastPrintedUs[i]=n;
  }
#if USE_OE
  pinMode(OE_PIN,OUTPUT); digitalWrite(OE_PIN,HIGH);
#endif

  Wire.begin(SDA_PIN,SCL_PIN);
  pwm.begin(); pwm.setPWMFreq(SERVO_FREQ);

  prefs.begin("servos", false);
  if (prefs.getBytesLength("pos")==sizeof(posUs)){
    prefs.getBytes("pos", posUs, sizeof(posUs));
    for(int i=0;i<N_SERVO;++i){
      posUs[i]=clampServoUs(i,posUs[i]);
      targetUs[i]=lastPrintedUs[i]=posUs[i];
    }
    Serial.println("Restored positions from NVS.");
  } else {
    Serial.println("No saved positions; using neutral within limits.");
  }

  for(uint8_t i=0;i<N_SERVO;++i) pwm.setPWM(i,0,us2cnt(posUs[i]));
  delay(80);
#if USE_OE
  digitalWrite(OE_PIN,LOW);
#endif

  WiFi.mode(WIFI_STA);
  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); for(;;); }
  ensurePeer(CONTROLLER_MAC);
  esp_now_register_recv_cb(onRx);

  q=xQueueCreate(8,sizeof(Msg));
  xTaskCreatePinnedToCore(TaskRx,     "rx",     4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskSlew,   "slew",   4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskGame,   "game",   3072, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(TaskRandom, "random", 2048, nullptr, 1, nullptr, 1);

  Serial.println("Mask ready (Free on boot; long-press toggles modes; GAME_PREVIEW supported).");
}

void loop(){}  // all work is in tasks
