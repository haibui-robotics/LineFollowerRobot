/*
 * ============================================================================
 *  ROBOT DÒ LINE TỐC ĐỘ CAO - HUST ROBOTICS COMPETITION 2026 (SA BÀN CHUẨN)
 *  BẢN V0: HỖ TRỢ MAP_SCALE (0.85 TẬP / 1.00 THI), XE DÀI SENSOR_OFFSET_MM=170,
 *          XOAY GÓC TUYỆT ĐỐI ENCODER CHO TỔ ONG & VÒM CHỮ H, PHANH ĐÍCH CHUẨN,
 *          IRAM ENCODER ISR, CLI TINH CHỈNH NHANH (scale / sensx / coast / sstop)
 *  Encoder: ENABLE_ENCODERS = true (cần nối đủ 2 encoder)
 * ============================================================================
 */

#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <soc/soc.h>
#include <soc/gpio_reg.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

struct SensorResult {
  bool     hasLine;
  bool     leftBranch;
  bool     rightBranch;
  bool     tJunction;
  bool     ySplit;
  bool     leftEdge;
  bool     rightEdge;
  bool     invTransition;
  uint8_t  activeCount;
  float    error;
  uint32_t readUs;
};

// ================= 1. CẤU HÌNH CHIẾN THUẬT & CÔNG TẮC =================
const bool ENABLE_ENCODERS = true;

bool USE_HONEYCOMB_ROUTE_A = true;   // true: Phương án A | false: Phương án B
bool ENABLE_FEEDFORWARD    = true;
bool ENABLE_SPEED_PLANNER  = true;
bool ENABLE_CURVE_OPT      = true;
bool ENABLE_AUTO_INVERT    = true;
bool ENABLE_TRACKING_GATE  = true;
bool USE_ENCODER_PI        = true;
bool ENABLE_FINISH_STOP    = true;
bool ENABLE_STALL_RECOVERY = true;
bool ENABLE_E18_SENSOR     = false;

const bool ACTIVE_ON_BLACK = false;  // false: line đen ADC THẤP, nền trắng CAO

// Thông số cơ khí & vật lý
float MAP_SCALE        = 0.85f;  // 0.85 cho sa bàn in 85% | 1.00 cho sa bàn thi đấu chuẩn (lệnh scale=)
float SENSOR_OFFSET_MM = 170.0f;
float CHAIN_COAST_MM   = 13.0f;  // trừ quãng phanh turnStage 15 trước khi xoay (lệnh coast=)
float A_BRAKE_MMSS     = 3800.0f;
float A_ACCEL_MMSS     = 3500.0f;
float S_STOP_FINISH_MM = 235.0f;
const float D_FILTER_ALPHA = 0.38f;
const float STRAIGHT_KP_SC = 0.86f;
const float VEL_EMA_ALPHA  = 0.35f;

// ---- Tham số cho XE DÀI (chỉnh nhanh bằng CLI, lưu bằng 'save') ----
float CURVE_EXP      = 1.35f;  // nhỏ hơn = giảm tốc sớm hơn khi lệch line (cũ 1.90)
float EDGE_BOOST_MAX = 0.60f;  // khuếch đại lái khi |error| > 2.0 (0 = tắt)
float FF_SIGN        = -1.0f;  // đảo dấu feedforward độ cong nếu bù ngược (lệnh ffsign=1 / -1)
const float HEAD_HOLD_KP = 2.0f;          // giữ hướng khi đi qua chỗ đứt line
const unsigned long LOST_HOLD_MS = 30;    // giữ lệnh cũ khi mất line thoáng qua

// ================= 2. THÔNG SỐ ENCODER & CƠ KHÍ =================
const int8_t ENC_L_SIGN = -1;
const int8_t ENC_R_SIGN =  1;

const float WHEEL_DIAMETER_MM = 68.0f;
const float WHEEL_TRACK_MM    = 155.313f;
const float TICKS_PER_REV     = 408.3f;
const float MM_PER_TICK       = (3.1415926f * WHEEL_DIAMETER_MM) / TICKS_PER_REV;
const float RAD_TO_DEG_F      = 57.2957795f;
float       MAX_SPEED_MMS     = 1050.0f;  // đo thực tế từ log encoder: v_thực / pwm * 255
const int   DEFAULT_PWM_PIVOT_MIN = 64;   // đo từ log: <=54 bị đứng bánh, >=58 quay mượt
int         PWM_PIVOT_MIN         = DEFAULT_PWM_PIVOT_MIN;

// ================= 3. SƠ ĐỒ CHÂN PHẦN CỨNG =================
const int LINE_PINS[8] = {36, 39, 34, 35, 32, 33, 25, 26};
const float WEIGHTS[8] = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};

// TB6612FNG (đã đổi đúng cặp chân chiều AIN1/2 <-> BIN1/2 khớp PWMA trái & PWMB phải)
const int AIN1 = 18;
const int AIN2 = 21;
const int PWMA = 14;
const int BIN1 = 22;
const int BIN2 = 19;
const int PWMB = 27;
const int STBY = 23;

// 2 Encoder 4x
const int ENC_L_A = 4;
const int ENC_L_B = 5;    // GPIO5 là chân strapping (dùng input được, tránh kéo cứng khi boot)
const int ENC_R_A = 16;
const int ENC_R_B = 17;

// Nút nhấn, LED, cảm biến vật cản E18
// LƯU Ý: GPIO12 / GPIO2 / GPIO15 là chân strapping, không giữ nút / kéo mức cao ngoài lúc cấp nguồn
const int BTN_START_CALIB = 13;
const int BTN_MODE        = 12;
const int LED_PIN         = 2;
const int E18_PIN         = 15;

const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

// ================= 4. 3 CHẾ ĐỘ TỐC ĐỘ (LƯU TRONG NVS) =================
struct SpeedProfile {
  const char* name;
  float speedScale;
  int   maxPWM;
  int   minDynPWM;
  float kp;
  float kd;
  float kff;
};

const SpeedProfile DEFAULT_PROFILES[3] = {
  // Name    Scale   MaxPWM  MinDyn   Kp      Kd      Kff
  {"CHAM",   0.65f,  180,     85,   20.0f,  15.0f,  75.0f},
  {"VUA",    0.85f,  215,     90,   22.0f,  20.0f,  80.0f},
  {"NHANH",  1.00f,  245,     95,   25.0f,  28.0f,  85.0f}
};

SpeedProfile profiles[3] = {
  DEFAULT_PROFILES[0],
  DEFAULT_PROFILES[1],
  DEFAULT_PROFILES[2]
};

int   currentMode = 0;
float speedScale  = profiles[0].speedScale;
int   maxSpeed    = profiles[0].maxPWM;
int   minDynBase  = profiles[0].minDynPWM;
float kp          = profiles[0].kp;
float kd          = profiles[0].kd;
float kff         = profiles[0].kff;

float velKp        = 0.05f;
float velKi        = 0.10f;
float velIntClamp  = 300.0f;
float velIntegralL = 0.0f;
float velIntegralR = 0.0f;
float velL_filt    = 0.0f;
float velR_filt    = 0.0f;

// ================= 5. BẢNG SPEED ZONE (ĐƠN VỊ MM/S) =================
enum ZoneType : uint8_t {
  Z_LINE = 0,
  Z_CURVE,
  Z_INVERSION,
  Z_HONEYCOMB,
  Z_GEAR,
  Z_CURVED_GAP,
  Z_SHORTCUT,
  Z_HUST,
  Z_BUMP,
  Z_FINISH
};

struct SpeedZone {
  float    sStartMM;
  float    sEndMM;
  float    vMaxMMS;
  float    curvatureInvM;
  ZoneType type;
};

const SpeedZone ZONES_ROUTE_A[] = {
  {    0.0f,   220.0f,  950.0f,  0.00f, Z_LINE},
  {  220.0f,   840.0f, 1250.0f,  0.00f, Z_LINE},
  {  840.0f,  1293.0f,  620.0f, -8.00f, Z_CURVE},
  { 1293.0f,  1843.0f,  780.0f,  0.00f, Z_INVERSION},
  { 1843.0f,  2471.0f,  850.0f, +3.33f, Z_CURVE},
  { 2471.0f,  2700.0f,  900.0f,  0.00f, Z_LINE},
  { 2700.0f,  3040.0f,  620.0f, -8.00f, Z_CURVE},
  { 3040.0f,  3350.0f,  750.0f,  0.00f, Z_LINE},
  { 3350.0f,  3794.0f,  820.0f,  0.00f, Z_LINE},
  { 3794.0f,  4610.0f,  820.0f,  0.00f, Z_HONEYCOMB},
  { 4610.0f,  5050.0f,  650.0f,  0.00f, Z_LINE},
  { 5050.0f,  5302.0f,  580.0f,  0.00f, Z_LINE},
  { 5302.0f,  7200.0f,  600.0f,  0.00f, Z_GEAR},
  { 7200.0f,  7360.0f,  650.0f,  0.00f, Z_GEAR},
  { 7360.0f,  7520.0f,  650.0f, -1.74f, Z_GEAR},
  { 7520.0f,  8850.0f,  760.0f, -1.74f, Z_CURVED_GAP},
  { 8850.0f,  9680.0f,  950.0f,  0.00f, Z_SHORTCUT},
  { 9680.0f, 10050.0f, 1200.0f,  0.00f, Z_SHORTCUT},
  {10050.0f, 10260.0f,  800.0f,  0.00f, Z_SHORTCUT},
  {10260.0f, 10460.0f,  880.0f,  0.00f, Z_SHORTCUT},
  {10460.0f, 11180.0f, 1200.0f,  0.00f, Z_SHORTCUT},
  {11180.0f, 11520.0f,  900.0f,  0.00f, Z_BUMP},
  {11520.0f, 12020.0f,  820.0f,  0.00f, Z_BUMP},
  {12020.0f, 12180.0f,  760.0f,  0.00f, Z_BUMP},
  {12180.0f, 12280.0f,  800.0f,  0.00f, Z_HUST},
  {12280.0f, 12540.0f,  880.0f,  0.00f, Z_HUST},
  {12540.0f, 12680.0f,  760.0f,  0.00f, Z_HUST},
  {12680.0f, 13040.0f,  820.0f,  0.00f, Z_BUMP},
  {13040.0f, 13820.0f,  780.0f,  0.00f, Z_HUST},
  {13820.0f, 14040.0f, 1000.0f,  0.00f, Z_LINE},
  {14040.0f, 15200.0f, 1400.0f,  0.00f, Z_LINE},
  {15200.0f, 16000.0f,  750.0f,  0.00f, Z_FINISH}
};
const uint8_t NUM_ZONES_A = sizeof(ZONES_ROUTE_A) / sizeof(ZONES_ROUTE_A[0]);

const SpeedZone ZONES_ROUTE_B[] = {
  {    0.0f,   220.0f,  950.0f,  0.00f, Z_LINE},
  {  220.0f,   840.0f, 1250.0f,  0.00f, Z_LINE},
  {  840.0f,  1293.0f,  620.0f, -8.00f, Z_CURVE},
  { 1293.0f,  1843.0f,  780.0f,  0.00f, Z_INVERSION},
  { 1843.0f,  2471.0f,  850.0f, +3.33f, Z_CURVE},
  { 2471.0f,  2700.0f,  900.0f,  0.00f, Z_LINE},
  { 2700.0f,  3040.0f,  620.0f, -8.00f, Z_CURVE},
  { 3040.0f,  3794.0f,  800.0f,  0.00f, Z_LINE},
  { 3794.0f,  4841.0f,  800.0f,  0.00f, Z_HONEYCOMB},
  { 4841.0f,  5533.0f,  680.0f,  0.00f, Z_LINE},
  { 5533.0f,  7751.0f,  600.0f,  0.00f, Z_GEAR},
  { 7751.0f,  9081.0f,  760.0f, -1.74f, Z_CURVED_GAP},
  { 9081.0f, 11411.0f, 1050.0f,  0.00f, Z_SHORTCUT},
  {11411.0f, 12411.0f,  820.0f,  0.00f, Z_BUMP},
  {12411.0f, 12911.0f,  860.0f,  0.00f, Z_HUST},
  {12911.0f, 13271.0f,  820.0f,  0.00f, Z_BUMP},
  {13271.0f, 14051.0f,  780.0f,  0.00f, Z_HUST},
  {14051.0f, 15431.0f, 1400.0f,  0.00f, Z_LINE},
  {15431.0f, 16231.0f,  750.0f,  0.00f, Z_FINISH}
};
const uint8_t NUM_ZONES_B = sizeof(ZONES_ROUTE_B) / sizeof(ZONES_ROUTE_B[0]);

const uint16_t PLAN_STEP_MM = 50;
const uint16_t PLAN_LEN     = 340;
float vPlanMMS[PLAN_LEN];

// ================= 6. BẢNG NÚT GIAO (TỌA ĐỘ CHUẨN 100%, TỰ SCALE THEO MAP_SCALE) =================
// dir: +1 = rẽ PHẢI, -1 = rẽ TRÁI (heading giảm khi rẽ phải)
const uint8_t TF_NONE      = 0x00;
const uint8_t TF_PURE_ENC  = 0x01; // Xoay thuần encoder (không dùng lineCaught/stage 25) khi cạnh sau < SENSOR_OFFSET_MM
const uint8_t TF_ABS_START = 0x02; // Lưu mốc hướng tuyệt đối absRefHead khi bắt đầu chuỗi (Tổ ong / vòm chữ H)
const uint8_t TF_ABS_HEAD  = 0x04; // Xoay về góc tuyệt đối (absRefHead + absDeg) để chống cộng dồn sai số góc

struct TurnEvent {
  int8_t   dir;
  float    deg;
  float    sExpect;
  float    tol;
  ZoneType zone;
  float    chainAdvMM; // chiều dài đoạn nối hình học 100% (mm); >0 = tự nối chuỗi sau lượt rẽ trước
  float    absDeg;     // góc mục tiêu so với absRefHead khi bật TF_ABS_HEAD
  uint8_t  flags;
};

const TurnEvent PLAN_A[] = {
  // Tổ ong (7 lục giác cạnh 115.5mm, nửa cạnh 57.7mm): idx 0..7 xoay thuần encoder theo góc tuyệt đối, idx 8 bắt line thoát
  {+1,  90.0f,  3793.7f,  450.0f, Z_HONEYCOMB,   0.0f, -90.0f, TF_PURE_ENC | TF_ABS_START | TF_ABS_HEAD},
  {-1,  60.0f,  3851.5f,   80.0f, Z_HONEYCOMB,  57.7f, -30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  60.0f,  3966.9f,   80.0f, Z_HONEYCOMB, 115.5f, +30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  60.0f,  4082.4f,   80.0f, Z_HONEYCOMB, 115.5f, -30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  60.0f,  4197.9f,   80.0f, Z_HONEYCOMB, 115.5f, +30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  60.0f,  4313.3f,   80.0f, Z_HONEYCOMB, 115.5f, -30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  60.0f,  4428.8f,   80.0f, Z_HONEYCOMB, 115.5f, +30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  60.0f,  4544.3f,   80.0f, Z_HONEYCOMB, 115.5f, +90.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  90.0f,  4602.0f,   80.0f, Z_HONEYCOMB,  57.7f,   0.0f, TF_ABS_HEAD},
  // Giao đường chéo sau Tổ ong -> đi thẳng (tol 110mm để không kích nhầm ngay cửa ra Tổ ong)
  { 0,   0.0f,  5054.5f,  110.0f, Z_LINE,        0.0f,   0.0f, TF_NONE},
  // Lối tắt có thưởng: idx 10 chỉ neo tọa độ tại ngã tư sau R574 (cửa sổ rộng phủ đủ Mode 1..3), idx 11 rẽ góc nhọn 100°
  { 0,   0.0f,  8950.0f,  550.0f, Z_SHORTCUT,    0.0f,   0.0f, TF_NONE},
  {-1, 100.0f, 10194.0f,  220.0f, Z_SHORTCUT,    0.0f,   0.0f, TF_NONE},
  // HUST: góc trên S/U (idx 12) & góc dưới S/U (idx 13) xoay thuần encoder góc tuyệt đối để tránh bắt nhầm vách trong chữ U (arccos(85/170)=60°)
  {-1,  90.0f, 12550.0f,  550.0f, Z_BUMP,        0.0f, +90.0f, TF_PURE_ENC | TF_ABS_START | TF_ABS_HEAD},
  {+1,  90.0f, 12620.0f,  260.0f, Z_HUST,        0.0f,   0.0f, TF_PURE_ENC | TF_ABS_HEAD},
  // Vòm dưới chữ H (200-200-200-100mm; ở 85% còn 170-170-170-85mm): nối chuỗi góc tuyệt đối, idx 18 bắt line về đích
  {+1,  90.0f, 13500.0f, 1100.0f, Z_BUMP,        0.0f, -90.0f, TF_PURE_ENC | TF_ABS_START | TF_ABS_HEAD},
  {-1,  90.0f, 13320.0f,   75.0f, Z_HUST,      200.0f,   0.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  90.0f, 13520.0f,   75.0f, Z_HUST,      200.0f, +90.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  90.0f, 13720.0f,   75.0f, Z_HUST,      200.0f,   0.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  90.0f, 13820.0f,   90.0f, Z_HUST,      100.0f, +90.0f, TF_ABS_HEAD},
};
const uint8_t LEN_PLAN_A = sizeof(PLAN_A) / sizeof(PLAN_A[0]);

const TurnEvent PLAN_B[] = {
  {+1,  90.0f,  3793.7f,  240.0f, Z_HONEYCOMB,   0.0f, -90.0f, TF_PURE_ENC | TF_ABS_START | TF_ABS_HEAD},
  {-1,  60.0f,  3851.5f,   80.0f, Z_HONEYCOMB,  57.7f, -30.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  60.0f,  3966.9f,   80.0f, Z_HONEYCOMB, 115.5f, -90.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  60.0f,  4544.3f,  160.0f, Z_HONEYCOMB,   0.0f,   0.0f, TF_PURE_ENC},
  {+1,  90.0f,  4833.0f,  120.0f, Z_HONEYCOMB,   0.0f,   0.0f, TF_NONE},
  { 0,   0.0f,  5285.5f,  110.0f, Z_LINE,        0.0f,   0.0f, TF_NONE},
  { 0,   0.0f,  9181.0f,  550.0f, Z_SHORTCUT,    0.0f,   0.0f, TF_NONE},
  {-1, 100.0f, 10425.0f,  220.0f, Z_SHORTCUT,    0.0f,   0.0f, TF_NONE},
  {-1,  90.0f, 12781.0f,  550.0f, Z_BUMP,        0.0f, +90.0f, TF_PURE_ENC | TF_ABS_START | TF_ABS_HEAD},
  {+1,  90.0f, 12851.0f,  260.0f, Z_HUST,        0.0f,   0.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  90.0f, 13731.0f, 1100.0f, Z_BUMP,        0.0f, -90.0f, TF_PURE_ENC | TF_ABS_START | TF_ABS_HEAD},
  {-1,  90.0f, 13551.0f,   75.0f, Z_HUST,      200.0f,   0.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  90.0f, 13751.0f,   75.0f, Z_HUST,      200.0f, +90.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {+1,  90.0f, 13951.0f,   75.0f, Z_HUST,      200.0f,   0.0f, TF_PURE_ENC | TF_ABS_HEAD},
  {-1,  90.0f, 14051.0f,   90.0f, Z_HUST,      100.0f, +90.0f, TF_ABS_HEAD},
};
const uint8_t LEN_PLAN_B = sizeof(PLAN_B) / sizeof(PLAN_B[0]);

// ================= 7. RING-BUFFER LOG RAM (2400 MẪU) =================
struct LogSample {
  uint32_t tMs;
  float    sMM;
  int16_t  err100;
  int16_t  vRef;
  int16_t  vL;
  int16_t  vR;
  int16_t  head10;
  int16_t  pwmL;
  int16_t  pwmR;
  uint8_t  zone;
  uint8_t  jnc;
  uint8_t  act;
  uint8_t  flags;
};
const uint16_t LOG_CAP = 2400;
LogSample logBuf[LOG_CAP];
uint16_t  logHead = 0;
uint16_t  logCount = 0;
uint8_t   logDecim = 0;

extern int sensorMin[8];
extern int sensorMax[8];

void pushLog(uint32_t tMs, float sMM, float err, float vRef, float vL, float vR,
             float headDeg, int pwmL, int pwmR, uint8_t z, uint8_t j, uint8_t act, uint8_t flags) {
  logBuf[logHead] = {
    tMs,
    sMM,
    (int16_t)constrain((long)lroundf(err * 100.0f), -32000L, 32000L),
    (int16_t)constrain((long)lroundf(vRef), -32000L, 32000L),
    (int16_t)constrain((long)lroundf(vL), -32000L, 32000L),
    (int16_t)constrain((long)lroundf(vR), -32000L, 32000L),
    (int16_t)constrain((long)lroundf(headDeg * 10.0f), -32000L, 32000L),
    (int16_t)pwmL,
    (int16_t)pwmR,
    z, j, act, flags
  };
  logHead = (logHead + 1) % LOG_CAP;
  if (logCount < LOG_CAP) logCount++;
}

inline void maybePushLog(uint32_t tMs, float sMM, const SensorResult &sr, float vRef,
                         float vL, float vR, float headDeg, int pwmL, int pwmR,
                         uint8_t z, uint8_t j, bool invMode, uint8_t turnSt, bool finBrk) {
  if ((++logDecim & 1) == 0) {
    uint8_t fl = (sr.hasLine     ? 0x01 : 0)
               | (invMode        ? 0x02 : 0)
               | (sr.tJunction   ? 0x04 : 0)
               | (sr.leftBranch  ? 0x08 : 0)
               | (sr.rightBranch ? 0x10 : 0)
               | (sr.ySplit      ? 0x20 : 0)
               | ((turnSt > 0)   ? 0x40 : 0)
               | (finBrk         ? 0x80 : 0);
    pushLog(tMs, sMM, sr.error, vRef, vL, vR, headDeg, pwmL, pwmR, z, j, sr.activeCount, fl);
  }
}

void dumpLogCSV() {
  Serial.println("=== RING-BUFFER LOG (CSV) ===");
  Serial.printf("#META Mode=%d(%s) Route=%c MapScale=%.2f Scale=%.2f MaxPWM=%d MinDyn=%d Kp=%.2f Kd=%.2f Kff=%.2f\n",
                currentMode + 1, profiles[currentMode].name, USE_HONEYCOMB_ROUTE_A ? 'A' : 'B',
                MAP_SCALE, speedScale, maxSpeed, minDynBase, kp, kd, kff);
  Serial.println("t_ms,s_mm,err,vRef_mms,vL_mms,vR_mms,head_deg,pwmL,pwmR,zone,jncIdx,act,flags");
  uint16_t start = (logCount == LOG_CAP) ? logHead : 0;
  for (uint16_t i = 0; i < logCount; i++) {
    const LogSample &ls = logBuf[(start + i) % LOG_CAP];
    Serial.printf("%lu,%.1f,%.2f,%d,%d,%d,%.1f,%d,%d,%u,%u,%u,%u\n",
                  (unsigned long)ls.tMs, ls.sMM, ls.err100 * 0.01f,
                  ls.vRef, ls.vL, ls.vR, ls.head10 * 0.1f,
                  ls.pwmL, ls.pwmR, ls.zone, ls.jnc, ls.act, ls.flags);
  }
  Serial.println("=== END LOG ===");
}

// ================= 8. BIẾN TRẠNG THÁI, NVS & ENCODER ISR =================
Preferences prefs;

bool isRunning        = false;
bool invertedMode     = false;
bool debugSensorMode  = false;
bool debugEncoderMode = false;
bool missedRecovery   = false;
bool wasLostLine      = false;

float invEnterDistMM = 0.0f;
float invExitDistMM  = 0.0f;
uint8_t jncConfirmCnt = 0;
uint8_t btnStopConfirmCnt = 0;
unsigned long lastBtnSampleUs = 0;

const unsigned long LOOP_INTERVAL_US = 5000; // 200 Hz
unsigned long lastLoopUs  = 0;
unsigned long runStartMs  = 0;
unsigned long lastDebugMs = 0;

float lastError     = 0.0f;
float dFiltered     = 0.0f;
float totalDistMM   = 0.0f;
float headingDeg    = 0.0f;   // dương = quay TRÁI, âm = quay PHẢI
float straightMM    = 0.0f;
float vCmdPrev      = 0.0f;
int8_t lastSharpDir = 0;
float lostStartMM   = 0.0f;
unsigned long lostStartMs = 0;
int lastPwmL = 100, lastPwmR = 100;
float postTurnT = 1.0f;

// Trạng thái xử lý mất line
bool          lostInit          = false;
float         lostHeadDeg       = 0.0f;
float         lostErrSnap       = 0.0f;
int8_t        lostDir           = 1;
unsigned long lostSearchStartMs = 0;

uint8_t jncIdx        = 0;
int8_t  forcedDir     = 0;
float   forcedDeg     = 0.0f;
float   forcedAdvMM   = 155.0f;
bool    forcedPureEnc = false; // true: xoay thuần theo encoder, không dùng lineCaught/stage 25
bool    forcedUseAbs  = false; // true: xoay tới góc tuyệt đối forcedTargetHead
bool    hasAbsRef     = false; // true sau khi đã lấy mốc góc ngang absRefHead ở cửa vào Tổ ong
float   absRefHead    = 0.0f;  // mốc góc tuyệt đối khi bắt đầu chuỗi Tổ ong hoặc vòm chữ H
float   forcedTargetHead = 0.0f;
float   advHoldHead      = 0.0f; // góc giữ thẳng trong turnStage == 1
float   forcedChainExitS = -1.0f; // >=0: neo lại totalDistMM khi kết thúc bước cuối của chuỗi
uint8_t turnStage     = 0;     // 0: không rẽ | 1: tiến tới tâm nút | 2: xoay tại chỗ | 25: xoay chậm tìm line
float   turnStageT    = 0.0f;
float   turnBrakeT    = 0.0f;  // thời gian phanh chủ động tắt đà sau mỗi lượt xoay (45ms)
float   turnStartMM   = 0.0f;
float   turnStartHead = 0.0f;
float   straightLockUntilMM = -1.0f;
float   straightLockHead    = 0.0f;

bool  finishBraking  = false;
float finishEntryMM  = 0.0f;

unsigned long stallStartMs   = 0;
unsigned long reverseUntilMs = 0;
uint8_t       stallRetries   = 0;

int   sensorMin[8]      = {500, 500, 500, 500, 500, 500, 500, 500};
int   sensorMax[8]      = {3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500};
float invSensorRange[8] = {1.0f / 3000.0f, 1.0f / 3000.0f, 1.0f / 3000.0f, 1.0f / 3000.0f,
                           1.0f / 3000.0f, 1.0f / 3000.0f, 1.0f / 3000.0f, 1.0f / 3000.0f};

char    cliBuf[48];
uint8_t cliLen = 0;
char    runTrace[256] = "";
uint16_t runTraceLen = 0;

void appendRunTrace(const char* fmt, ...) {
  if (runTraceLen >= sizeof(runTrace) - 2) return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(runTrace + runTraceLen, sizeof(runTrace) - runTraceLen, fmt, ap);
  va_end(ap);
  if (n > 0) {
    runTraceLen = min((uint16_t)(sizeof(runTrace) - 1), (uint16_t)(runTraceLen + n));
  }
}

volatile int32_t encCountL = 0, encCountR = 0;
volatile uint8_t encStateL = 0, encStateR = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

DRAM_ATTR const int8_t QEM_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0,
};

static_assert(ENC_L_A < 32 && ENC_L_B < 32 && ENC_R_A < 32 && ENC_R_B < 32,
              "Encoder pins must be < 32 for fast GPIO_IN_REG read");

void IRAM_ATTR isrEncL() {
  portENTER_CRITICAL_ISR(&encMux);
  uint32_t in = REG_READ(GPIO_IN_REG);
  uint8_t curr = (((in >> ENC_L_A) & 1U) << 1) | ((in >> ENC_L_B) & 1U);
  encCountL += QEM_TABLE[(encStateL << 2) | curr];
  encStateL = curr;
  portEXIT_CRITICAL_ISR(&encMux);
}

void IRAM_ATTR isrEncR() {
  portENTER_CRITICAL_ISR(&encMux);
  uint32_t in = REG_READ(GPIO_IN_REG);
  uint8_t curr = (((in >> ENC_R_A) & 1U) << 1) | ((in >> ENC_R_B) & 1U);
  encCountR += QEM_TABLE[(encStateR << 2) | curr];
  encStateR = curr;
  portEXIT_CRITICAL_ISR(&encMux);
}

void readEncoders(int32_t &outL, int32_t &outR) {
  portENTER_CRITICAL(&encMux);
  outL = encCountL * ENC_L_SIGN;
  outR = encCountR * ENC_R_SIGN;
  portEXIT_CRITICAL(&encMux);
}

void resetEncoders() {
  portENTER_CRITICAL(&encMux);
  encCountL = 0;
  encCountR = 0;
  portEXIT_CRITICAL(&encMux);
}

int32_t prevTickL = 0, prevTickR = 0;

// ================= 9. NVS STORAGE =================
void updateInvSensorRange() {
  for (int i = 0; i < 8; i++) {
    invSensorRange[i] = 1.0f / max(50.0f, (float)(sensorMax[i] - sensorMin[i]));
  }
}

void saveConfigToNVS() {
  if (prefs.begin("hrc2026", false)) {
    prefs.putBytes("sMin", sensorMin, sizeof(sensorMin));
    prefs.putBytes("sMax", sensorMax, sizeof(sensorMax));
    prefs.putInt("mode", currentMode);
    prefs.putBool("routeA", USE_HONEYCOMB_ROUTE_A);
    prefs.putFloat("mapSc", MAP_SCALE);
    prefs.putFloat("sensX", SENSOR_OFFSET_MM);
    prefs.putFloat("coast", CHAIN_COAST_MM);
    prefs.putFloat("aBrake", A_BRAKE_MMSS);
    prefs.putFloat("aAccel", A_ACCEL_MMSS);
    prefs.putFloat("sStop", S_STOP_FINISH_MM);
    prefs.putFloat("maxV", MAX_SPEED_MMS);
    prefs.putFloat("velKp", velKp);
    prefs.putFloat("velKi", velKi);
    prefs.putInt("pivMin", PWM_PIVOT_MIN);
    prefs.putFloat("cExp", CURVE_EXP);
    prefs.putFloat("boost", EDGE_BOOST_MAX);
    prefs.putFloat("ffSign", FF_SIGN);

    float pKp[3], pKd[3], pKff[3];
    for (int i = 0; i < 3; i++) {
      pKp[i]  = profiles[i].kp;
      pKd[i]  = profiles[i].kd;
      pKff[i] = profiles[i].kff;
    }
    prefs.putBytes("pKp", pKp, sizeof(pKp));
    prefs.putBytes("pKd", pKd, sizeof(pKd));
    prefs.putBytes("pKff", pKff, sizeof(pKff));
    prefs.end();
    Serial.println("[NVS] Da luu cau hinh vao Flash!");
  }
}

void loadConfigFromNVS() {
  if (prefs.begin("hrc2026", true)) {
    if (prefs.getBytesLength("sMin") == sizeof(sensorMin) &&
        prefs.getBytesLength("sMax") == sizeof(sensorMax)) {
      int tmpMin[8], tmpMax[8];
      prefs.getBytes("sMin", tmpMin, sizeof(tmpMin));
      prefs.getBytes("sMax", tmpMax, sizeof(tmpMax));
      for (int i = 0; i < 8; i++) {
        if (tmpMin[i] >= 0 && tmpMax[i] <= 4095 && (tmpMax[i] - tmpMin[i]) >= 150) {
          sensorMin[i] = tmpMin[i];
          sensorMax[i] = tmpMax[i];
        }
      }
    }
    int   vMode   = prefs.getInt("mode", 0);
    int   vPivMin = prefs.getInt("pivMin", DEFAULT_PWM_PIVOT_MIN);
    float vMapSc  = prefs.getFloat("mapSc", 0.85f);
    float vSensX  = prefs.getFloat("sensX", 170.0f);
    float vCoast  = prefs.getFloat("coast", 13.0f);
    float vABrake = prefs.getFloat("aBrake", 3800.0f);
    float vAAccel = prefs.getFloat("aAccel", 3500.0f);
    float vSStop  = prefs.getFloat("sStop", 235.0f);
    float vMaxV   = prefs.getFloat("maxV", 1050.0f);
    float vVelKp  = prefs.getFloat("velKp", 0.05f);
    float vVelKi  = prefs.getFloat("velKi", 0.10f);
    float vCExp   = prefs.getFloat("cExp", 1.35f);
    float vBoost  = prefs.getFloat("boost", 0.60f);
    float vFfSign = prefs.getFloat("ffSign", -1.0f);

    currentMode           = constrain(vMode, 0, 2);
    USE_HONEYCOMB_ROUTE_A = prefs.getBool("routeA", true);
    MAP_SCALE             = isfinite(vMapSc)  ? constrain(vMapSc,  0.50f, 1.20f)    : 0.85f;
    SENSOR_OFFSET_MM      = isfinite(vSensX)  ? constrain(vSensX,  40.0f, 300.0f)   : 170.0f;
    CHAIN_COAST_MM        = isfinite(vCoast)  ? constrain(vCoast,  0.0f,  40.0f)    : 13.0f;
    A_BRAKE_MMSS          = isfinite(vABrake) ? constrain(vABrake, 500.0f, 15000.0f): 3800.0f;
    A_ACCEL_MMSS          = isfinite(vAAccel) ? constrain(vAAccel, 500.0f, 15000.0f): 3500.0f;
    S_STOP_FINISH_MM      = isfinite(vSStop)  ? constrain(vSStop,  50.0f, 400.0f)   : 235.0f;
    MAX_SPEED_MMS         = isfinite(vMaxV)   ? constrain(vMaxV,   400.0f, 3500.0f) : 1050.0f;
    velKp                 = isfinite(vVelKp)  ? constrain(vVelKp,  0.0f, 2.0f)      : 0.05f;
    velKi                 = isfinite(vVelKi)  ? constrain(vVelKi,  0.0f, 5.0f)      : 0.10f;
    PWM_PIVOT_MIN         = constrain(vPivMin, 25, 110);
    CURVE_EXP             = isfinite(vCExp)   ? constrain(vCExp,   0.8f, 3.0f)      : 1.35f;
    EDGE_BOOST_MAX        = isfinite(vBoost)  ? constrain(vBoost,  0.0f, 1.5f)      : 0.60f;
    FF_SIGN               = (isfinite(vFfSign) && vFfSign >= 0.0f) ? 1.0f : -1.0f;

    float pKp[3], pKd[3], pKff[3];
    if (prefs.getBytesLength("pKp") == sizeof(pKp)) {
      prefs.getBytes("pKp", pKp, sizeof(pKp));
      for (int i = 0; i < 3; i++) {
        if (isfinite(pKp[i])) profiles[i].kp = constrain(pKp[i], 0.0f, 200.0f);
      }
    }
    if (prefs.getBytesLength("pKd") == sizeof(pKd)) {
      prefs.getBytes("pKd", pKd, sizeof(pKd));
      for (int i = 0; i < 3; i++) {
        if (isfinite(pKd[i])) profiles[i].kd = constrain(pKd[i], 0.0f, 200.0f);
      }
    }
    if (prefs.getBytesLength("pKff") == sizeof(pKff)) {
      prefs.getBytes("pKff", pKff, sizeof(pKff));
      for (int i = 0; i < 3; i++) {
        if (isfinite(pKff[i])) profiles[i].kff = constrain(pKff[i], 0.0f, 400.0f);
      }
    }
    String lr = prefs.getString("lastRun", "");
    if (lr.length() > 0) {
      strncpy(runTrace, lr.c_str(), sizeof(runTrace) - 1);
      runTrace[sizeof(runTrace) - 1] = '\0';
      runTraceLen = strlen(runTrace);
    }
    prefs.end();
  }
  updateInvSensorRange();
}

// ================= 10. QUÉT NGƯỢC PHANH VẬT LÝ =================
void computeBackwardBrakeProfile() {
  const SpeedZone* zArr = USE_HONEYCOMB_ROUTE_A ? ZONES_ROUTE_A : ZONES_ROUTE_B;
  uint8_t zLen          = USE_HONEYCOMB_ROUTE_A ? NUM_ZONES_A : NUM_ZONES_B;
  float ms              = MAP_SCALE;
  float tightCurveSc    = sqrtf(min(1.0f, ms)); // khi sa bàn thu nhỏ, bán kính cua nhỏ đi -> giảm v theo sqrt(R)

  for (uint16_t i = 0; i < PLAN_LEN; i++) {
    float s = i * (float)PLAN_STEP_MM;
    float v = zArr[zLen - 1].vMaxMMS * speedScale;
    for (uint8_t z = 0; z < zLen; z++) {
      if (s >= zArr[z].sStartMM * ms && s <= zArr[z].sEndMM * ms) {
        float cSc = (zArr[z].type == Z_CURVE || zArr[z].type == Z_GEAR) ? tightCurveSc : 1.0f;
        v = zArr[z].vMaxMMS * speedScale * cSc;
        break;
      }
    }
    vPlanMMS[i] = v;
  }
  for (int i = PLAN_LEN - 2; i >= 0; i--) {
    float vNext = vPlanMMS[i + 1];
    float vAllowed = sqrtf(vNext * vNext + 2.0f * A_BRAKE_MMSS * (float)PLAN_STEP_MM);
    if (vPlanMMS[i] > vAllowed) vPlanMMS[i] = vAllowed;
  }
}

void queryPlan(float sMM, float &outV_mms, float &outKappa, ZoneType &outType) {
  float ms    = MAP_SCALE;
  float sAxle = max(0.0f, sMM);
  float sMid  = max(0.0f, sMM + 55.0f);
  float sSens = max(0.0f, sMM + SENSOR_OFFSET_MM);
  uint16_t idxA = min((uint16_t)(PLAN_LEN - 1), (uint16_t)(sAxle / PLAN_STEP_MM));
  uint16_t idxS = min((uint16_t)(PLAN_LEN - 1), (uint16_t)(sSens / PLAN_STEP_MM));
  outV_mms = min(vPlanMMS[idxA], vPlanMMS[idxS]);

  const SpeedZone* zArr = USE_HONEYCOMB_ROUTE_A ? ZONES_ROUTE_A : ZONES_ROUTE_B;
  uint8_t zLen          = USE_HONEYCOMB_ROUTE_A ? NUM_ZONES_A : NUM_ZONES_B;
  outKappa = 0.0f;
  for (uint8_t z = 0; z < zLen; z++) {
    if (sMid >= zArr[z].sStartMM * ms && sMid <= zArr[z].sEndMM * ms) {
      outKappa = zArr[z].curvatureInvM / ms;
      break;
    }
  }
  if (sSens > zArr[zLen - 1].sEndMM * ms) {
    outType = zArr[zLen - 1].type;
    return;
  }
  outType = Z_LINE;
  for (uint8_t z = 0; z < zLen; z++) {
    if (sSens >= zArr[z].sStartMM * ms && sSens <= zArr[z].sEndMM * ms) {
      outType = zArr[z].type;
      return;
    }
  }
}

// ================= 11. ĐỘNG CƠ, LED, PROFILE & CALIB =================
void setMotorRaw(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (leftPWM < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
  }

  if (rightPWM > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (rightPWM < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
  }

  ledcWrite(PWMA, abs(leftPWM));
  ledcWrite(PWMB, abs(rightPWM));
}

void resetWheelPI() {
  velIntegralL = 0.0f;
  velIntegralR = 0.0f;
}

void stopRobot() {
  bool wasRunning     = isRunning;
  isRunning           = false;
  forcedDir           = 0;
  forcedPureEnc       = false;
  forcedUseAbs        = false;
  forcedChainExitS    = -1.0f;
  turnStage           = 0;
  turnBrakeT          = 0.0f;
  reverseUntilMs      = 0;
  stallStartMs        = 0;
  finishBraking       = false;
  straightLockUntilMM = -1.0f;
  btnStopConfirmCnt   = 0;
  setMotorRaw(0, 0);
  resetWheelPI();
  digitalWrite(LED_PIN, LOW);
  if (wasRunning && totalDistMM > 100.0f) {
    appendRunTrace("END@%.0f(j%u,h%.0f,e%.1f)", totalDistMM, jncIdx, headingDeg, lastError);
    if (prefs.begin("hrc2026", false)) {
      prefs.putString("lastRun", runTrace);
      prefs.end();
    }
  }
}

void blinkLedBlocking(int times, int ms = 120) {
  if (isRunning) return;
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}

void applyProfile(int idx, bool persistNVS = true) {
  currentMode = idx % 3;
  speedScale  = profiles[currentMode].speedScale;
  maxSpeed    = profiles[currentMode].maxPWM;
  minDynBase  = profiles[currentMode].minDynPWM;
  kp          = profiles[currentMode].kp;
  kd          = profiles[currentMode].kd;
  kff         = profiles[currentMode].kff;
  computeBackwardBrakeProfile();
  if (persistNVS) saveConfigToNVS();
  Serial.printf(">>> MODE %d (%s): Scale=%.2f Kp=%.1f Kd=%.1f Kff=%.1f\n",
                currentMode + 1, profiles[currentMode].name, speedScale, kp, kd, kff);
  blinkLedBlocking(currentMode + 1, 140);
}

void calibrateSensors() {
  stopRobot();
  debugSensorMode  = false;
  debugEncoderMode = false;
  Serial.println(">>> CALIB 8 MAT IR (Dat xe sat san, quet qua lai line den & nen trang trong 3s)...");
  for (int i = 0; i < 8; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    digitalWrite(LED_PIN, (millis() / 100) % 2);
    for (int i = 0; i < 8; i++) {
      int v = (analogRead(LINE_PINS[i]) + analogRead(LINE_PINS[i])) >> 1;
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
    delay(2);
  }
  digitalWrite(LED_PIN, LOW);
  Serial.print(">>> IR Contrast (Max-Min): ");
  for (int i = 0; i < 8; i++) {
    int span = sensorMax[i] - sensorMin[i];
    Serial.printf("[%d:%d] ", i, span);
    if (span < 150) {
      Serial.print("(!YEU) ");
      sensorMin[i] = 500;
      sensorMax[i] = 3500;
    }
  }
  Serial.println();
  updateInvSensorRange();
  saveConfigToNVS();
  blinkLedBlocking(3, 100);
}

void resetConfigToDefault() {
  if (prefs.begin("hrc2026", false)) {
    prefs.clear();
    prefs.end();
  }
  for (int i = 0; i < 3; i++) profiles[i] = DEFAULT_PROFILES[i];
  for (int i = 0; i < 8; i++) { sensorMin[i] = 500; sensorMax[i] = 3500; }
  updateInvSensorRange();
  USE_HONEYCOMB_ROUTE_A = true;
  MAP_SCALE             = 0.85f;
  SENSOR_OFFSET_MM      = 170.0f;
  CHAIN_COAST_MM        = 13.0f;
  A_BRAKE_MMSS          = 3800.0f;
  A_ACCEL_MMSS          = 3500.0f;
  S_STOP_FINISH_MM      = 235.0f;
  MAX_SPEED_MMS         = 1050.0f;
  velKp                 = 0.05f;
  velKi                 = 0.10f;
  PWM_PIVOT_MIN         = DEFAULT_PWM_PIVOT_MIN;
  CURVE_EXP             = 1.35f;
  EDGE_BOOST_MAX        = 0.60f;
  FF_SIGN               = -1.0f;
  applyProfile(0, false);
  Serial.println("[NVS] Da xoa NVS & khoi phuc cau hinh mac dinh!");
}

// ================= 12. ĐỌC 8 MẮT IR =================
SensorResult readLineSensors(ZoneType zType, float dsAbsMM) {
  uint32_t t0 = micros();
  SensorResult res = {false, false, false, false, false, false, false, false, 0, 0.0f, 0};
  float dark[8];
  int darkCnt = 0;

  for (int i = 0; i < 8; i++) {
    int raw = (analogRead(LINE_PINS[i]) + analogRead(LINE_PINS[i])) >> 1;
    float norm = constrain((float)(raw - sensorMin[i]) * invSensorRange[i], 0.0f, 1.0f);
    dark[i] = ACTIVE_ON_BLACK ? norm : (1.0f - norm);
    if (dark[i] > 0.50f) darkCnt++;
  }

  float ms   = MAP_SCALE;
  float sEff = totalDistMM + SENSOR_OFFSET_MM;
  bool inInvWindow = (sEff >= 1150.0f * ms && sEff <= 2050.0f * ms);
  if (ENABLE_AUTO_INVERT && (inInvWindow || invertedMode)) {
    bool lw = (dark[0] > 0.55f) || (dark[1] > 0.60f);
    bool rw = (dark[7] > 0.55f) || (dark[6] > 0.60f);
    bool midW = (dark[2] < 0.45f) || (dark[3] < 0.45f) || (dark[4] < 0.45f) || (dark[5] < 0.45f);
    bool outW = (dark[0] < 0.35f) && (dark[1] < 0.35f) && (dark[6] < 0.35f) && (dark[7] < 0.35f);
    if (!invertedMode) {
      if (inInvWindow && lw && rw && midW && darkCnt >= 4 && darkCnt <= 7) {
        invEnterDistMM += dsAbsMM;
        if (invEnterDistMM >= 20.0f) { invertedMode = true; invEnterDistMM = 0.0f; }
      } else {
        invEnterDistMM = 0.0f;
      }
    } else {
      if ((outW && darkCnt <= 3) || !inInvWindow) {
        invExitDistMM += dsAbsMM;
        if (invExitDistMM >= 25.0f || !inInvWindow) {
          invertedMode = false;
          invExitDistMM = 0.0f;
          if (inInvWindow && sEff >= 1650.0f * ms) {
            totalDistMM = max(0.0f, 1818.0f * ms - SENSOR_OFFSET_MM);
          }
        }
      } else {
        invExitDistMM = 0.0f;
      }
    }
  } else {
    invertedMode = false;
    invEnterDistMM = invExitDistMM = 0.0f;
  }
  res.invTransition = (invEnterDistMM > 0.0f) || (invExitDistMM > 0.0f);

  float s[8], sClean[8];
  for (int i = 0; i < 8; i++) {
    s[i] = invertedMode ? (1.0f - dark[i]) : dark[i];
    if (s[i] < 0.25f) s[i] = 0.0f;
  }
  if (invertedMode && (s[1] + s[2] + s[3] + s[4] + s[5] + s[6]) > 0.35f) {
    if (dark[1] > 0.50f) s[0] = 0.0f;
    if (dark[6] > 0.50f) s[7] = 0.0f;
  }
  for (int i = 0; i < 8; i++) sClean[i] = s[i];

  float rShift = (USE_HONEYCOMB_ROUTE_A ? 0.0f : 231.0f) * ms;

  bool inRewBox = (zType == Z_SHORTCUT && sEff >= (9620.0f * ms + rShift) && sEff <= (10020.0f * ms + rShift));
  if (inRewBox && (sClean[2] + sClean[3] + sClean[4] + sClean[5]) > 0.35f) {
    sClean[0] = sClean[1] = sClean[6] = sClean[7] = 0.0f;
  }

  int clStart[4], clEnd[4], clCnt = 0;
  bool inCl = false;
  for (int i = 0; i < 8; i++) {
    if (sClean[i] > 0.40f) {
      if (!inCl && clCnt < 4) { clStart[clCnt] = i; inCl = true; }
      if (inCl) clEnd[clCnt] = i;
    } else if (inCl) { clCnt++; inCl = false; }
  }
  if (inCl) clCnt++;

  if (ENABLE_TRACKING_GATE && clCnt >= 2) {
    int bestCl = 0;
    if ((zType == Z_LINE && sEff >= (4750.0f * ms + rShift) && sEff <= (5350.0f * ms + rShift)) ||
        (zType == Z_GEAR && sEff < (7300.0f * ms + rShift))) {
      bestCl = clCnt - 1;
    } else if (zType == Z_CURVED_GAP || (zType == Z_GEAR && sEff >= (7300.0f * ms + rShift))) {
      bestCl = 0;
    } else if (zType == Z_SHORTCUT && sEff < (10100.0f * ms + rShift)) {
      bestCl = 0;
    } else if ((sEff >= (11150.0f * ms + rShift) && sEff <= (11520.0f * ms + rShift)) ||
               (sEff >= (12120.0f * ms + rShift) && sEff <= (12560.0f * ms + rShift))) {
      bestCl = clCnt - 1;
    } else {
      float bestDiff = 999.0f;
      for (int c = 0; c < clCnt; c++) {
        float d = fabsf(0.5f * (WEIGHTS[clStart[c]] + WEIGHTS[clEnd[c]]) - lastError);
        if (d < bestDiff) { bestDiff = d; bestCl = c; }
      }
    }
    for (int i = 0; i < 8; i++) {
      if (i < clStart[bestCl] || i > clEnd[bestCl]) sClean[i] = 0.0f;
    }
  }

  float sumW = 0.0f, sumS = 0.0f;
  for (int i = 0; i < 8; i++) {
    if (s[i] > 0.45f) res.activeCount++;
    sumW += WEIGHTS[i] * sClean[i];
    sumS += sClean[i];
  }

  res.tJunction   = (!invertedMode) && (s[0] > 0.45f && s[7] > 0.45f) && (res.activeCount >= 5);
  res.leftBranch  = (!invertedMode) && (s[0] > 0.45f && s[1] > 0.45f && s[2] > 0.45f) && (s[7] < 0.40f);
  res.rightBranch = (!invertedMode) && (s[7] > 0.45f && s[6] > 0.45f && s[5] > 0.45f) && (s[0] < 0.40f);
  res.ySplit      = (!invertedMode) && ((clCnt >= 2) || (res.activeCount >= 4));
  res.leftEdge    = (!invertedMode) && ((s[0] > 0.45f) || (s[1] > 0.50f));
  res.rightEdge   = (!invertedMode) && ((s[7] > 0.45f) || (s[6] > 0.50f));

  if (sumS >= 0.40f && res.activeCount > 0) {
    res.hasLine = true;
    float err = sumW / sumS;
    // Vùng giao 2 đường chéo sau Tổ ong (p_split): không cho nhánh chéo bên trái kéo xe rẽ trái
    if (sEff >= (4750.0f * ms + rShift) && sEff <= (5350.0f * ms + rShift) &&
        res.activeCount >= 3 && err < -0.5f) err = 0.0f;
    if (sEff >= (7350.0f * ms + rShift) && sEff <= (7800.0f * ms + rShift) && res.activeCount >= 2 && err > 1.2f) err = 0.3f;
    if (sEff >= (11180.0f * ms + rShift) && sEff <= (11480.0f * ms + rShift) && res.activeCount >= 3 && err < -1.0f) err = -0.3f;
    uint8_t idxSU  = USE_HONEYCOMB_ROUTE_A ? 13 : 9;
    uint8_t idxHIn = USE_HONEYCOMB_ROUTE_A ? 14 : 10;
    if (jncIdx == idxSU && sEff >= (12120.0f * ms + rShift) && sEff <= (12560.0f * ms + rShift) &&
        res.activeCount >= 3 && err < -0.8f) err = 0.0f;
    if (jncIdx == idxHIn && res.activeCount >= 4 && err > 0.6f) err = 0.0f;
    res.error = err;
  }
  res.readUs = micros() - t0;
  return res;
}

// ================= 13. KHỞI ĐỘNG LƯỢT CHẠY =================
void startRun() {
  resetEncoders();
  prevTickL = prevTickR = 0;
  totalDistMM = 0.0f;
  headingDeg  = 0.0f;
  straightMM  = 0.0f;
  vCmdPrev    = 0.0f;
  lastError   = 0.0f;
  dFiltered   = 0.0f;
  velL_filt = velR_filt = 0.0f;
  resetWheelPI();
  invertedMode = false;
  invEnterDistMM = invExitDistMM = 0.0f;
  jncConfirmCnt = 0;
  btnStopConfirmCnt = 0;
  lastBtnSampleUs = micros();
  jncIdx = 0;
  forcedDir = 0;
  forcedDeg = 0.0f;
  forcedAdvMM = max(75.0f, SENSOR_OFFSET_MM - 15.0f);
  forcedPureEnc = false;
  forcedUseAbs  = false;
  hasAbsRef     = false;
  absRefHead    = 0.0f;
  forcedTargetHead = 0.0f;
  advHoldHead      = 0.0f;
  forcedChainExitS = -1.0f;
  runTrace[0]   = '\0';
  runTraceLen   = 0;
  turnStage = 0;
  turnStageT = 0.0f;
  turnBrakeT = 0.0f;
  turnStartMM = 0.0f;
  turnStartHead = 0.0f;
  straightLockUntilMM = -1.0f;
  straightLockHead    = 0.0f;
  postTurnT = 1.0f;
  missedRecovery = false;
  wasLostLine    = false;
  lostInit       = false;
  lostHeadDeg    = 0.0f;
  lostErrSnap    = 0.0f;
  lostDir        = 1;
  lostSearchStartMs = 0;
  lastSharpDir   = 0;
  lastPwmL = 100;
  lastPwmR = 100;
  lostStartMM = 0.0f;
  lostStartMs = millis();
  finishBraking = false;
  finishEntryMM = 0.0f;
  stallStartMs = 0;
  reverseUntilMs = 0;
  stallRetries = 0;
  logHead = 0; logCount = 0; logDecim = 0;
  debugSensorMode  = false;
  debugEncoderMode = false;

  computeBackwardBrakeProfile();
  runStartMs = millis();
  lastLoopUs = micros();
  isRunning  = true;
  digitalWrite(LED_PIN, HIGH);
  Serial.printf(">>> START RUN! Mode=%s Route=%c MapScale=%.2f Enc=%d\n",
                profiles[currentMode].name, USE_HONEYCOMB_ROUTE_A ? 'A' : 'B', MAP_SCALE, ENABLE_ENCODERS);
}

// ================= 14. SERIAL CLI NON-BLOCKING =================
void printStatus() {
  Serial.println("=== STATUS (V0.9) ===");
  Serial.printf("Mode=%d (%s) | Route=%c | MapScale=%.2f | Running=%d | Enc=%d\n",
                currentMode + 1, profiles[currentMode].name,
                USE_HONEYCOMB_ROUTE_A ? 'A' : 'B', MAP_SCALE, isRunning, ENABLE_ENCODERS);
  Serial.printf("Scale=%.2f MaxPWM=%d MinDyn=%d | Kp=%.2f Kd=%.2f Kff=%.2f\n",
                speedScale, maxSpeed, minDynBase, kp, kd, kff);
  Serial.printf("MaxV=%.0f SensX=%.1f Coast=%.1f SStop=%.0f PivMin=%d Exp=%.2f Boost=%.2f FFsign=%+.0f\n",
                MAX_SPEED_MMS, SENSOR_OFFSET_MM, CHAIN_COAST_MM, S_STOP_FINISH_MM,
                PWM_PIVOT_MIN, CURVE_EXP, EDGE_BOOST_MAX, FF_SIGN);
  Serial.print("IR Min/Max: ");
  for (int i = 0; i < 8; i++) Serial.printf("[%d:%d/%d] ", i, sensorMin[i], sensorMax[i]);
  Serial.println();
  if (runTraceLen > 0) Serial.printf("LastRun: %s\n", runTrace);
}

void executeCLICommand(const char* rawCmd) {
  char cmd[48];
  const char* p = rawCmd;
  while (*p == ' ' || *p == '\t') p++;
  size_t len = strlen(p);
  while (len > 0 && (p[len - 1] == ' ' || p[len - 1] == '\t')) len--;
  if (len == 0) return;
  if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
  memcpy(cmd, p, len);
  cmd[len] = '\0';

  if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "x") == 0 || strcmp(cmd, "X") == 0) {
    stopRobot();
    Serial.println(">>> STOPPED BY SERIAL");
    return;
  }
  if (isRunning) return;

  if (strcmp(cmd, "go") == 0) { startRun(); }
  else if (strcmp(cmd, "calib") == 0) { calibrateSensors(); }
  else if (strcmp(cmd, "dump") == 0) { dumpLogCSV(); }
  else if (strcmp(cmd, "status") == 0) { printStatus(); }
  else if (strcmp(cmd, "save") == 0) { saveConfigToNVS(); }
  else if (strcmp(cmd, "resetnvs") == 0) { resetConfigToDefault(); }
  else if (strcmp(cmd, "mode1") == 0) { applyProfile(0, true); }
  else if (strcmp(cmd, "mode2") == 0) { applyProfile(1, true); }
  else if (strcmp(cmd, "mode3") == 0) { applyProfile(2, true); }
  else if (strcmp(cmd, "route=A") == 0 || strcmp(cmd, "route=a") == 0) {
    USE_HONEYCOMB_ROUTE_A = true;
    computeBackwardBrakeProfile();
    saveConfigToNVS();
    Serial.println(">>> Route A");
  }
  else if (strcmp(cmd, "route=B") == 0 || strcmp(cmd, "route=b") == 0) {
    USE_HONEYCOMB_ROUTE_A = false;
    computeBackwardBrakeProfile();
    saveConfigToNVS();
    Serial.println(">>> Route B");
  }
  else if (strcmp(cmd, "ir") == 0) {
    debugSensorMode = !debugSensorMode;
    if (debugSensorMode) debugEncoderMode = false;
    Serial.printf("ir=%d\n", debugSensorMode);
  }
  else if (strcmp(cmd, "enc") == 0) {
    debugEncoderMode = !debugEncoderMode;
    if (debugEncoderMode) {
      debugSensorMode = false;
      resetEncoders();
      prevTickL = prevTickR = 0;
      headingDeg = 0.0f;
    }
    Serial.printf("enc=%d\n", debugEncoderMode);
  }
  else if (strncmp(cmd, "kp=", 3) == 0) {
    kp = constrain(atof(cmd + 3), 0.0f, 200.0f);
    profiles[currentMode].kp = kp;
    Serial.printf("kp=%.2f (go 'save' de luu)\n", kp);
  }
  else if (strncmp(cmd, "kd=", 3) == 0) {
    kd = constrain(atof(cmd + 3), 0.0f, 200.0f);
    profiles[currentMode].kd = kd;
    Serial.printf("kd=%.2f (go 'save' de luu)\n", kd);
  }
  else if (strncmp(cmd, "kff=", 4) == 0) {
    kff = constrain(atof(cmd + 4), 0.0f, 400.0f);
    profiles[currentMode].kff = kff;
    Serial.printf("kff=%.2f (go 'save' de luu)\n", kff);
  }
  else if (strncmp(cmd, "ffsign=", 7) == 0) {
    FF_SIGN = (atof(cmd + 7) >= 0.0f) ? 1.0f : -1.0f;
    Serial.printf("ffsign=%+.0f (go 'save' de luu)\n", FF_SIGN);
  }
  else if (strncmp(cmd, "maxv=", 5) == 0) {
    MAX_SPEED_MMS = constrain((float)atof(cmd + 5), 400.0f, 3500.0f);
    Serial.printf("maxv=%.0f mm/s (go 'save' de luu)\n", MAX_SPEED_MMS);
  }
  else if (strncmp(cmd, "boost=", 6) == 0) {
    EDGE_BOOST_MAX = constrain((float)atof(cmd + 6), 0.0f, 1.5f);
    Serial.printf("boost=%.2f (go 'save' de luu)\n", EDGE_BOOST_MAX);
  }
  else if (strncmp(cmd, "exp=", 4) == 0) {
    CURVE_EXP = constrain((float)atof(cmd + 4), 0.8f, 3.0f);
    Serial.printf("exp=%.2f (go 'save' de luu)\n", CURVE_EXP);
  }
  else if (strncmp(cmd, "piv=", 4) == 0) {
    int v = atoi(cmd + 4);
    PWM_PIVOT_MIN = constrain(v, 25, 110);
    Serial.printf("piv=%d (go 'save' de luu)\n", PWM_PIVOT_MIN);
  }
  else if (strncmp(cmd, "sensx=", 6) == 0) {
    float v = (float)atof(cmd + 6);
    SENSOR_OFFSET_MM = constrain(v, 40.0f, 300.0f);
    Serial.printf("sensx=%.1f mm (go 'save' de luu)\n", SENSOR_OFFSET_MM);
  }
  else if (strncmp(cmd, "scale=", 6) == 0) {
    float v = (float)atof(cmd + 6);
    MAP_SCALE = constrain(v, 0.50f, 1.20f);
    computeBackwardBrakeProfile();
    Serial.printf("scale=%.2f (go 'save' de luu)\n", MAP_SCALE);
  }
  else if (strncmp(cmd, "coast=", 6) == 0) {
    float v = (float)atof(cmd + 6);
    CHAIN_COAST_MM = constrain(v, 0.0f, 40.0f);
    Serial.printf("coast=%.1f mm (go 'save' de luu)\n", CHAIN_COAST_MM);
  }
  else if (strncmp(cmd, "sstop=", 6) == 0) {
    float v = (float)atof(cmd + 6);
    S_STOP_FINISH_MM = constrain(v, 50.0f, 400.0f);
    Serial.printf("sstop=%.0f mm (go 'save' de luu)\n", S_STOP_FINISH_MM);
  }
  else {
    Serial.println("CLI: go | stop | calib | dump | status | save | resetnvs | enc | ir | mode1..3 | route=A/B");
    Serial.println("     kp= kd= kff= | ffsign=1/-1 | maxv= | boost= | exp= | piv= | sensx= | scale= | coast= | sstop=");
  }
}

void pollSerialNonBlocking() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (cliLen > 0) {
        cliBuf[cliLen] = '\0';
        executeCLICommand(cliBuf);
        cliLen = 0;
      }
    } else if (cliLen < sizeof(cliBuf) - 1) {
      cliBuf[cliLen++] = c;
    }
  }
}

// ================= 15. XỬ LÝ NÚT NHẤN =================
void handleButtons() {
  if (isRunning) {
    unsigned long nowUs = micros();
    if (nowUs - lastBtnSampleUs >= 5000UL) {
      lastBtnSampleUs = nowUs;
      if (digitalRead(BTN_START_CALIB) == LOW) {
        if (++btnStopConfirmCnt >= 3) {
          unsigned long elapsedMs = millis() - runStartMs;
          stopRobot();
          Serial.printf(">>> EMERGENCY STOP (s=%.1f mm | t=%.2f s)\n",
                        totalDistMM, elapsedMs * 0.001f);
          while (digitalRead(BTN_START_CALIB) == LOW) delay(10);
        }
      } else {
        btnStopConfirmCnt = 0;
      }
    }
    return;
  }

  if (digitalRead(BTN_START_CALIB) == LOW) {
    delay(25);
    if (digitalRead(BTN_START_CALIB) == LOW) {
      unsigned long tPress = millis();
      bool triggeredCalib = false;
      while (digitalRead(BTN_START_CALIB) == LOW) {
        if (millis() - tPress >= 1000) {
          triggeredCalib = true;
          calibrateSensors();
          while (digitalRead(BTN_START_CALIB) == LOW) delay(10);
          break;
        }
        delay(10);
      }
      if (!triggeredCalib) {
        delay(300);
        startRun();
      }
    }
  }

  if (!isRunning && digitalRead(BTN_MODE) == LOW) {
    delay(25);
    if (digitalRead(BTN_MODE) == LOW) {
      applyProfile(currentMode + 1, true);
      while (digitalRead(BTN_MODE) == LOW) delay(10);
    }
  }
}

// ================= 16. SETUP & LOOP 200 HZ =================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  for (int i = 0; i < 8; i++) pinMode(LINE_PINS[i], INPUT);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BTN_START_CALIB, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  if (ENABLE_E18_SENSOR) pinMode(E18_PIN, INPUT_PULLUP);

  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  setMotorRaw(0, 0);

  if (ENABLE_ENCODERS) {
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    encStateL = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
    encStateR = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrEncL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrEncR, CHANGE);
  }

  Serial.println("\n=== HRC 2026 ESP32 READY V0.9 (5kHz PWM, NVS) ===");
  loadConfigFromNVS();
  applyProfile(currentMode, false);
}

void loop() {
  pollSerialNonBlocking();
  handleButtons();

  if (!isRunning) {
    if ((debugSensorMode || debugEncoderMode) && (millis() - lastDebugMs >= 200)) {
      lastDebugMs = millis();
      if (debugSensorMode) {
        SensorResult dbg = readLineSensors(Z_LINE, 0.0f);
        Serial.printf("IR(%luus): ", (unsigned long)dbg.readUs);
        for (int i = 0; i < 8; i++) Serial.printf("%4d ", analogRead(LINE_PINS[i]));
        Serial.printf("| err=%+.2f act=%u\n", dbg.error, dbg.activeCount);
      } else if (debugEncoderMode) {
        int32_t curL, curR;
        readEncoders(curL, curR);
        float distL = curL * MM_PER_TICK;
        float distR = curR * MM_PER_TICK;
        float dDeg  = ((distR - distL) / WHEEL_TRACK_MM) * RAD_TO_DEG_F;
        Serial.printf("ENC L: %6ld (%6.1f mm) | ENC R: %6ld (%6.1f mm) | Heading: %+6.1f deg\n",
                      (long)curL, distL, (long)curR, distR, dDeg);
      }
    }
    return;
  }

  unsigned long nowUs = micros();
  if (nowUs - lastLoopUs < LOOP_INTERVAL_US) return;
  float dt = constrain((nowUs - lastLoopUs) * 1e-6f, 0.001f, 0.020f);
  lastLoopUs = nowUs;

  // 1. Odometry
  int32_t curL = 0, curR = 0;
  if (ENABLE_ENCODERS) readEncoders(curL, curR);
  float dsL = (curL - prevTickL) * MM_PER_TICK;
  float dsR = (curR - prevTickR) * MM_PER_TICK;
  prevTickL = curL; prevTickR = curR;

  float dsMM = 0.5f * (dsL + dsR);
  float dThetaDeg = ((dsR - dsL) / WHEEL_TRACK_MM) * RAD_TO_DEG_F;
  if (turnStage != 2 && turnStage != 25) {   // không cộng quãng đường khi xoay tại chỗ
    totalDistMM = max(0.0f, totalDistMM + dsMM);
  }
  headingDeg += dThetaDeg;

  float rawVL = dsL / dt;
  float rawVR = dsR / dt;
  velL_filt = VEL_EMA_ALPHA * rawVL + (1.0f - VEL_EMA_ALPHA) * velL_filt;
  velR_filt = VEL_EMA_ALPHA * rawVR + (1.0f - VEL_EMA_ALPHA) * velR_filt;
  float vCenter = 0.5f * (velL_filt + velR_filt);
  postTurnT = min(1.0f, postTurnT + dt);

  // 2. Speed Planner & Đọc cảm biến IR
  float vRefMMS = 900.0f;
  float mapKappa = 0.0f;
  ZoneType zType = Z_LINE;
  if (ENABLE_SPEED_PLANNER) {
    queryPlan(totalDistMM, vRefMMS, mapKappa, zType);
  }

  float ms = MAP_SCALE;
  float rShift = (USE_HONEYCOMB_ROUTE_A ? 0.0f : 231.0f) * ms;
  float sSensPre = totalDistMM + SENSOR_OFFSET_MM;
  uint8_t idxBump1 = USE_HONEYCOMB_ROUTE_A ? 12 : 8;
  uint8_t idxBump2 = USE_HONEYCOMB_ROUTE_A ? 14 : 10;
  if (ENABLE_SPEED_PLANNER &&
      ((jncIdx == idxBump1 && sSensPre >= (11520.0f * ms + rShift)) ||
       (jncIdx == idxBump2 && sSensPre >= (12680.0f * ms + rShift)))) {
    zType = Z_BUMP;
    vRefMMS = min(vRefMMS, 820.0f * speedScale);
    if ((jncIdx == idxBump1 && sSensPre >= (12100.0f * ms + rShift)) ||
        (jncIdx == idxBump2 && sSensPre >= (13100.0f * ms + rShift))) {
      vRefMMS = min(vRefMMS, 760.0f * speedScale);
    }
  }
  if (missedRecovery) vRefMMS = min(vRefMMS, 780.0f);
  if (ENABLE_E18_SENSOR && zType == Z_SHORTCUT && forcedDir == 0 && digitalRead(E18_PIN) == LOW) {
    vRefMMS = max(vRefMMS, 1250.0f * speedScale);
  }
  float vAccelCap = vCmdPrev + A_ACCEL_MMSS * dt;
  if (vRefMMS > vAccelCap) vRefMMS = vAccelCap;
  vCmdPrev = vRefMMS;

  SensorResult sr = readLineSensors(zType, fabsf(dsMM));

  // 3. Phục hồi khi bị kẹt tông Vật cản thưởng
  if (reverseUntilMs > 0) {
    if (millis() < reverseUntilMs) {
      setMotorRaw(-130, -130);
      maybePushLog(millis() - runStartMs, totalDistMM, sr, 0.0f, velL_filt, velR_filt,
                   headingDeg, -130, -130, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
      return;
    } else {
      reverseUntilMs = 0;
      vCmdPrev = 0.0f;
      resetWheelPI();
    }
  }
  if (ENABLE_ENCODERS && ENABLE_STALL_RECOVERY && zType == Z_SHORTCUT && forcedDir == 0) {
    if (vRefMMS > 500.0f && fabsf(vCenter) < 0.20f * vRefMMS) {
      if (stallStartMs == 0) stallStartMs = millis();
      else if (millis() - stallStartMs > 300) {
        stallStartMs = 0;
        if (++stallRetries <= 2) {
          reverseUntilMs = millis() + 240;
          resetWheelPI();
          return;
        } else {
          stopRobot();
          Serial.println(">>> STALL OBSTACLE! STOPPED SAFELY.");
          return;
        }
      }
    } else {
      stallStartMs = 0;
    }
  } else {
    stallStartMs = 0;
  }

  // 4. Dừng Đích
  if (ENABLE_ENCODERS && ENABLE_FINISH_STOP && zType == Z_FINISH) {
    float sFinSens = totalDistMM + SENSOR_OFFSET_MM;
    if (!finishBraking && !invertedMode &&
        (sr.tJunction || sr.activeCount >= 5 || (sFinSens >= (15380.0f * ms + rShift)))) {
      finishBraking = true;
      finishEntryMM = totalDistMM;
    }
    if (finishBraking) {
      // Ô đích 300mm * MAP_SCALE (ở 85% là 255mm -> dừng tối đa ~220mm cho an toàn)
      float effStopMM = min(S_STOP_FINISH_MM, max(185.0f, 300.0f * ms - 35.0f));
      float sInBox = totalDistMM - finishEntryMM;
      float sRem   = effStopMM - sInBox;
      if (sRem <= 8.0f) {
        maybePushLog(millis() - runStartMs, totalDistMM, sr, 0.0f, velL_filt, velR_filt,
                     headingDeg, 0, 0, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
        stopRobot();
        Serial.printf(">>> FINISH STOP! s=%.1f mm | t=%.2f s\n",
                      totalDistMM, (millis() - runStartMs) * 0.001f);
        return;
      }
      vRefMMS  = min(vRefMMS, sqrtf(2.0f * A_BRAKE_MMSS * sRem));
      vCmdPrev = vRefMMS;
    }
  }

  // 4b. Khóa đi thẳng qua ngã tư p_split (giữ hướng bằng encoder để không bị kéo sang nhánh chéo)
  if (ENABLE_ENCODERS && totalDistMM < straightLockUntilMM) {
    int baseSt = constrain((int)((vRefMMS / MAX_SPEED_MMS) * 255.0f), PWM_PIVOT_MIN, 185);
    int holdSt = (int)(HEAD_HOLD_KP * (headingDeg - straightLockHead));
    int stL = constrain(baseSt + holdSt, -maxSpeed, maxSpeed);
    int stR = constrain(baseSt - holdSt, -maxSpeed, maxSpeed);
    setMotorRaw(stL, stR);
    maybePushLog(millis() - runStartMs, totalDistMM, sr, vRefMMS, velL_filt, velR_filt,
                 headingDeg, stL, stR, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
    return;
  }

  // 4c. Phanh chủ động tắt đà sau mỗi lượt xoay (45ms) trước khi sang bước nối chuỗi / bám line
  if (turnBrakeT > 0.0f) {
    turnBrakeT -= dt;
    setMotorRaw(0, 0);
    lastPwmL = 0; lastPwmR = 0;
    maybePushLog(millis() - runStartMs, totalDistMM, sr, 0.0f, velL_filt, velR_filt,
                 headingDeg, 0, 0, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
    return;
  }

  // 5. Nút giao (chỉ chạy khi bật ENABLE_ENCODERS & ENABLE_SPEED_PLANNER)
  float sSensor = totalDistMM + SENSOR_OFFSET_MM;
  if (ENABLE_ENCODERS && ENABLE_SPEED_PLANNER) {
    const TurnEvent* activePlan = USE_HONEYCOMB_ROUTE_A ? PLAN_A : PLAN_B;
    uint8_t activePlanLen       = USE_HONEYCOMB_ROUTE_A ? LEN_PLAN_A : LEN_PLAN_B;

    if (forcedDir == 0 && jncIdx < activePlanLen) {
      const TurnEvent &ev = activePlan[jncIdx];
      float sExp = ev.sExpect * ms;
      float sTol = max(55.0f, ev.tol * ms);

      if (ev.chainAdvMM > 0.0f) {
        // Không kích chuỗi rẽ mù nếu nút cha trước đó đã bị lỡ hoặc xe đã vượt xa vị trí dự kiến
        if (missedRecovery || sSensor > sExp + sTol + 250.0f) {
          jncIdx++;
          jncConfirmCnt = 0;
        } else {
          bool isLastInChain = (jncIdx + 1 >= activePlanLen) || (activePlan[jncIdx + 1].chainAdvMM <= 0.0f);
          forcedDir        = ev.dir;
          forcedDeg        = ev.deg;
          forcedAdvMM      = max(20.0f, ev.chainAdvMM * ms - CHAIN_COAST_MM);
          forcedPureEnc    = ((ev.flags & TF_PURE_ENC) != 0);
          forcedUseAbs     = ((ev.flags & TF_ABS_HEAD) != 0);
          advHoldHead      = forcedUseAbs ? forcedTargetHead : headingDeg;
          forcedTargetHead = absRefHead + ev.absDeg;
          forcedChainExitS = isLastInChain ? sExp : -1.0f;
          turnStage        = 1;
          turnStageT       = 0.0f;
          turnStartMM      = totalDistMM;
          turnStartHead    = headingDeg;
          jncIdx++;
        }
      } else if (sSensor > sExp + sTol) {
        appendRunTrace("M%u@%.0f ", jncIdx, totalDistMM);
        jncIdx++;
        missedRecovery = true;
        jncConfirmCnt = 0;
      } else {
        bool zoneOk = (zType == ev.zone)
                   || (jncIdx == 0 && zType == Z_LINE)
                   || (USE_HONEYCOMB_ROUTE_A && (
                         (jncIdx == 10 && zType == Z_CURVED_GAP) ||
                         (jncIdx == 12 && zType == Z_HUST) ||
                         (jncIdx == 13 && zType == Z_BUMP) ||
                         (jncIdx == 14 && zType == Z_HUST)))
                   || (!USE_HONEYCOMB_ROUTE_A && (
                         (jncIdx == 6 && zType == Z_CURVED_GAP) ||
                         (jncIdx == 8 && zType == Z_HUST) ||
                         (jncIdx == 9 && zType == Z_BUMP) ||
                         (jncIdx == 10 && zType == Z_HUST)));

        if (fabsf(sSensor - sExp) <= sTol && !invertedMode && zoneOk) {
          bool trig = false;
          uint8_t idxPsplitRet = USE_HONEYCOMB_ROUTE_A ? 10 : 6;
          uint8_t idxCorner100 = USE_HONEYCOMB_ROUTE_A ? 11 : 7;
          uint8_t idxEndS      = USE_HONEYCOMB_ROUTE_A ? 12 : 8;
          uint8_t idxSU        = USE_HONEYCOMB_ROUTE_A ? 13 : 9;
          uint8_t idxHIn       = USE_HONEYCOMB_ROUTE_A ? 14 : 10;

          if (ev.zone == Z_HONEYCOMB) trig = sr.tJunction || sr.leftBranch || sr.rightBranch;
          else if (jncIdx == idxPsplitRet) trig = sr.leftEdge && (sr.tJunction || sr.leftBranch || sr.ySplit || sr.activeCount >= 3);
          else if (ev.dir == 0) trig = sr.leftBranch || sr.rightBranch || sr.tJunction || sr.ySplit || (sr.activeCount >= 3);
          else if (jncIdx == idxCorner100) trig = sr.leftBranch || sr.tJunction || (sr.leftEdge && sr.activeCount >= 2) || (!sr.hasLine && sSensor >= (10080.0f * ms + rShift));
          else if (jncIdx == idxEndS) trig = (sSensor >= (12000.0f * ms + rShift)) && (sr.leftBranch || sr.tJunction);
          else if (jncIdx == idxSU) trig = (sSensor >= (12580.0f * ms + rShift)) && (sr.rightBranch || sr.tJunction || !sr.hasLine || (sr.rightEdge && sr.activeCount >= 3));
          else if (jncIdx == idxHIn) trig = (!sr.hasLine) && (sSensor >= (13080.0f * ms + rShift));
          else {
            bool dirBr = (ev.dir < 0) ? sr.leftBranch : sr.rightBranch;
            trig = dirBr || sr.tJunction || (!sr.hasLine && sSensor >= sExp - 45.0f * ms);
          }

          if (trig) {
            jncConfirmCnt++;
            uint8_t idxHBridgeEnd = USE_HONEYCOMB_ROUTE_A ? 17 : 13;
            bool oneSampleOk = sr.tJunction || jncIdx == 0 || ev.dir == 0 ||
                               (jncIdx >= idxPsplitRet && jncIdx <= idxHBridgeEnd && jncIdx != idxHIn);
            if (jncConfirmCnt >= (oneSampleOk ? 1 : 2)) {
              jncConfirmCnt = 0;
              uint8_t curIdx = jncIdx;
              float anchorS = sExp;
              // idxPsplitRet neo về 9250*ms để 803mm (ở 85%) tới đúng góc nhọn 100° idxCorner100 (10194*ms)
              if (curIdx == idxPsplitRet) anchorS = 9250.0f * ms + rShift;
              else if (curIdx == idxEndS) anchorS = 12120.0f * ms + rShift;
              else if (curIdx == idxHIn)  anchorS = 13120.0f * ms + rShift;
              appendRunTrace("J%u@%.0f(h%.0f) ", curIdx, totalDistMM, headingDeg);
              totalDistMM = max(0.0f, anchorS - SENSOR_OFFSET_MM);
              missedRecovery = false;
              jncIdx++;
              if (ev.dir == 0) {
                if (curIdx != idxPsplitRet) {
                  // Cụm giao kép 2 đường chéo sau Tổ ong (p_split): khóa thẳng 155*ms theo hướng ngang chuẩn
                  straightLockUntilMM = totalDistMM + 155.0f * ms;
                  straightLockHead    = hasAbsRef ? absRefHead : headingDeg;
                }
                // Với idxPsplitRet: không khóa thẳng và không xoay dừng -> để tracking gate bestCl=0 tự rẽ trái êm vào lối tắt
              } else {
                if ((ev.flags & TF_ABS_START) != 0) {
                  absRefHead = headingDeg;
                  hasAbsRef  = true;
                }
                forcedDir        = ev.dir;
                forcedDeg        = ev.deg;
                forcedPureEnc    = ((ev.flags & TF_PURE_ENC) != 0);
                forcedUseAbs     = ((ev.flags & TF_ABS_HEAD) != 0);
                forcedTargetHead = absRefHead + ev.absDeg;
                advHoldHead      = headingDeg;
                forcedChainExitS = -1.0f;
                // Quy đổi quãng tiến trục bánh tới tâm nút giao theo SENSOR_OFFSET_MM (trừ ~20mm quãng phanh stage 15)
                forcedAdvMM = max(75.0f, SENSOR_OFFSET_MM - 20.0f);
                turnStage     = 1;
                turnStageT    = 0.0f;
                turnStartMM   = totalDistMM;
                turnStartHead = headingDeg;
              }
            }
          } else {
            jncConfirmCnt = 0;
          }
        }
      }
    }
  }

  // 5b. THỰC THI RẼ (1 tiến tới tâm nút -> 15 phanh tắt đà tiến -> 2 xoay tại chỗ -> 25 xoay chậm tìm line)
  if (turnStage != 0) {
    turnStageT += dt;
    int cmdL = 0, cmdR = 0;
    bool done = false;

    if (turnStage == 1) {
      float adv = totalDistMM - turnStartMM;
      if (adv >= forcedAdvMM || turnStageT > 1.2f) {
        turnStage  = 15;
        turnStageT = 0.0f;
      } else {
        int maxAdvPWM = forcedUseAbs ? 100 : 120;
        int baseAdv = constrain((int)((vRefMMS / MAX_SPEED_MMS) * 255.0f), max(PWM_PIVOT_MIN, 55), maxAdvPWM);
        int hold = (int)(HEAD_HOLD_KP * (headingDeg - advHoldHead));  // trôi trái -> bù sang phải
        cmdL = baseAdv + hold;
        cmdR = baseAdv - hold;
      }
    }
    if (turnStage == 15) {
      // Phanh chủ động tắt đà tiến trước khi xoay tại chỗ (chống văng trục bánh tiến thêm ~60mm lọt vào lòng chữ U!)
      if ((turnStageT >= 0.030f && fabsf(vCenter) < 45.0f) || turnStageT >= 0.085f) {
        turnStage     = 2;
        turnStageT    = 0.0f;
        turnStartHead = headingDeg;
      } else {
        cmdL = 0;
        cmdR = 0;
      }
    }
    if (turnStage == 2) {
      // rotated > 0 khi xe đã quay đúng chiều rẽ (rẽ phải: heading giảm; rẽ trái: heading tăng)
      float rotated = (float)forcedDir * (turnStartHead - headingDeg);
      float remaining = forcedUseAbs
                        ? ((float)forcedDir * (headingDeg - forcedTargetHead))
                        : (forcedDeg - rotated);
      // Yêu cầu quay ít nhất 82% góc đặt trước khi cho lineCaught ngắt sớm (tránh bắt nhầm tường trong chữ U / cửa Tổ ong)
      bool lineCaught = (!forcedPureEnc) && (rotated >= 0.82f * forcedDeg) && sr.hasLine &&
                        sr.activeCount >= 1 && sr.activeCount <= 3 && fabsf(sr.error) < 1.0f;
      if (lineCaught) {
        done = true;
      } else if (forcedPureEnc && (remaining <= 4.0f || turnStageT > 1.6f)) {
        done = true;
      } else if (!forcedPureEnc && (remaining <= 0.0f || turnStageT > 1.6f)) {
        turnStage  = 25;
        turnStageT = 0.0f;
      } else {
        int pv = constrain(PWM_PIVOT_MIN + (int)(0.9f * max(0.0f, remaining)), PWM_PIVOT_MIN, 150);
        cmdL =  forcedDir * pv;   // dir=+1 (phải): bánh trái tiến, bánh phải lùi
        cmdR = -forcedDir * pv;
      }
    }
    if (turnStage == 25 && !done) {
      bool caught = sr.hasLine && sr.activeCount >= 1 && sr.activeCount <= 3 && fabsf(sr.error) < 2.2f;
      if (caught || turnStageT > 1.05f) {
        done = true;
      } else {
        // 0.35s đầu xoay tiếp cùng chiều (phòng quay thiếu), sau đó đảo chiều quét ngược (phòng quay lố)
        int8_t sweepDir = (turnStageT < 0.35f) ? forcedDir : (int8_t)(-forcedDir);
        cmdL =  sweepDir * PWM_PIVOT_MIN;
        cmdR = -sweepDir * PWM_PIVOT_MIN;
      }
    }

    if (done) {
      int8_t prevDir = forcedDir;
      if (forcedChainExitS >= 0.0f) {
        totalDistMM = forcedChainExitS;
        forcedChainExitS = -1.0f;
        appendRunTrace("X%u@%.0f(h%.0f) ", jncIdx, totalDistMM, headingDeg);
      }
      forcedDir     = 0;
      forcedPureEnc = false;
      forcedUseAbs  = false;
      turnStage     = 0;
      turnStageT    = 0.0f;
      turnBrakeT    = 0.045f;       // phanh chủ động 45ms để tắt hẳn đà xoay
      postTurnT     = 0.0f;
      // Nếu thoát rẽ mà chưa thấy line, giữ dấu lệch mạnh để bộ mất line KHÔNG hiểu nhầm là đoạn đứt thẳng
      lastError    = sr.hasLine ? sr.error : ((float)prevDir * 3.5f);
      dFiltered    = 0.0f;
      wasLostLine  = true;          // để nhánh bám line reset thành phần D
      lostInit     = false;
      lostSearchStartMs = 0;
      lostStartMs  = millis();
      lostStartMM  = totalDistMM;
      lastSharpDir = sr.hasLine ? 0 : prevDir;
      straightMM   = 0.0f;
      vCmdPrev     = 0.0f;
      resetWheelPI();
      setMotorRaw(0, 0);
      lastPwmL = 0; lastPwmR = 0;
      maybePushLog(millis() - runStartMs, totalDistMM, sr, vRefMMS, velL_filt, velR_filt,
                   headingDeg, 0, 0, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
      return;
    } else {
      setMotorRaw(cmdL, cmdR);
      lastPwmL = cmdL; lastPwmR = cmdR;
      maybePushLog(millis() - runStartMs, totalDistMM, sr, vRefMMS, velL_filt, velR_filt,
                   headingDeg, cmdL, cmdR, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
      return;
    }
  }

  // 6. BÁM LINE
  int pwmL = 0, pwmR = 0;
  if (sr.hasLine) {
    lostStartMM = totalDistMM;
    lostStartMs = millis();
    lostInit = false;
    lostSearchStartMs = 0;

    if (fabsf(sr.error) >= 0.85f) lastSharpDir = (sr.error > 0.0f) ? 1 : -1;
    else if (fabsf(sr.error) < 0.30f) lastSharpDir = 0;

    if (fabsf(sr.error) < 0.60f && !invertedMode) straightMM += fabsf(dsMM);
    else straightMM = 0.0f;

    float rawD = 0.0f;
    if (wasLostLine) {
      lastError = sr.error;
      dFiltered = 0.0f;
      wasLostLine = false;
    } else {
      rawD = constrain((sr.error - lastError) * (0.005f / dt), -4.0f, 4.0f);
      lastError = sr.error;
    }
    dFiltered = ENABLE_CURVE_OPT ? (D_FILTER_ALPHA * rawD + (1.0f - D_FILTER_ALPHA) * dFiltered) : rawD;

    // Không kẹp sàn effBase bằng minDynBase để Speed Planner và finishBraking giảm tốc chính xác
    int floorPWM = finishBraking ? 0 : PWM_PIVOT_MIN;
    int effBase = ENABLE_SPEED_PLANNER
                  ? constrain((int)((vRefMMS / MAX_SPEED_MMS) * 255.0f), floorPWM, maxSpeed)
                  : constrain((int)(180 * speedScale), floorPWM, maxSpeed);
    // Ngay sau khi rẽ: đi chậm 0.3s cho xe dài ổn định lại
    if (postTurnT < 0.30f) effBase = min(effBase, 130);

    float activeKp = (straightMM > 150.0f && fabsf(mapKappa) < 0.1f) ? (kp * STRAIGHT_KP_SC) : kp;
    float activeKd = kd;

    float uFF = 0.0f;
    if (ENABLE_SPEED_PLANNER && ENABLE_FEEDFORWARD && fabsf(mapKappa) > 0.05f) {
      uFF = FF_SIGN * kff * (mapKappa * (WHEEL_TRACK_MM * 0.0005f)) * ((float)effBase / 180.0f);
    }

    float corr = 0.0f;
    if (sr.invTransition) {
      dFiltered = 0.0f;
      corr = 0.0f;
    } else {
      corr = activeKp * sr.error + activeKd * dFiltered + uFF;
      // EDGE BOOST: line sắp ra khỏi dải cảm biến -> lái mạnh hơn (bánh trong có thể quay ngược)
      float eAbs = fabsf(sr.error);
      if (eAbs > 2.0f) {
        corr *= 1.0f + EDGE_BOOST_MAX * min(1.0f, (eAbs - 2.0f) / 1.5f);
      }
    }

    float ratio = sr.invTransition ? 0.0f : min(1.0f, fabsf(sr.error) / 3.5f);
    int dynBase = effBase;
    if (!finishBraking) {
      int localMin = (zType == Z_GEAR) ? min(52, effBase) : min(minDynBase, effBase);
      if (localMin == effBase) localMin = min(effBase, max(floorPWM, 52));
      dynBase = effBase - (int)(powf(ratio, CURVE_EXP) * max(0, effBase - localMin));
      dynBase = max(dynBase, localMin);
    }

    int rawL = dynBase + (int)corr;
    int rawR = dynBase - (int)corr;
    if (ENABLE_CURVE_OPT) {
      if (rawL > maxSpeed) { rawR -= (rawL - maxSpeed); rawL = maxSpeed; }
      else if (rawR > maxSpeed) { rawL -= (rawR - maxSpeed); rawR = maxSpeed; }
    }
    pwmL = constrain(rawL, -maxSpeed, maxSpeed);
    pwmR = constrain(rawR, -maxSpeed, maxSpeed);

    if (ENABLE_ENCODERS && USE_ENCODER_PI) {
      float tgtVL = (pwmL / 255.0f) * MAX_SPEED_MMS;
      float tgtVR = (pwmR / 255.0f) * MAX_SPEED_MMS;
      float errVL = tgtVL - velL_filt;
      float errVR = tgtVR - velR_filt;

      float outL_pre = pwmL + velKp * errVL + velKi * velIntegralL;
      if (!((outL_pre >= maxSpeed && errVL > 0.0f) || (outL_pre <= -maxSpeed && errVL < 0.0f))) {
        velIntegralL = constrain(velIntegralL + errVL * dt, -velIntClamp, velIntClamp);
      }
      float outR_pre = pwmR + velKp * errVR + velKi * velIntegralR;
      if (!((outR_pre >= maxSpeed && errVR > 0.0f) || (outR_pre <= -maxSpeed && errVR < 0.0f))) {
        velIntegralR = constrain(velIntegralR + errVR * dt, -velIntClamp, velIntClamp);
      }

      pwmL = constrain(pwmL + (int)(velKp * errVL + velKi * velIntegralL), -maxSpeed, maxSpeed);
      pwmR = constrain(pwmR + (int)(velKp * errVR + velKi * velIntegralR), -maxSpeed, maxSpeed);
    }
    lastPwmL = pwmL; lastPwmR = pwmR;
    setMotorRaw(pwmL, pwmR);
  } else {
    // ===== MẤT LINE (tối ưu cho xe dài, cảm biến xa trục bánh) =====
    wasLostLine = true;
    straightMM = 0.0f;
    resetWheelPI();

    if (!lostInit) {
      lostInit    = true;
      lostHeadDeg = headingDeg;
      lostErrSnap = lastError;
      // Chiều tìm line = phía line cuối cùng nhìn thấy (error > 0: line bên phải -> xoay phải)
      lostDir = (lastSharpDir != 0) ? lastSharpDir : ((lastError >= 0.0f) ? 1 : -1);
    }

    unsigned long lostMs = millis() - lostStartMs;
    float lostDist = fabsf(totalDistMM - lostStartMM);
    bool gapZone = (zType == Z_CURVED_GAP || zType == Z_SHORTCUT || zType == Z_BUMP || zType == Z_FINISH);
    // Trong Z_CURVED_GAP (cung R574 nét đứt), xe dài lệch line lớn hơn -> nới ngưỡng để không xoay tại chỗ giữa cung đứt
    float centerLim = (zType == Z_CURVED_GAP) ? 2.6f : 1.2f;
    bool wasCentered = fabsf(lostErrSnap) < centerLim;

    unsigned long gapMs = 0;
    float gapMM = 0.0f;
    if (wasCentered) {
      gapMs = gapZone ? 700UL : 180UL;
      gapMM = gapZone ? (450.0f * ms) : 70.0f;
    }
    bool inGap = wasCentered && (lostMs < gapMs) && (lostDist < gapMM);

    if (!wasCentered && lostMs < LOST_HOLD_MS) {
      // Mất thoáng qua ở mép: giữ nguyên lệnh lái cũ
      pwmL = lastPwmL; pwmR = lastPwmR;
      setMotorRaw(pwmL, pwmR);
    } else if (inGap) {
      // Đoạn đứt cong -> feedforward động học đầy đủ + giữ góc cung tròn; phanh đích -> hãm chủ động theo vận tốc thực
      if (finishBraking && vCenter > vRefMMS + 180.0f) {
        pwmL = -25; pwmR = -25;
        setMotorRaw(pwmL, pwmR);
      } else if (finishBraking && vCenter > vRefMMS + 50.0f) {
        pwmL = 0; pwmR = 0;
        setMotorRaw(0, 0);
      } else {
        int baseGMin = finishBraking ? 0 : PWM_PIVOT_MIN;
        int baseG = constrain((int)((vRefMMS / MAX_SPEED_MMS) * 255.0f), baseGMin, 150);
        float c = 0.0f;
        if (gapZone && fabsf(mapKappa) > 0.05f) {
          float cFF = FF_SIGN * (float)baseG * (mapKappa * (WHEEL_TRACK_MM * 0.0005f));
          float expHeadDeg = lostHeadDeg + (-FF_SIGN) * (mapKappa * 0.001f * lostDist) * RAD_TO_DEG_F;
          c = cFF + HEAD_HOLD_KP * (headingDeg - expHeadDeg);
        } else {
          c = HEAD_HOLD_KP * (headingDeg - lostHeadDeg);
        }
        pwmL = constrain(baseG + (int)c, -maxSpeed, maxSpeed);
        pwmR = constrain(baseG - (int)c, -maxSpeed, maxSpeed);
        setMotorRaw(pwmL, pwmR);
      }
    } else {
      // Tìm line: xoay tại chỗ về phía cuối cùng thấy line, không thấy thì quét ngược lại, rồi dừng
      if (lostSearchStartMs == 0) lostSearchStartMs = millis();
      unsigned long tS = millis() - lostSearchStartMs;
      int8_t d = lostDir;
      if (tS >= 450UL && tS < 1300UL) d = -lostDir;
      if (tS >= 1300UL) {
        pwmL = 0; pwmR = 0;
        maybePushLog(millis() - runStartMs, totalDistMM, sr, vRefMMS, velL_filt, velR_filt,
                     headingDeg, 0, 0, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
        stopRobot();
        Serial.println(">>> LOST LINE! STOPPED.");
        return;
      }
      int sp = constrain(PWM_PIVOT_MIN + 35, PWM_PIVOT_MIN, 110);
      pwmL =  d * sp;     // d=+1 (phải): bánh trái tiến, bánh phải lùi
      pwmR = -d * sp;
      setMotorRaw(pwmL, pwmR);
    }
  }
  maybePushLog(millis() - runStartMs, totalDistMM, sr, vRefMMS, velL_filt, velR_filt,
               headingDeg, pwmL, pwmR, (uint8_t)zType, jncIdx, invertedMode, turnStage, finishBraking);
}