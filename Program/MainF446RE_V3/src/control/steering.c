#include "steering.h"

#include <stdio.h>

#include "flash.h"
#include "mymath.h"
#include "timer.h"

#define STEERING_CALIB_MAGIC 0x53544545u  // "STEE"
// 駆動電源の投入直後に呼ばれるため、MDのブートから状態フレーム送信開始までを待てる長さにしてある
#define STEERING_CALIB_TIMEOUT_S 5.0f
// 中心点として記録する角度を、この数の連続した状態フレームの平均から求める (MD は 500us ごとに
// 返すので約 25ms ぶん)。1フレームだけで決めると、同期ずれで CRC-8 をすり抜けたフレーム
// (約 1/256) や、手を離した直後の揺れをそのまま中心点として Flash に書いてしまう
#define STEERING_CALIB_FRAMES 50
// 上の区間で、最初のフレームからこれ以上 [rad] 離れた角度が出たらやり直す
#define STEERING_CALIB_MAX_SPREAD_RAD 0.02f

typedef struct {
  uint32_t magic;
  float center_rad;
} SteeringCalibData;

// リンクの換算 (steering.h)。x = モータ角 × STEERING_LINKAGE_RATIO の可動範囲は ±kMaxLinearRad
static const float kMaxLinearRad = STEERING_MAX_ANGLE_RAD * STEERING_LINKAGE_RATIO;
static float link_gain = 1.0f;
static float link_cubic = 0.0f;

static float LinearToRoad(float x_rad) {
  return link_gain * x_rad + link_cubic * x_rad * x_rad * x_rad;
}

// 路面舵角 → x。可動範囲で単調 (Steering_SetLinkage が保証) なのでニュートン法で解ける。
// 初期値 x = 路面舵角 から 4回で float の精度に収まる
static float RoadToLinear(float road_rad) {
  float x = road_rad;
  for (int i = 0; i < 4; i++) {
    float slope = link_gain + 3.0f * link_cubic * x * x;
    if (slope < STEERING_LINKAGE_MIN_SLOPE) slope = STEERING_LINKAGE_MIN_SLOPE;
    x -= (LinearToRoad(x) - road_rad) / slope;
    x = Constrain(x, -kMaxLinearRad, kMaxLinearRad);
  }
  return x;
}

void Steering_SetLinkage(float gain, float cubic) {
  // 傾き gain + 3*cubic*x^2 が可動範囲の端でも下限を割らないようにする
  float cubic_min = (STEERING_LINKAGE_MIN_SLOPE - gain) / (3.0f * kMaxLinearRad * kMaxLinearRad);
  link_gain = gain;
  link_cubic = cubic < cubic_min ? cubic_min : cubic;
}

static void LoadFromFlash(Steering* obj) {
  SteeringCalibData calib;
  Flash_ReadData(FLASH_USER_START_ADDR, &calib, sizeof(calib));
  // 値そのものも確かめる (機械角は 0〜2π。書き込みが途中で切れた・化けた値を中心点にしない)
  if (calib.magic == STEERING_CALIB_MAGIC && calib.center_rad >= 0.0f && calib.center_rad < TWO_PI + 0.01f) {
    obj->center_rad = calib.center_rad;
    obj->center_valid = true;
    printf("Steering: loaded center_rad=%.3f\n", (double)obj->center_rad);
  } else {
    obj->center_rad = 0.0f;
    obj->center_valid = false;
    printf("Steering: no calibration data found, using center_rad=0\n");
  }
}

// MDから状態フレームを実際に受信できるまで待ってから中心点を記録する。MD起動直後は送信開始まで
// 時間がかかる場合があるため、固定時間ではなく最初の1フレームが届くまで待つ。タイムアウトまでに
// 一度も受信できなければ配線・ボーレート不一致等で受信できていないと判断し、trueを返さない。
static bool Calibrate(Steering* obj) {
  uint32_t rx_count = BldcMotor_GetRxCount(obj->motor);

  Timer timer;
  Timer_Init(&timer);
  // 0/2π の境界をまたいでも平均が半周ずれないよう、最初のフレームからの差分を平均する
  float first_rad = 0.0f;
  float diff_sum_rad = 0.0f;
  int frames = 0;
  while (frames < STEERING_CALIB_FRAMES) {
    BldcMotor_Update(obj->motor);
    if (Timer_Read(&timer) > STEERING_CALIB_TIMEOUT_S) {
      printf("Steering: calibration failed (%s)\n",
             frames == 0 ? "no data received from motor" : "angle not steady");
      return false;
    }
    if (BldcMotor_GetRxCount(obj->motor) == rx_count) continue;
    rx_count = BldcMotor_GetRxCount(obj->motor);

    float angle_rad = BldcMotor_GetMechAngle(obj->motor);
    if (frames == 0) first_rad = angle_rad;
    float diff_rad = GapRadians(angle_rad, first_rad);
    if (Abs(diff_rad) > STEERING_CALIB_MAX_SPREAD_RAD) {
      frames = 0;
      diff_sum_rad = 0.0f;
      continue;
    }
    diff_sum_rad += diff_rad;
    frames++;
  }

  obj->center_rad = NormalizeRadians(first_rad + diff_sum_rad / (float)STEERING_CALIB_FRAMES);

  SteeringCalibData calib = {STEERING_CALIB_MAGIC, obj->center_rad};
  // 保存に失敗しても今回の起動では測った値を使う (次の起動では前の値に戻る)
  if (Flash_WriteData(FLASH_USER_START_ADDR, &calib, sizeof(calib)) != HAL_OK) {
    printf("Steering: failed to save calibration to flash\n");
  }
  printf("Steering calibrated: center_rad=%.3f\n", (double)obj->center_rad);
  return true;
}

void Steering_Init(Steering* obj, BldcMotor* motor, bool do_calibrate) {
  obj->motor = motor;
  obj->center_valid = false;

  BldcMotor_SetTorqueLimitNm(motor, STEERING_MAX_TORQUE_NM);

  if (do_calibrate && Calibrate(obj)) {
    obj->center_valid = true;
    return;
  }

  LoadFromFlash(obj);
}

void Steering_SetAngleRad(Steering* obj, float angle_rad) {
  // 中心点が無い (未較正) 間は何も指令しない。未較正時の center_rad は 0 で、これを基準に
  // 絶対角を送ると、駆動電源が入った瞬間にステアが無関係な機械角へ動く
  if (!obj->center_valid) return;
  angle_rad = Constrain(angle_rad, -STEERING_MAX_ANGLE_RAD, STEERING_MAX_ANGLE_RAD);
  float target_rad = NormalizeRadians(obj->center_rad + STEERING_DIRECTION_SIGN * angle_rad);
  BldcMotor_SetPosition(obj->motor, target_rad);
}

float Steering_GetAngleRad(const Steering* obj) {
  // 給電が切れている間はMDの機械角がリセットされて0固定になり、中心点との差分をそのまま
  // 取ると「中心点ぶんズレた角度」という誤った値になる。有効な受信データが無い間は0を返す
  if (!BldcMotor_IsDataValid(obj->motor)) return 0.0f;
  return STEERING_DIRECTION_SIGN * GapRadians(BldcMotor_GetMechAngle(obj->motor), obj->center_rad);
}

void Steering_SetRoadWheelAngleRad(Steering* obj, float angle_rad) {
  float max_road_rad = Steering_GetMaxRoadWheelAngleRad();
  angle_rad = Constrain(angle_rad, -max_road_rad, max_road_rad);
  Steering_SetAngleRad(obj, RoadToLinear(angle_rad) / STEERING_LINKAGE_RATIO);
}

float Steering_GetRoadWheelAngleRad(const Steering* obj) {
  return LinearToRoad(Steering_GetAngleRad(obj) * STEERING_LINKAGE_RATIO);
}

float Steering_GetMaxRoadWheelAngleRad(void) {
  return LinearToRoad(kMaxLinearRad);
}

bool Steering_IsCenterValid(const Steering* obj) {
  return obj->center_valid;
}
