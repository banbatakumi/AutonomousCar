#include "ahrs.h"

#include <math.h>

#define AHRS_DEG_TO_RAD 0.017453292519943295f
#define AHRS_RAD_TO_DEG 57.29577951308232f

// クォータニオン -> ZYX オイラー角 [deg]
static void UpdateEuler(Ahrs *obj) {
  float q0 = obj->q0, q1 = obj->q1, q2 = obj->q2, q3 = obj->q3;

  obj->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                     1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
              AHRS_RAD_TO_DEG;

  float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
  if (sin_pitch > 1.0f) sin_pitch = 1.0f;
  if (sin_pitch < -1.0f) sin_pitch = -1.0f;
  obj->pitch = asinf(sin_pitch) * AHRS_RAD_TO_DEG;

  obj->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
             AHRS_RAD_TO_DEG;
}

void Ahrs_Init(Ahrs *obj, float kp, float ki) {
  obj->q0 = 1.0f;
  obj->q1 = 0.0f;
  obj->q2 = 0.0f;
  obj->q3 = 0.0f;
  obj->integral_fb_x = 0.0f;
  obj->integral_fb_y = 0.0f;
  obj->integral_fb_z = 0.0f;
  Ahrs_SetGains(obj, kp, ki);
  UpdateEuler(obj);
}

void Ahrs_SetGains(Ahrs *obj, float kp, float ki) {
  obj->two_kp = 2.0f * kp;
  obj->two_ki = 2.0f * ki;
}

void Ahrs_SetFromAccel(Ahrs *obj, float ax, float ay, float az) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);

  // yaw = 0 とした ZYX オイラー角 -> クォータニオン
  obj->q0 = cr * cp;
  obj->q1 = sr * cp;
  obj->q2 = cr * sp;
  obj->q3 = -sr * sp;

  obj->integral_fb_x = 0.0f;
  obj->integral_fb_y = 0.0f;
  obj->integral_fb_z = 0.0f;
  UpdateEuler(obj);
}

void Ahrs_Update(Ahrs *obj, float gx_dps, float gy_dps, float gz_dps, float ax,
                 float ay, float az, float dt) {
  if (dt <= 0.0f) return;

  float gx = gx_dps * AHRS_DEG_TO_RAD;
  float gy = gy_dps * AHRS_DEG_TO_RAD;
  float gz = gz_dps * AHRS_DEG_TO_RAD;

  float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
  if (accel_norm > AHRS_ACCEL_GATE_MIN && accel_norm < AHRS_ACCEL_GATE_MAX) {
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;

    // 現在の姿勢推定から求まる重力方向 (の半分)
    float half_vx = obj->q1 * obj->q3 - obj->q0 * obj->q2;
    float half_vy = obj->q0 * obj->q1 + obj->q2 * obj->q3;
    float half_vz = obj->q0 * obj->q0 - 0.5f + obj->q3 * obj->q3;

    // 推定と観測の外積が姿勢誤差 (ロール・ピッチ方向のみに現れる)
    float half_ex = ay * half_vz - az * half_vy;
    float half_ey = az * half_vx - ax * half_vz;
    float half_ez = ax * half_vy - ay * half_vx;

    if (obj->two_ki > 0.0f) {
      obj->integral_fb_x += obj->two_ki * half_ex * dt;
      obj->integral_fb_y += obj->two_ki * half_ey * dt;
      obj->integral_fb_z += obj->two_ki * half_ez * dt;
      gx += obj->integral_fb_x;
      gy += obj->integral_fb_y;
      gz += obj->integral_fb_z;
    } else {
      obj->integral_fb_x = 0.0f;
      obj->integral_fb_y = 0.0f;
      obj->integral_fb_z = 0.0f;
    }

    gx += obj->two_kp * half_ex;
    gy += obj->two_kp * half_ey;
    gz += obj->two_kp * half_ez;
  }

  // クォータニオンの1次積分
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  float q0 = obj->q0, q1 = obj->q1, q2 = obj->q2, q3 = obj->q3;
  obj->q0 += -q1 * gx - q2 * gy - q3 * gz;
  obj->q1 += q0 * gx + q2 * gz - q3 * gy;
  obj->q2 += q0 * gy - q1 * gz + q3 * gx;
  obj->q3 += q0 * gz + q1 * gy - q2 * gx;

  float norm = sqrtf(obj->q0 * obj->q0 + obj->q1 * obj->q1 +
                     obj->q2 * obj->q2 + obj->q3 * obj->q3);
  if (norm > 1e-6f) {
    obj->q0 /= norm;
    obj->q1 /= norm;
    obj->q2 /= norm;
    obj->q3 /= norm;
  }

  UpdateEuler(obj);
}

void Ahrs_GetGravityDirection(const Ahrs *obj, float *gx, float *gy, float *gz) {
  *gx = 2.0f * (obj->q1 * obj->q3 - obj->q0 * obj->q2);
  *gy = 2.0f * (obj->q0 * obj->q1 + obj->q2 * obj->q3);
  *gz = 2.0f * (obj->q0 * obj->q0 + obj->q3 * obj->q3) - 1.0f;
}
