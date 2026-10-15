/*
 * MPU6050_Attitude.ino
 * ------------------------------------------------------------------
 * ESP32-WROOM-32 + MPU6050 六轴姿态解算（Arduino 环境）
 *
 * 功能：
 *  1. I2C 读取 MPU6050（SCL/SDA）
 *  2. 上电自动校准陀螺仪零偏；以“上电瞬间的姿态”作为 0 初始位置
 *  3. Mahony 互补滤波姿态解算，输出相对初始位置的四元数与欧拉角
 *  4. 串口 JSON 输出（默认 115200），供前端 3D 界面实时显示
 *  5. 串口收到字符 R（或 r）时重新归零（把当前姿态设为 0）
 *
 * 接线（ESP32-WROOM-32 DevKit）：
 *    MPU6050  VCC  ->  3V3
 *    MPU6050  GND  ->  GND
 *    MPU6050  SCL  ->  GPIO22
 *    MPU6050  SDA  ->  GPIO21
 *    MPU6050  AD0  ->  GND（I2C 地址 0x68）
 *    XDA / XCL / INT 悬空即可
 *
 * 依赖：仅 Arduino ESP32 内核自带的 Wire 库，无需第三方库。
 * 注意：上电后请保持模块静止约 3 秒，完成校准与归零。
 */

#include <Wire.h>

/* ---------------- 引脚与参数 ---------------- */
#define PIN_SDA       21
#define PIN_SCL       22
#define MPU_ADDR      0x68          // AD0 接 GND 时地址为 0x68

#define LOOP_HZ       100           // 姿态解算/输出频率
#define CALIB_N       300           // 上电校准采样数（100Hz 下约 3 秒）
#define GYRO_FS       2000.0f       // 陀螺仪量程（dps）
#define ACC_FS        2.0f          // 加速度计量程（g）
#define TWO_KP        1.0f          // Mahony 比例增益（twoKp = 2*Kp，即 Kp ≈ 0.5）
#define TWO_KI        0.02f         // Mahony 积分增益（twoKi = 2*Ki，即 Ki ≈ 0.01）

/* ---------------- MPU6050 寄存器 ---------------- */
#define REG_WHOAMI       0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

static const float DEG2RAD = PI / 180.0f;

/* 姿态四元数（世界 -> 机体）与参考四元数 */
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float rq0 = 1, rq1 = 0, rq2 = 0, rq3 = 0;   // 上电/归零参考姿态
float dq0 = 1, dq1 = 0, dq2 = 0, dq3 = 0;   // 相对参考的姿态

float gbias[3] = {0, 0, 0};                // 陀螺仪零偏
float ibx = 0, iby = 0, ibz = 0;           // Mahony 积分项
float outRoll = 0, outPitch = 0, outYaw = 0;

/* ---------------- I2C 读写 ---------------- */
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len);
  if (Wire.available() < len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

/* ---------------- 初始化 ---------------- */
bool initMPU6050() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  uint8_t id = 0;
  if (!readRegs(REG_WHOAMI, &id, 1) || id != 0x68) return false;

  writeReg(REG_PWR_MGMT_1, 0x00);      // 唤醒（清 sleep 与复位）
  delay(50);
  writeReg(REG_SMPLRT_DIV, 9);         // 采样率 1000/(1+9) = 100Hz
  writeReg(REG_CONFIG, 0x03);          // DLPF：加速度 44Hz / 陀螺 41Hz
  writeReg(REG_GYRO_CONFIG, 0x18);     // ±2000dps
  writeReg(REG_ACCEL_CONFIG, 0x00);    // ±2g
  return true;
}

/* 读取 14 字节原始数据：accel(3) temp(1) gyro(3) */
bool readRaw(int16_t *ax, int16_t *ay, int16_t *az,
             int16_t *gx, int16_t *gy, int16_t *gz) {
  uint8_t buf[14];
  if (!readRegs(REG_ACCEL_XOUT_H, buf, 14)) return false;
  *ax = (int16_t)((buf[0] << 8) | buf[1]);
  *ay = (int16_t)((buf[2] << 8) | buf[3]);
  *az = (int16_t)((buf[4] << 8) | buf[5]);
  *gx = (int16_t)((buf[8] << 8) | buf[9]);
  *gy = (int16_t)((buf[10] << 8) | buf[11]);
  *gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

/* ---------------- 陀螺仪零偏校准（上电静止约 3 秒） ---------------- */
void calibrateGyro() {
  int32_t sum[3] = {0, 0, 0};
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < CALIB_N; i++) {
    if (readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
      sum[0] += gx; sum[1] += gy; sum[2] += gz;
    }
    delay(10);
  }
  gbias[0] = sum[0] / (float)CALIB_N;
  gbias[1] = sum[1] / (float)CALIB_N;
  gbias[2] = sum[2] / (float)CALIB_N;
}

/* ---------------- 归零：把当前姿态设为 0 初始位置 ---------------- */
void zeroAttitude() {
  int32_t sx = 0, sy = 0, sz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  const int N = 32;
  for (int i = 0; i < N; i++) {
    if (readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
      sx += ax; sy += ay; sz += az;
    }
    delay(5);
  }
  float axg = (sx / (float)N) * (ACC_FS / 32768.0f);
  float ayg = (sy / (float)N) * (ACC_FS / 32768.0f);
  float azg = (sz / (float)N) * (ACC_FS / 32768.0f);

  /* 由重力方向求初始横滚/俯仰（偏航取 0），构造参考四元数 rq = Ry(pitch)*Rx(roll) */
  float roll  = atan2f(ayg, azg);
  float pitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg));
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  rq0 = cp * cr; rq1 = cp * sr; rq2 = sp * cr; rq3 = -sp * sr;
}

/* ---------------- Mahony 滤波 ---------------- */
void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  /* 加速度计归一化 */
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm > 1e-6f) { ax /= norm; ay /= norm; az /= norm; }
  else { ax = ay = az = 0; }

  /* 估计重力方向（由当前四元数推算） */
  float hvx = q1 * q3 - q0 * q2;
  float hvy = q0 * q1 + q2 * q3;
  float hvz = q0 * q0 - 0.5f + q3 * q3;

  /* 测量与估计的叉积 = 姿态误差 */
  float ex = ay * hvz - az * hvy;
  float ey = az * hvx - ax * hvz;
  float ez = ax * hvy - ay * hvx;

  if (TWO_KI > 0.0f) {
    ibx += TWO_KI * ex * dt;
    iby += TWO_KI * ey * dt;
    ibz += TWO_KI * ez * dt;
    gx += ibx; gy += iby; gz += ibz;
  }
  gx += TWO_KP * ex;
  gy += TWO_KP * ey;
  gz += TWO_KP * ez;

  /* 四元数积分（一阶）并归一化 */
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (norm > 1e-10f) { q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm; }
}

/* 相对参考姿态：dq = conj(rq) * q */
void deltaFromReference() {
  float c0 = rq0, c1 = -rq1, c2 = -rq2, c3 = -rq3;
  dq0 = c0 * q0 - c1 * q1 - c2 * q2 - c3 * q3;
  dq1 = c0 * q1 + c1 * q0 + c2 * q3 - c3 * q2;
  dq2 = c0 * q2 - c1 * q3 + c2 * q0 + c3 * q1;
  dq3 = c0 * q3 + c1 * q2 - c2 * q1 + c3 * q0;
}

/* 四元数 -> 欧拉角（相对参考，单位：度） */
void quatToEuler() {
  outRoll  = atan2f(2.0f * (dq0 * dq1 + dq2 * dq3),
                    1.0f - 2.0f * (dq1 * dq1 + dq2 * dq2)) / DEG2RAD;
  float sp = 2.0f * (dq0 * dq2 - dq3 * dq1);
  sp = constrain(sp, -1.0f, 1.0f);
  outPitch = asinf(sp) / DEG2RAD;
  outYaw   = atan2f(2.0f * (dq0 * dq3 + dq1 * dq2),
                    1.0f - 2.0f * (dq2 * dq2 + dq3 * dq3)) / DEG2RAD;
}

/* ---------------- 主程序 ---------------- */
#ifdef LED_BUILTIN
#define CALIB_LED LED_BUILTIN
#else
#define CALIB_LED 2
#endif

void setup() {
  Serial.begin(115200);
  pinMode(CALIB_LED, OUTPUT);

  Serial.println("{\"msg\":\"boot: calibrating, keep sensor still ~3s\"}");

  if (!initMPU6050()) {
    Serial.println("{\"error\":\"MPU6050 not found, check SCL/SDA wiring\"}");
    while (1) { digitalWrite(CALIB_LED, !digitalRead(CALIB_LED)); delay(200); }
  }

  /* 校准期间 LED 闪烁提示“请保持静止” */
  digitalWrite(CALIB_LED, LOW);
  calibrateGyro();
  digitalWrite(CALIB_LED, HIGH);

  zeroAttitude();
  Serial.println("{\"msg\":\"ready: power-on pose = zero reference\"}");
}

void loop() {
  static uint32_t lastUs = 0;
  uint32_t now = micros();
  if (now - lastUs < (uint32_t)(1000000UL / LOOP_HZ)) return;
  float dt = (now - lastUs) / 1000000.0f;
  lastUs = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / LOOP_HZ;

  /* 串口指令：R = 重新归零 */
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'R' || c == 'r') {
      zeroAttitude();
      Serial.println("{\"msg\":\"re-zeroed\"}");
    }
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
    /* 去零偏，转换为 rad/s 与 g */
    float gxr = (gx - gbias[0]) * (GYRO_FS / 32768.0f) * DEG2RAD;
    float gyr = (gy - gbias[1]) * (GYRO_FS / 32768.0f) * DEG2RAD;
    float gzr = (gz - gbias[2]) * (GYRO_FS / 32768.0f) * DEG2RAD;
    float axg = ax * (ACC_FS / 32768.0f);
    float ayg = ay * (ACC_FS / 32768.0f);
    float azg = az * (ACC_FS / 32768.0f);

    mahonyUpdate(gxr, gyr, gzr, axg, ayg, azg, dt);
    deltaFromReference();
    quatToEuler();

    char buf[120];
    snprintf(buf, sizeof(buf),
             "{\"q0\":%.3f,\"q1\":%.3f,\"q2\":%.3f,\"q3\":%.3f,"
             "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
             dq0, dq1, dq2, dq3, outRoll, outPitch, outYaw);
    Serial.println(buf);
  }
}
