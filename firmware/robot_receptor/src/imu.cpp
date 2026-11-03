#include "imu.h"
#include "config.h"
#include <Wire.h>

static const uint8_t DIR_MPU = 0x68;   // AD0 a GND (0x69 si AD0 a 3.3 V)
static const float   G = 9.80665f;
static const float   ACC_LSB  = 8192.0f;                 // ±4 g
static const float   GIRO_LSB = 65.5f;                   // ±500 °/s
static const float   GRAD_A_RAD = PI / 180.0f;

static DatosImu d = {};
static float sesgoGx = 0, sesgoGy = 0, sesgoGz = 0;
static float intEx = 0, intEy = 0, intEz = 0;            // integral de Mahony
static uint32_t usUltima = 0;
static uint32_t msReintento = 0;
static uint16_t erroresSeguidos = 0;

// ---------------- I2C ----------------
static bool escribirReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DIR_MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool leerRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(DIR_MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(DIR_MPU, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool configurarSensor() {
  uint8_t quien = 0;
  if (!leerRegs(0x75, &quien, 1)) return false;
  // 0x68 = original; 0x70/0x72/0x98 = clones habituales (MPU6500, etc.)
  if (quien != 0x68 && quien != 0x70 && quien != 0x72 && quien != 0x98) {
    Serial.printf("[IMU] WHO_AM_I inesperado: 0x%02X (se intenta igual)\n", quien);
  }
  bool ok = escribirReg(0x6B, 0x01);   // despertar, reloj del giro X
  delay(50);
  ok &= escribirReg(0x1A, 0x03);       // filtro digital ~44 Hz
  ok &= escribirReg(0x19, 0x04);       // 1 kHz / (1+4) = 200 Hz
  ok &= escribirReg(0x1B, 0x08);       // giro ±500 °/s
  ok &= escribirReg(0x1C, 0x08);       // acel ±4 g
  return ok;
}

// Lectura cruda -> unidades SI (sin quitar el sesgo del giro)
static bool leerSensor(float &ax, float &ay, float &az,
                       float &gx, float &gy, float &gz, float &temp) {
  uint8_t b[14];
  if (!leerRegs(0x3B, b, 14)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  ax = s16(0)  / ACC_LSB * G;
  ay = s16(2)  / ACC_LSB * G;
  az = s16(4)  / ACC_LSB * G;
  temp = s16(6) / 340.0f + 36.53f;
  gx = s16(8)  / GIRO_LSB * GRAD_A_RAD;
  gy = s16(10) / GIRO_LSB * GRAD_A_RAD;
  gz = s16(12) / GIRO_LSB * GRAD_A_RAD;
  return true;
}

// ---------------- Orientación ----------------
static void cuaternionDesdeAcel(float ax, float ay, float az) {
  float r = atan2f(ay, az);
  float p = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
  d.qw = cr * cp;  d.qx = sr * cp;  d.qy = cr * sp;  d.qz = -sr * sp;
}

static void mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float q0 = d.qw, q1 = d.qx, q2 = d.qy, q3 = d.qz;
  float norma = sqrtf(ax * ax + ay * ay + az * az);

  // Corregir con el acelerómetro solo si mide algo parecido a 1 g
  // (en choques o arranques bruscos la aceleración engaña).
  if (norma > 0.7f * G && norma < 1.3f * G) {
    ax /= norma; ay /= norma; az /= norma;
    float vx = q1 * q3 - q0 * q2;
    float vy = q0 * q1 + q2 * q3;
    float vz = q0 * q0 - 0.5f + q3 * q3;
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;
    if (MAHONY_KI > 0) {
      intEx += 2 * MAHONY_KI * ex * dt;
      intEy += 2 * MAHONY_KI * ey * dt;
      intEz += 2 * MAHONY_KI * ez * dt;
      gx += intEx; gy += intEy; gz += intEz;
    }
    gx += 2 * MAHONY_KP * ex;
    gy += 2 * MAHONY_KP * ey;
    gz += 2 * MAHONY_KP * ez;
  }

  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);
  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  d.qw = q0 / n; d.qx = q1 / n; d.qy = q2 / n; d.qz = q3 / n;

  d.roll  = atan2f(2 * (d.qw * d.qx + d.qy * d.qz), 1 - 2 * (d.qx * d.qx + d.qy * d.qy)) / GRAD_A_RAD;
  float s = constrain(2 * (d.qw * d.qy - d.qz * d.qx), -1.0f, 1.0f);
  d.pitch = asinf(s) / GRAD_A_RAD;
  d.yaw   = atan2f(2 * (d.qw * d.qz + d.qx * d.qy), 1 - 2 * (d.qy * d.qy + d.qz * d.qz)) / GRAD_A_RAD;
}

// ---------------- API ----------------
bool imuIniciar() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
  d.qw = 1;

  if (!configurarSensor()) {
    Serial.println("[IMU] MPU6050 no encontrado (revisa SDA=21, SCL=22, 3.3 V)");
    d.ok = false;
    return false;
  }

  Serial.println("[IMU] Calibrando giroscopio, no muevas el robot...");
  const int N = 400;
  int validas = 0;
  float sx = 0, sy = 0, sz = 0, sax = 0, say = 0, saz = 0;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz, t;
    if (leerSensor(ax, ay, az, gx, gy, gz, t)) {
      sx += gx; sy += gy; sz += gz; sax += ax; say += ay; saz += az;
      validas++;
    }
    delay(5);
  }
  if (validas < N / 2) {
    Serial.println("[IMU] Demasiados errores durante la calibración");
    d.ok = false;
    return false;
  }
  sesgoGx = sx / validas; sesgoGy = sy / validas; sesgoGz = sz / validas;
  cuaternionDesdeAcel(sax / validas, say / validas, saz / validas);
  usUltima = micros();
  d.ok = true;
  Serial.printf("[IMU] Lista. Sesgo giro (rad/s): %.4f %.4f %.4f\n", sesgoGx, sesgoGy, sesgoGz);
  return true;
}

void imuActualizar() {
  uint32_t ahora = micros();
  if (ahora - usUltima < 1000000UL / IMU_FREQ_HZ) return;
  float dt = (ahora - usUltima) * 1e-6f;
  usUltima = ahora;
  if (dt > 0.1f) dt = 0.1f;   // tras una pausa larga (p. ej. reconexión)

  // Sensor caído: reintentar cada segundo sin bloquear el resto
  if (!d.ok) {
    if (millis() - msReintento > 1000) {
      msReintento = millis();
      if (configurarSensor()) { d.ok = true; erroresSeguidos = 0; Serial.println("[IMU] Recuperada"); }
    }
    return;
  }

  float ax, ay, az, gx, gy, gz, t;
  if (!leerSensor(ax, ay, az, gx, gy, gz, t)) {
    d.errores++;
    if (++erroresSeguidos > 50) { d.ok = false; Serial.println("[IMU] Sin respuesta"); }
    return;
  }
  erroresSeguidos = 0;

  d.ax = ax; d.ay = ay; d.az = az;
  d.gx = gx - sesgoGx; d.gy = gy - sesgoGy; d.gz = gz - sesgoGz;
  d.temperatura = t;
  d.lecturas++;
  mahony(d.gx, d.gy, d.gz, ax, ay, az, dt);
}

const DatosImu &imuDatos() { return d; }
