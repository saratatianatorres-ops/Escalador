#pragma once
#include <Arduino.h>

// ============================================================
//  MPU6050 por I2C (sin librerías externas) + filtro de Mahony
//  Acelerómetro ±4 g, giroscopio ±500 °/s, filtro digital 44 Hz.
// ============================================================

struct DatosImu {
  bool  ok;                    // sensor encontrado y respondiendo
  float ax, ay, az;            // m/s²
  float gx, gy, gz;            // rad/s (sin sesgo)
  float qw, qx, qy, qz;        // orientación (cuaternión)
  float roll, pitch, yaw;      // grados (el yaw deriva: no hay magnetómetro)
  float temperatura;           // °C
  uint32_t lecturas;
  uint32_t errores;
};

bool imuIniciar();             // calibra el giro: el robot debe estar QUIETO (~2 s)
void imuActualizar();          // llamar en cada loop(); lee a IMU_FREQ_HZ
const DatosImu &imuDatos();
