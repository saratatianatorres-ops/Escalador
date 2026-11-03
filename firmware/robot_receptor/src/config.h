#pragma once
#include <Arduino.h>

// ============================================================
//  CONFIGURACIÓN GENERAL DEL ROBOT (todo lo ajustable está aquí)
// ============================================================

// ---------------- ESP-NOW ----------------
// Canal WiFi: debe ser el MISMO en el robot y en la ESP puente (PC).
#define CANAL_ESPNOW 1

// MAC de la ESP puente (la conectada al PC).
// Si la dejas toda en 0xFF, el robot la aprende sola con el primer
// paquete válido que reciba y desde ahí solo acepta a esa ESP.
static const uint8_t MAC_PUENTE[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Primer byte de cada paquete ESP-NOW (filtra paquetes ajenos).
#define CAB_ROBOT_A_PUENTE 0xA1
#define CAB_PUENTE_A_ROBOT 0xA2

// ---------------- Pines ----------------
#define PIN_ESC   18   // señal de los dos ESC (en paralelo)
#define PIN_AIN1  26   // DRV8833 motor A (izquierdo)
#define PIN_AIN2  25
#define PIN_BIN1  33   // DRV8833 motor B (derecho)
#define PIN_BIN2  32
#define PIN_SDA   21   // MPU6050
#define PIN_SCL   22

// ---------------- ESC (hélices) ----------------
#define ESC_APAGADO             1000   // us
#define ESC_MIN                 1100   // us
#define ESC_MAX                 2000   // us
#define ESC_PASO_RAMPA          20     // us por paso
#define ESC_INTERVALO_RAMPA_MS  200    // ms entre pasos
#define ESC_TIEMPO_ARMADO_MS    3000   // 1000 us al arrancar para armar
#define ESC_TIMEOUT_MS          2000   // sin órdenes de ESC -> apagar

// ---------------- Motorreductores (DRV8833) ----------------
#define MOTOR_PWM_FREQ          20000  // Hz (fuera del rango audible)
#define MOTOR_PWM_BITS          10
#define MOTOR_PWM_MAX           1023
#define MOTOR_PWM_MIN_ARRANQUE  300    // duty mínimo para que el N20 se mueva
#define MOTOR_IZQ_INVERTIDO     false  // cambia si un lado gira al revés
#define MOTOR_DER_INVERTIDO     false
#define MOTORES_TIMEOUT_MS      500    // sin órdenes de ruedas -> parar

// Conversión de /cmd_vel (lazo abierto, sin encoders)
#define V_MAX_MS     0.30f   // velocidad aprox. del robot con PWM al 100 %
#define ANCHO_VIA_M  0.15f   // distancia entre ruedas izquierda y derecha

// ---------------- IMU (MPU6050) ----------------
#define IMU_FREQ_HZ    200   // lectura + filtro de orientación
#define IMU_PUB_HZ     50    // publicación de /robot/imu
#define ESTADO_PUB_HZ  5     // publicación de /robot/estado
#define MAHONY_KP      1.0f
#define MAHONY_KI      0.0f
#define IMU_FRAME_ID   "imu_link"

// ---------------- micro-ROS ----------------
#define NOMBRE_NODO      "robot_trepador"
#define ESPACIO_NOMBRES  "robot"   // tópicos quedan como /robot/...
