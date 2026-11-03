#pragma once
#include <Arduino.h>

// ============================================================
//  Transporte micro-ROS sobre ESP-NOW
// ------------------------------------------------------------
//  El cliente micro-ROS genera bytes XRCE-DDS con "framing" serie
//  (el mismo formato que usa el agente con `serial`). Este módulo
//  los parte en paquetes ESP-NOW:
//
//      [ cabecera 1 byte ][ hasta 249 bytes de datos XRCE ]
//
//      cabecera = 0xA1  robot  -> puente
//      cabecera = 0xA2  puente -> robot
//
//  La ESP puente solo copia los datos entre ESP-NOW y su USB,
//  así que en el PC basta con:
//      ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
// ============================================================

bool transporteRegistrar();          // llamar una vez en setup()

// Estadísticas (se publican en /robot/estado)
struct EstadisticasEspNow {
  uint32_t paquetesRx;
  uint32_t paquetesTx;
  uint32_t fallosTx;
  uint32_t bytesDescartados;   // buffer de recepción lleno
  bool     puenteConocido;
  uint8_t  macPuente[6];
};
const EstadisticasEspNow &transporteEstadisticas();
uint32_t transporteMsDesdeUltimoRx();
