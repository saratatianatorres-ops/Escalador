#pragma once
#include <Arduino.h>

// ============================================================
//  Batería: tensión por divisor resistivo en PIN_BATERIA (ADC1)
// ============================================================

enum NivelBateria : uint8_t { BAT_OK = 0, BAT_BAJA = 1, BAT_CRITICA = 2 };

struct DatosBateria {
  bool    medida;        // false = no hay divisor (pin a ~0 V)
  float   voltios;       // tensión total de la batería (filtrada)
  float   vCelda;        // voltios / BAT_CELDAS
  int     porcentaje;    // 0..100, estimado por la tensión de cada celda
  NivelBateria nivel;
};

void bateriaIniciar();
void bateriaActualizar();      // llamar en cada loop(); mide a BAT_FREQ_HZ
const DatosBateria &bateriaDatos();
