#include "bateria.h"
#include "config.h"

static DatosBateria b = {};
static uint32_t msUltima = 0;
static bool primera = true;

// Curva aproximada de una celda LiPo en reposo o con poca carga (V -> %)
static const float CURVA_V[]   = {3.30f, 3.50f, 3.68f, 3.74f, 3.80f, 3.87f, 3.95f, 4.05f, 4.20f};
static const float CURVA_PCT[] = {0,     5,     20,    35,    50,    65,    80,    92,    100};
static const int   N_CURVA = sizeof(CURVA_V) / sizeof(CURVA_V[0]);

static int porcentajeDeCelda(float v) {
  if (v <= CURVA_V[0]) return 0;
  if (v >= CURVA_V[N_CURVA - 1]) return 100;
  for (int i = 1; i < N_CURVA; i++) {
    if (v <= CURVA_V[i]) {
      float f = (v - CURVA_V[i - 1]) / (CURVA_V[i] - CURVA_V[i - 1]);
      return (int)lroundf(CURVA_PCT[i - 1] + f * (CURVA_PCT[i] - CURVA_PCT[i - 1]));
    }
  }
  return 100;
}

void bateriaIniciar() {
  pinMode(PIN_BATERIA, INPUT);
  analogSetPinAttenuation(PIN_BATERIA, ADC_11db);   // rango hasta ~3,1 V
  b.nivel = BAT_OK;
}

void bateriaActualizar() {
  uint32_t ahora = millis();
  if (ahora - msUltima < 1000 / BAT_FREQ_HZ) return;
  msUltima = ahora;

  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BATERIA);   // calibrado de fábrica
  float vPin = mv / 8.0f / 1000.0f;

  // Sin divisor el pin queda cerca de 0 V: se informa "sin medición" en vez de 0 %.
  b.medida = vPin > 0.3f;
  if (!b.medida) {
    b.voltios = b.vCelda = 0;
    b.porcentaje = 0;
    b.nivel = BAT_OK;
    primera = true;
    return;
  }

  float v = vPin * (BAT_R1_OHM + BAT_R2_OHM) / BAT_R2_OHM * BAT_CALIBRACION;
  // Filtro (~1 s): las hélices hacen caer la tensión a saltos
  b.voltios = primera ? v : b.voltios + 0.1f * (v - b.voltios);
  primera = false;
  b.vCelda = b.voltios / BAT_CELDAS;
  b.porcentaje = porcentajeDeCelda(b.vCelda);

  // Con histéresis, para que el aviso no parpadee
  if (b.vCelda < BAT_V_CELDA_CRITICA) {
    b.nivel = BAT_CRITICA;
  } else if (b.vCelda < BAT_V_CELDA_BAJA) {
    if (b.nivel == BAT_OK) b.nivel = BAT_BAJA;
    else if (b.nivel == BAT_CRITICA && b.vCelda > BAT_V_CELDA_CRITICA + 0.05f) b.nivel = BAT_BAJA;
  } else if (b.vCelda > BAT_V_CELDA_BAJA + 0.05f) {
    b.nivel = BAT_OK;
  }
}

const DatosBateria &bateriaDatos() { return b; }
