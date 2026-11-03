#pragma once
#include <Arduino.h>

// ============================================================
//  Actuadores: ESC de las hélices (pin 18, dos ESC en paralelo)
//  y dos motorreductores con un DRV8833.
// ============================================================

void actuadoresIniciar();          // arma los ESC (bloquea ESC_TIEMPO_ARMADO_MS)
void actuadoresActualizar();       // rampa del ESC y timeouts; llamar en cada loop()

// ESC: <ESC_MIN (p. ej. 1000) = apagar al instante; 1100..ESC_MAX (1700) = objetivo con rampa; más se recorta
void escOrden(int us);

// Ruedas: -1.0 .. +1.0 por lado (positivo = adelante)
void ruedasOrden(float izq, float der);

// Parada de emergencia con enclavamiento: mientras esté activa se
// ignoran todas las órdenes de ESC y ruedas.
void paradaEmergencia(bool activa);

// Apaga todo sin enclavar (pérdida de enlace, reinicio de sesión...)
void pararTodo();

struct EstadoActuadores {
  int      escObjetivo;
  int      escActual;
  uint32_t msDesdeOrdenEsc;
  float    ruedaIzq, ruedaDer;     // orden recibida (-1..1)
  int      pwmIzq, pwmDer;         // duty aplicado con signo (-1023..1023)
  uint32_t msDesdeOrdenRuedas;
  bool     paradaActiva;
  bool     timeoutEsc;
  bool     timeoutRuedas;
};
const EstadoActuadores &actuadoresEstado();
