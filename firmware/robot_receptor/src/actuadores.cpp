#include "actuadores.h"
#include "config.h"
#include <esp_arduino_version.h>

// ---------------- Canales LEDC ----------------
// ESC en el canal 0 (50 Hz); motores en 2..5 (20 kHz). En el core 2.x
// los canales van en parejas por temporizador, así no se pisan.
static const uint8_t CH_ESC  = 0;
static const uint8_t CH_AIN1 = 2, CH_AIN2 = 3, CH_BIN1 = 4, CH_BIN2 = 5;

static const uint8_t  ESC_BITS    = 16;
static const uint32_t ESC_FREQ    = 50;
static const uint32_t PERIODO_US  = 1000000UL / ESC_FREQ;      // 20000 us
static const uint32_t MOTOR_ALTO  = (1UL << MOTOR_PWM_BITS);    // 100 % fijo

static EstadoActuadores e = {};
static uint32_t msOrdenEsc = 0, msOrdenRuedas = 0, msPasoRampa = 0;

// ---------------- PWM compatible con core 2.x y 3.x ----------------
static void pwmAdjuntar(uint8_t pin, uint8_t canal, uint32_t freq, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, freq, bits, canal);
#else
  ledcSetup(canal, freq, bits);
  ledcAttachPin(pin, canal);
#endif
}

static void pwmEscribir(uint8_t pin, uint8_t canal, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(canal, duty);
#endif
}

// ---------------- ESC ----------------
static void escEscribirUs(int us) {
  uint32_t duty = (uint32_t)us * ((1UL << ESC_BITS) - 1) / PERIODO_US;
  pwmEscribir(PIN_ESC, CH_ESC, duty);
}

static void escApagar() {
  e.escObjetivo = ESC_APAGADO;
  e.escActual   = ESC_APAGADO;
  escEscribirUs(ESC_APAGADO);
}

// ---------------- DRV8833 (decaimiento lento) ----------------
// Adelante:  IN1 = 100 %, IN2 = PWM invertido
// Atrás:     IN1 = PWM invertido, IN2 = 100 %
// Cero:      IN1 = IN2 = 100 %  -> freno (ayuda a sostenerse en la pared)
static void puenteH(uint8_t p1, uint8_t c1, uint8_t p2, uint8_t c2, int pwm) {
  uint32_t mag = (uint32_t)min(abs(pwm), (int)MOTOR_PWM_MAX);
  uint32_t inv = MOTOR_ALTO - mag;
  if (pwm > 0)      { pwmEscribir(p1, c1, MOTOR_ALTO); pwmEscribir(p2, c2, inv); }
  else if (pwm < 0) { pwmEscribir(p1, c1, inv);        pwmEscribir(p2, c2, MOTOR_ALTO); }
  else              { pwmEscribir(p1, c1, MOTOR_ALTO); pwmEscribir(p2, c2, MOTOR_ALTO); }
}

static int velocidadAPwm(float v) {
  v = constrain(v, -1.0f, 1.0f);
  if (fabsf(v) < 0.02f) return 0;
  int mag = MOTOR_PWM_MIN_ARRANQUE + (int)(fabsf(v) * (MOTOR_PWM_MAX - MOTOR_PWM_MIN_ARRANQUE));
  return v > 0 ? mag : -mag;
}

static void aplicarRuedas(float izq, float der) {
  e.ruedaIzq = constrain(izq, -1.0f, 1.0f);
  e.ruedaDer = constrain(der, -1.0f, 1.0f);
  e.pwmIzq = velocidadAPwm(MOTOR_IZQ_INVERTIDO ? -e.ruedaIzq : e.ruedaIzq);
  e.pwmDer = velocidadAPwm(MOTOR_DER_INVERTIDO ? -e.ruedaDer : e.ruedaDer);
  puenteH(PIN_AIN1, CH_AIN1, PIN_AIN2, CH_AIN2, e.pwmIzq);
  puenteH(PIN_BIN1, CH_BIN1, PIN_BIN2, CH_BIN2, e.pwmDer);
}

// ---------------- API ----------------
void actuadoresIniciar() {
  pwmAdjuntar(PIN_ESC,  CH_ESC,  ESC_FREQ,       ESC_BITS);
  pwmAdjuntar(PIN_AIN1, CH_AIN1, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  pwmAdjuntar(PIN_AIN2, CH_AIN2, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  pwmAdjuntar(PIN_BIN1, CH_BIN1, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  pwmAdjuntar(PIN_BIN2, CH_BIN2, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);

  aplicarRuedas(0, 0);
  escApagar();
  Serial.println("[ESC] Armando (1000 us)...");
  delay(ESC_TIEMPO_ARMADO_MS);
  Serial.println("[ESC] Listo");
  e.timeoutEsc = e.timeoutRuedas = true;   // nada ordenado todavía
}

void escOrden(int us) {
  msOrdenEsc = millis();
  e.timeoutEsc = false;
  if (e.paradaActiva) return;
  if (us < ESC_MIN) { escApagar(); return; }
  e.escObjetivo = min(us, ESC_MAX);
}

void ruedasOrden(float izq, float der) {
  msOrdenRuedas = millis();
  e.timeoutRuedas = false;
  if (e.paradaActiva) return;
  aplicarRuedas(izq, der);
}

void pararTodo() {
  escApagar();
  aplicarRuedas(0, 0);
}

void paradaEmergencia(bool activa) {
  if (activa && !e.paradaActiva) Serial.println("[SEGURIDAD] Parada de emergencia ACTIVADA");
  if (!activa && e.paradaActiva) Serial.println("[SEGURIDAD] Parada de emergencia liberada");
  e.paradaActiva = activa;
  if (activa) pararTodo();
}

void actuadoresActualizar() {
  uint32_t ahora = millis();

  // Seguridad: sin órdenes recientes -> parar (como el receptor original)
  if (!e.timeoutEsc && ahora - msOrdenEsc > ESC_TIMEOUT_MS) {
    e.timeoutEsc = true;
    if (e.escObjetivo != ESC_APAGADO || e.escActual != ESC_APAGADO) {
      escApagar();
      Serial.println("[SEGURIDAD] Sin órdenes de ESC: hélices apagadas");
    }
  }
  if (!e.timeoutRuedas && ahora - msOrdenRuedas > MOTORES_TIMEOUT_MS) {
    e.timeoutRuedas = true;
    if (e.pwmIzq != 0 || e.pwmDer != 0) aplicarRuedas(0, 0);
  }

  // Rampa del ESC
  if (ahora - msPasoRampa >= ESC_INTERVALO_RAMPA_MS) {
    msPasoRampa = ahora;
    if (e.escActual != e.escObjetivo) {
      if (e.escActual < e.escObjetivo) {
        // desde apagado salta directo al mínimo útil
        int base = max(e.escActual, ESC_MIN - ESC_PASO_RAMPA);
        e.escActual = min(base + ESC_PASO_RAMPA, e.escObjetivo);
      } else {
        e.escActual = max(e.escActual - ESC_PASO_RAMPA, e.escObjetivo);
      }
      escEscribirUs(e.escActual);
    }
  }

  e.msDesdeOrdenEsc    = msOrdenEsc    ? ahora - msOrdenEsc    : UINT32_MAX;
  e.msDesdeOrdenRuedas = msOrdenRuedas ? ahora - msOrdenRuedas : UINT32_MAX;
}

const EstadoActuadores &actuadoresEstado() { return e; }
