/*
  ESP32 PUENTE  (PC <-> ESP-NOW <-> robot)
  ----------------------------------------
  - USB a 921600 baudios, conectado al micro_ros_agent del PC.
  - Lo que llega por USB se envía al robot por ESP-NOW (cabecera 0xA2).
  - Lo que llega del robot (cabecera 0xA1) se escribe tal cual en el USB.
  - Aprende la MAC del robot con el primer paquete que recibe de él.

  IMPORTANTE: este programa no imprime NADA por Serial, porque el puerto
  es exclusivo del agente. El LED (GPIO 2) indica el estado:
     parpadeo lento  -> esperando al robot
     encendido fijo  -> robot conocido, con tráfico reciente
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <freertos/stream_buffer.h>
#include <freertos/semphr.h>

// ---------------- Configuración (igual que en el robot) ----------------
#define CANAL_ESPNOW        1
#define BAUDIOS             921600
#define CAB_ROBOT_A_PUENTE  0xA1
#define CAB_PUENTE_A_ROBOT  0xA2
#define PIN_LED             2

// MAC del robot. Con todo 0xFF se aprende sola del primer paquete.
static const uint8_t MAC_ROBOT[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------- Internos ----------------
static const size_t   MAX_DATOS  = ESP_NOW_MAX_DATA_LEN - 1;   // 249
static const uint32_t US_PAUSA   = 300;   // silencio en el USB que cierra un paquete
static const uint8_t  BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static StreamBufferHandle_t bufferRobot;   // robot -> USB
static SemaphoreHandle_t    semEnvio;
static uint8_t  macDestino[6];
static bool     robotConocido = false;
static volatile bool macPendiente = false;
static uint8_t  macAprendida[6];
static volatile uint32_t msUltimoRx = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void alRecibir(const esp_now_recv_info_t *info, const uint8_t *d, int len) {
  const uint8_t *mac = info->src_addr;
#else
static void alRecibir(const uint8_t *mac, const uint8_t *d, int len) {
#endif
  if (len < 2 || d[0] != CAB_ROBOT_A_PUENTE) return;
  if (robotConocido) {
    if (memcmp(mac, macDestino, 6) != 0) return;
  } else if (!macPendiente) {
    memcpy(macAprendida, mac, 6);
    macPendiente = true;
  }
  xStreamBufferSend(bufferRobot, d + 1, len - 1, 0);
  msUltimoRx = millis();
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void alEnviar(const wifi_tx_info_t *, esp_now_send_status_t) {
#else
static void alEnviar(const uint8_t *, esp_now_send_status_t) {
#endif
  xSemaphoreGive(semEnvio);
}

static bool agregarPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = CANAL_ESPNOW;
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

static void fijarRobot(const uint8_t *mac) {
  if (agregarPeer(mac)) {
    memcpy(macDestino, mac, 6);
    robotConocido = true;
  }
}

static void enviarAlRobot(const uint8_t *datos, size_t n) {
  uint8_t paquete[ESP_NOW_MAX_DATA_LEN];
  paquete[0] = CAB_PUENTE_A_ROBOT;
  memcpy(paquete + 1, datos, n);
  xSemaphoreTake(semEnvio, 0);
  if (esp_now_send(macDestino, paquete, n + 1) == ESP_OK) {
    xSemaphoreTake(semEnvio, pdMS_TO_TICKS(30));
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(4096);
  Serial.begin(BAUDIOS);

  bufferRobot = xStreamBufferCreate(8192, 1);
  semEnvio    = xSemaphoreCreateBinary();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }  // error: parpadeo rápido
  }
  esp_now_register_recv_cb(alRecibir);
  esp_now_register_send_cb(alEnviar);

  if (memcmp(MAC_ROBOT, BROADCAST, 6) != 0) {
    fijarRobot(MAC_ROBOT);
  } else {
    agregarPeer(BROADCAST);
    memcpy(macDestino, BROADCAST, 6);
  }
}

void loop() {
  if (macPendiente && !robotConocido) {
    fijarRobot(macAprendida);
    macPendiente = false;
  }

  // ---- Robot -> USB ----
  uint8_t tmp[256];
  size_t n;
  while ((n = xStreamBufferReceive(bufferRobot, tmp, sizeof(tmp), 0)) > 0) {
    Serial.write(tmp, n);
  }

  // ---- USB -> robot (paquetes de hasta 249 bytes) ----
  static uint8_t acum[MAX_DATOS];
  static size_t  lleno = 0;
  static uint32_t usUltimoByte = 0;

  while (Serial.available() && lleno < MAX_DATOS) {
    acum[lleno++] = Serial.read();
    usUltimoByte = micros();
  }
  if (lleno == MAX_DATOS || (lleno > 0 && micros() - usUltimoByte > US_PAUSA)) {
    enviarAlRobot(acum, lleno);
    lleno = 0;
  }

  // ---- LED ----
  bool activo = robotConocido && (millis() - msUltimoRx < 1000);
  digitalWrite(PIN_LED, activo ? HIGH : ((millis() / 500) % 2));
}
