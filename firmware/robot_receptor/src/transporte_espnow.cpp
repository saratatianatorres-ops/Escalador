#include "transporte_espnow.h"
#include "config.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/semphr.h>

#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

// ---------------- Parámetros internos ----------------
static const size_t   MAX_DATOS_PAQUETE = ESP_NOW_MAX_DATA_LEN - 1;  // 249
static const size_t   TAM_BUFFER_RX     = 4096;
static const uint32_t ESPERA_ENVIO_MS   = 30;   // espera de confirmación por paquete

static const uint8_t MAC_BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static StreamBufferHandle_t bufferRx = nullptr;
static SemaphoreHandle_t    semEnvio = nullptr;
static volatile bool        ultimoEnvioOK = true;
static volatile uint32_t    msUltimoRx = 0;

static EstadisticasEspNow est = {};
static uint8_t  macDestino[6];
static volatile bool macPendiente = false;   // aprendida en el callback, falta añadir el peer
static uint8_t  macAprendida[6];

// ---------------- Callbacks ESP-NOW (tarea WiFi) ----------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void alRecibir(const esp_now_recv_info_t *info, const uint8_t *datos, int len) {
  const uint8_t *mac = info->src_addr;
#else
static void alRecibir(const uint8_t *mac, const uint8_t *datos, int len) {
#endif
  if (len < 2 || datos[0] != CAB_PUENTE_A_ROBOT) return;

  if (est.puenteConocido) {
    if (memcmp(mac, macDestino, 6) != 0) return;          // otra ESP: ignorar
  } else if (!macPendiente) {
    memcpy(macAprendida, mac, 6);                          // primer puente que habla
    macPendiente = true;
  }

  size_t n = xStreamBufferSend(bufferRx, datos + 1, len - 1, 0);
  if (n < (size_t)(len - 1)) est.bytesDescartados += (len - 1) - n;
  est.paquetesRx++;
  msUltimoRx = millis();
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void alEnviar(const wifi_tx_info_t *info, esp_now_send_status_t estado) {
#else
static void alEnviar(const uint8_t *mac, esp_now_send_status_t estado) {
#endif
  ultimoEnvioOK = (estado == ESP_NOW_SEND_SUCCESS);
  xSemaphoreGive(semEnvio);
}

// ---------------- Utilidades ----------------
static bool agregarPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = CANAL_ESPNOW;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

static void fijarPuente(const uint8_t *mac) {
  if (agregarPeer(mac)) {
    memcpy(macDestino, mac, 6);
    memcpy(est.macPuente, mac, 6);
    est.puenteConocido = true;
    Serial.printf("[ESP-NOW] Puente: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}

// ---------------- Funciones del transporte micro-ROS ----------------
static bool trOpen(struct uxrCustomTransport *) {
  return true;   // ESP-NOW ya está iniciado en transporteRegistrar()
}

static bool trClose(struct uxrCustomTransport *) {
  return true;
}

static size_t trWrite(struct uxrCustomTransport *, const uint8_t *buf, size_t len, uint8_t *err) {
  if (macPendiente && !est.puenteConocido) {
    fijarPuente(macAprendida);
    macPendiente = false;
  }

  uint8_t paquete[ESP_NOW_MAX_DATA_LEN];
  paquete[0] = CAB_ROBOT_A_PUENTE;
  size_t enviados = 0;

  while (enviados < len) {
    size_t n = min(MAX_DATOS_PAQUETE, len - enviados);
    memcpy(paquete + 1, buf + enviados, n);

    xSemaphoreTake(semEnvio, 0);   // limpiar confirmación vieja
    esp_err_t r = esp_now_send(macDestino, paquete, n + 1);
    if (r != ESP_OK ||
        xSemaphoreTake(semEnvio, pdMS_TO_TICKS(ESPERA_ENVIO_MS)) != pdTRUE) {
      est.fallosTx++;
      *err = 1;
      return enviados;
    }
    // En broadcast (puente aún desconocido) no hay ACK: no cuenta como fallo.
    if (!ultimoEnvioOK && est.puenteConocido) est.fallosTx++;

    est.paquetesTx++;
    enviados += n;
  }
  return enviados;
}

static size_t trRead(struct uxrCustomTransport *, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
  size_t n = xStreamBufferReceive(bufferRx, buf, len, pdMS_TO_TICKS(timeout));
  if (n == 0) *err = 0;   // sin datos todavía: no es error
  return n;
}

// ---------------- API pública ----------------
bool transporteRegistrar() {
  bufferRx = xStreamBufferCreate(TAM_BUFFER_RX, 1);
  semEnvio = xSemaphoreCreateBinary();
  if (!bufferRx || !semEnvio) return false;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);                 // menos latencia
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("[ESP-NOW] MAC de este robot: %02X:%02X:%02X:%02X:%02X:%02X  (canal %d)\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], CANAL_ESPNOW);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Error al iniciar");
    return false;
  }
  esp_now_register_recv_cb(alRecibir);
  esp_now_register_send_cb(alEnviar);

  // Destino: MAC fija si está configurada; si no, broadcast hasta aprenderla.
  if (memcmp(MAC_PUENTE, MAC_BROADCAST, 6) != 0) {
    fijarPuente(MAC_PUENTE);
  } else {
    agregarPeer(MAC_BROADCAST);
    memcpy(macDestino, MAC_BROADCAST, 6);
    Serial.println("[ESP-NOW] Puente sin configurar: se aprenderá del primer paquete");
  }

  rmw_uros_set_custom_transport(
      true,      // framing XRCE (igual que el transporte serie del agente)
      NULL,
      trOpen, trClose, trWrite, trRead);
  return true;
}

const EstadisticasEspNow &transporteEstadisticas() { return est; }

uint32_t transporteMsDesdeUltimoRx() {
  return msUltimoRx == 0 ? UINT32_MAX : millis() - msUltimoRx;
}
