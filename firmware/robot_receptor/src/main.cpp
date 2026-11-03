/*
  ROBOT TREPADOR - ESP32 del robot (receptor) - nodo micro-ROS
  ------------------------------------------------------------
  Transporte: ESP-NOW <-> ESP puente (USB) <-> micro_ros_agent (PC)

  Nodo: /robot/robot_trepador

  SUSCRIPCIONES (órdenes desde el PC)
    /robot/esc       std_msgs/Int32              us para los ESC: 1000 = apagar, 1100..1700 = velocidad (con rampa; tope ESC_MAX)
    /robot/cmd_vel   geometry_msgs/Twist         linear.x (m/s) y angular.z (rad/s), lazo abierto
    /robot/ruedas    std_msgs/Float32MultiArray  [izq, der] de -1.0 a 1.0 (control directo del PWM)
    /robot/parada    std_msgs/Bool               true = parada de emergencia enclavada, false = liberar

  PUBLICACIONES (telemetría hacia el PC)
    /robot/imu       sensor_msgs/Imu             MPU6050: orientación, giro y aceleración (50 Hz)
    /robot/estado    std_msgs/String (JSON)      todo lo que el robot recibió y aplicó, y la batería (5 Hz)

  SEGURIDAD
    - Sin órdenes de /robot/esc durante 2 s      -> hélices apagadas (reenvía la orden periódicamente)
    - Sin órdenes de ruedas durante 0,5 s        -> ruedas frenadas
    - Se pierde el agente / el enlace ESP-NOW    -> todo apagado y reconexión automática

  Pines: ESC=18 | DRV8833 AIN1=25 AIN2=26 BIN1=33 BIN2=32 | MPU6050 SDA=21 SCL=22 | Batería=34
*/

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"
#include "transporte_espnow.h"
#include "actuadores.h"
#include "imu.h"
#include "bateria.h"

// ---------------- Utilidades ----------------
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { \
  Serial.printf("[micro-ROS] Error %d en línea %d\n", (int)rc, __LINE__); return false; } }
#define RCSOFT(fn)  { (void)(fn); }

#define CADA_MS(ms, ...) do { static uint32_t _t = 0; \
  if (millis() - _t >= (ms)) { _t = millis(); __VA_ARGS__; } } while (0)

// ---------------- Entidades micro-ROS ----------------
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

static rcl_subscription_t subEsc, subCmdVel, subRuedas, subParada;
static rcl_publisher_t    pubImu, pubEstado;

static std_msgs__msg__Int32              msgEsc;
static std_msgs__msg__Bool               msgParada;
static geometry_msgs__msg__Twist         msgCmdVel;
static std_msgs__msg__Float32MultiArray  msgRuedas;
static sensor_msgs__msg__Imu             msgImu;
static std_msgs__msg__String             msgEstado;

static float bufRuedas[4];
static char  bufEstado[640];

// Contadores de lo recibido (se reportan en /robot/estado)
static struct {
  uint32_t esc, cmdVel, ruedas, parada;
  int      ultimoEsc;
  float    ultimoV, ultimoW;
} rx = {};

enum EstadoAgente { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, AGENTE_CONECTADO, AGENTE_DESCONECTADO };
static EstadoAgente estadoAgente = ESPERANDO_AGENTE;
static uint8_t fallosPing = 0;
static uint32_t conexiones = 0;

// ---------------- Callbacks de suscripción ----------------
static void alRecibirEsc(const void *m) {
  const auto *msg = (const std_msgs__msg__Int32 *)m;
  rx.esc++;
  rx.ultimoEsc = msg->data;
  escOrden(msg->data);
}

static void alRecibirCmdVel(const void *m) {
  const auto *msg = (const geometry_msgs__msg__Twist *)m;
  rx.cmdVel++;
  rx.ultimoV = msg->linear.x;
  rx.ultimoW = msg->angular.z;
  float vIzq = msg->linear.x - msg->angular.z * ANCHO_VIA_M / 2.0f;
  float vDer = msg->linear.x + msg->angular.z * ANCHO_VIA_M / 2.0f;
  ruedasOrden(vIzq / V_MAX_MS, vDer / V_MAX_MS);
}

static void alRecibirRuedas(const void *m) {
  const auto *msg = (const std_msgs__msg__Float32MultiArray *)m;
  rx.ruedas++;
  if (msg->data.size >= 2) ruedasOrden(msg->data.data[0], msg->data.data[1]);
}

static void alRecibirParada(const void *m) {
  const auto *msg = (const std_msgs__msg__Bool *)m;
  rx.parada++;
  paradaEmergencia(msg->data);
}

// ---------------- Preparación de mensajes (memoria estática) ----------------
static void prepararMensajes() {
  msgRuedas.data.data     = bufRuedas;
  msgRuedas.data.capacity = sizeof(bufRuedas) / sizeof(bufRuedas[0]);
  msgRuedas.data.size     = 0;
  msgRuedas.layout.dim.data     = NULL;
  msgRuedas.layout.dim.capacity = 0;
  msgRuedas.layout.dim.size     = 0;

  msgEstado.data.data     = bufEstado;
  msgEstado.data.capacity = sizeof(bufEstado);
  msgEstado.data.size     = 0;

  static char frame[] = IMU_FRAME_ID;
  msgImu.header.frame_id.data     = frame;
  msgImu.header.frame_id.size     = strlen(frame);
  msgImu.header.frame_id.capacity = sizeof(frame);

  // Covarianzas (diagonales) aproximadas del MPU6050
  for (int i = 0; i < 9; i++) {
    msgImu.orientation_covariance[i] = 0;
    msgImu.angular_velocity_covariance[i] = 0;
    msgImu.linear_acceleration_covariance[i] = 0;
  }
  msgImu.orientation_covariance[0] = msgImu.orientation_covariance[4] = 0.0025;  // ~3°
  msgImu.orientation_covariance[8] = 0.1;                                        // yaw deriva
  msgImu.angular_velocity_covariance[0] = msgImu.angular_velocity_covariance[4] =
      msgImu.angular_velocity_covariance[8] = 0.0003;
  msgImu.linear_acceleration_covariance[0] = msgImu.linear_acceleration_covariance[4] =
      msgImu.linear_acceleration_covariance[8] = 0.01;
}

// ---------------- Crear / destruir entidades ----------------
static bool crearEntidades() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NOMBRE_NODO, ESPACIO_NOMBRES, &support));

  RCCHECK(rclc_subscription_init_default(&subEsc, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esc"));
  RCCHECK(rclc_subscription_init_default(&subCmdVel, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&subRuedas, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "ruedas"));
  RCCHECK(rclc_subscription_init_default(&subParada, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "parada"));

  RCCHECK(rclc_publisher_init_best_effort(&pubImu, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
  RCCHECK(rclc_publisher_init_default(&pubEstado, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "estado"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subEsc,    &msgEsc,    &alRecibirEsc,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subCmdVel, &msgCmdVel, &alRecibirCmdVel, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subRuedas, &msgRuedas, &alRecibirRuedas, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subParada, &msgParada, &alRecibirParada, ON_NEW_DATA));

  rmw_uros_sync_session(200);   // reloj del PC para los stamps
  return true;
}

static void destruirEntidades() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

  RCSOFT(rcl_subscription_fini(&subEsc, &node));
  RCSOFT(rcl_subscription_fini(&subCmdVel, &node));
  RCSOFT(rcl_subscription_fini(&subRuedas, &node));
  RCSOFT(rcl_subscription_fini(&subParada, &node));
  RCSOFT(rcl_publisher_fini(&pubImu, &node));
  RCSOFT(rcl_publisher_fini(&pubEstado, &node));
  RCSOFT(rclc_executor_fini(&executor));
  RCSOFT(rcl_node_fini(&node));
  RCSOFT(rclc_support_fini(&support));
}

// ---------------- Publicación ----------------
static void sellarTiempo(builtin_interfaces__msg__Time &t) {
  int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos()
                                              : (int64_t)millis() * 1000000LL;
  t.sec     = (int32_t)(ns / 1000000000LL);
  t.nanosec = (uint32_t)(ns % 1000000000LL);
}

static void publicarImu() {
  const DatosImu &d = imuDatos();
  if (!d.ok) return;
  sellarTiempo(msgImu.header.stamp);
  msgImu.orientation.w = d.qw;  msgImu.orientation.x = d.qx;
  msgImu.orientation.y = d.qy;  msgImu.orientation.z = d.qz;
  msgImu.angular_velocity.x = d.gx;
  msgImu.angular_velocity.y = d.gy;
  msgImu.angular_velocity.z = d.gz;
  msgImu.linear_acceleration.x = d.ax;
  msgImu.linear_acceleration.y = d.ay;
  msgImu.linear_acceleration.z = d.az;
  RCSOFT(rcl_publish(&pubImu, &msgImu, NULL));
}

static long msONeg(uint32_t ms) { return ms == UINT32_MAX ? -1 : (long)ms; }

static void publicarEstado() {
  const EstadoActuadores &a = actuadoresEstado();
  const DatosImu &d = imuDatos();
  const EstadisticasEspNow &e = transporteEstadisticas();
  const DatosBateria &b = bateriaDatos();

  int n = snprintf(bufEstado, sizeof(bufEstado),
    "{\"t\":%lu,\"parada\":%s,"
    "\"esc\":{\"ord\":%d,\"obj\":%d,\"act\":%d,\"ms\":%ld,\"n\":%lu},"
    "\"ruedas\":{\"izq\":%.2f,\"der\":%.2f,\"pwm_izq\":%d,\"pwm_der\":%d,\"ms\":%ld,"
    "\"v\":%.2f,\"w\":%.2f,\"n_vel\":%lu,\"n_dir\":%lu},"
    "\"imu\":{\"ok\":%s,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"temp\":%.1f,\"err\":%lu},"
    "\"enlace\":{\"rx\":%lu,\"tx\":%lu,\"fallos\":%lu,\"desc\":%lu,\"conex\":%lu},"
    "\"bat\":{\"ok\":%s,\"v\":%.2f,\"vc\":%.2f,\"pct\":%d,\"nivel\":%d,\"celdas\":%d}}",
    (unsigned long)millis(), a.paradaActiva ? "true" : "false",
    rx.ultimoEsc, a.escObjetivo, a.escActual, msONeg(a.msDesdeOrdenEsc), (unsigned long)rx.esc,
    a.ruedaIzq, a.ruedaDer, a.pwmIzq, a.pwmDer, msONeg(a.msDesdeOrdenRuedas),
    rx.ultimoV, rx.ultimoW, (unsigned long)rx.cmdVel, (unsigned long)rx.ruedas,
    d.ok ? "true" : "false", d.roll, d.pitch, d.yaw, d.temperatura, (unsigned long)d.errores,
    (unsigned long)e.paquetesRx, (unsigned long)e.paquetesTx, (unsigned long)e.fallosTx,
    (unsigned long)e.bytesDescartados, (unsigned long)conexiones,
    b.medida ? "true" : "false", b.voltios, b.vCelda, b.porcentaje, (int)b.nivel, BAT_CELDAS);

  if (n < 0) return;
  msgEstado.data.size = min((size_t)n, sizeof(bufEstado) - 1);
  RCSOFT(rcl_publish(&pubEstado, &msgEstado, NULL));
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);       // el USB del robot queda libre para depurar
  delay(200);
  Serial.println("\n=== Robot trepador - micro-ROS sobre ESP-NOW ===");

  actuadoresIniciar();        // ESC a 1000 us y ruedas frenadas desde el arranque
  imuIniciar();
  bateriaIniciar();
  prepararMensajes();

  if (!transporteRegistrar()) {
    Serial.println("[ERROR] No se pudo iniciar ESP-NOW. Reinicio en 3 s.");
    delay(3000);
    ESP.restart();
  }
  Serial.println("[micro-ROS] Buscando el agente...");
}

void loop() {
  imuActualizar();
  actuadoresActualizar();
  bateriaActualizar();

  switch (estadoAgente) {
    case ESPERANDO_AGENTE:
      CADA_MS(500, {
        estadoAgente = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENTE_DISPONIBLE
                                                                     : ESPERANDO_AGENTE;
      });
      break;

    case AGENTE_DISPONIBLE:
      if (crearEntidades()) {
        estadoAgente = AGENTE_CONECTADO;
        fallosPing = 0;
        conexiones++;
        Serial.println("[micro-ROS] Conectado al agente");
      } else {
        destruirEntidades();
        estadoAgente = ESPERANDO_AGENTE;
      }
      break;

    case AGENTE_CONECTADO:
      CADA_MS(1000, {
        if (rmw_uros_ping_agent(50, 2) == RMW_RET_OK) fallosPing = 0;
        else if (++fallosPing >= 3) estadoAgente = AGENTE_DESCONECTADO;
      });
      if (estadoAgente != AGENTE_CONECTADO) break;

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      CADA_MS(1000 / IMU_PUB_HZ,    publicarImu());
      CADA_MS(1000 / ESTADO_PUB_HZ, publicarEstado());
      CADA_MS(60000,                rmw_uros_sync_session(50));
      break;

    case AGENTE_DESCONECTADO:
      Serial.println("[micro-ROS] Agente perdido: todo apagado, reconectando...");
      pararTodo();
      destruirEntidades();
      estadoAgente = ESPERANDO_AGENTE;
      break;
  }

  // Depuración por USB cada 2 s
  CADA_MS(2000, {
    const EstadoActuadores &a = actuadoresEstado();
    const DatosImu &d = imuDatos();
    const DatosBateria &b = bateriaDatos();
    Serial.printf("agente=%d esc=%d/%d ruedas=%d/%d roll=%.1f pitch=%.1f parada=%d bat=%.2fV %d%%\n",
                  estadoAgente, a.escActual, a.escObjetivo, a.pwmIzq, a.pwmDer,
                  d.roll, d.pitch, a.paradaActiva, b.voltios, b.porcentaje);
  });
}
