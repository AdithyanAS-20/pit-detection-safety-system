/**
 * ============================================================
 *  IR PIT DETECTOR BRIDGE  —  ESP32-S3  (I2C Master)
 * ============================================================
 *  Reads pit status from Arduino Nano (I2C slave 0x08).
 *  Stops motors via micro-ROS if ANY sensor sees no ground.
 *
 *  IR logic (same as Arduino):
 *    Sensor LOW  (0) = ground present = safe
 *    Sensor HIGH (1) = no ground      = PIT → STOP MOTORS
 *
 *  I2C pins:  SDA = GPIO8,  SCL = GPIO9
 *  micro-ROS: Serial (UART0 115200) → Raspberry Pi USB
 *
 *  Topics published:
 *    /pit_detected  [std_msgs/Bool]         — true = any pit
 *    /pit_status    [std_msgs/UInt8]         — bitmask FL/FR/BL/BR
 *    /cmd_vel_safe  [geometry_msgs/Twist]    — zero vel when pit
 *
 *  Topic subscribed:
 *    /pit_override  [std_msgs/Bool]          — operator override
 *
 *  Dependencies (platformio.ini):
 *    lib_deps = https://github.com/micro-ROS/micro_ros_arduino
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <geometry_msgs/msg/twist.h>

// ── I2C ──────────────────────────────────────────────────────
#define I2C_SDA_PIN       8
#define I2C_SCL_PIN       9
#define ARDUINO_ADDR      0x08
#define I2C_READ_BYTES    2
#define I2C_FREQ          100000UL

// ── Timing ───────────────────────────────────────────────────
#define POLL_MS           50       // poll Arduino every 50 ms
#define STOP_HOLD_MS    2000       // keep sending stop 2 s after danger clears
#define RECONNECT_MS     500

// ── Status byte bits ─────────────────────────────────────────
#define BIT_FL   0x01
#define BIT_FR   0x02
#define BIT_BL   0x04
#define BIT_BR   0x08
#define BIT_ANY  0x80   // set by Arduino if ANY sensor sees pit

// ── State ────────────────────────────────────────────────────
bool   g_override      = false;
bool   g_pit_active    = false;
unsigned long g_stop_until = 0;
unsigned long lastPoll  = 0;

// ── micro-ROS handles ─────────────────────────────────────────
rcl_node_t             node;
rcl_allocator_t        allocator;
rclc_support_t         support;
rclc_executor_t        executor;
rcl_publisher_t        pub_detected, pub_status, pub_cmd_vel;
rcl_subscription_t     sub_override;
std_msgs__msg__Bool      msg_detected, msg_override_in;
std_msgs__msg__UInt8     msg_status;
geometry_msgs__msg__Twist msg_stop;   // always zero

enum AgentState { WAIT, AVAILABLE, CONNECTED, DISCONNECTED };
AgentState agent = WAIT;

#define RCCHECK(fn)     { if((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { (void)(fn); }

// ── Helpers ───────────────────────────────────────────────────
void error_loop() {
  while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
}

/**
 * Read 2 bytes from Arduino Nano.
 * Byte 0 = status (bit7=ANY_DANGER, bits 0-3 = per sensor)
 * Byte 1 = raw sensor bitmask
 *
 * A sensor bit = 1 means that sensor saw NO object → pit/edge → stop.
 */
bool readArduino(uint8_t &status, uint8_t &raw) {
  uint8_t n = Wire.requestFrom((uint8_t)ARDUINO_ADDR, (uint8_t)I2C_READ_BYTES);
  if (n < I2C_READ_BYTES) { Wire.flush(); return false; }
  status = Wire.read();
  raw    = Wire.read();
  return true;
}

void publishStop() {
  // msg_stop is all zeros — zero linear, zero angular = full stop
  RCSOFTCHECK(rcl_publish(&pub_cmd_vel, &msg_stop, NULL));
}

void logSensors(uint8_t bitmask) {
  const char* names[4] = {"FL", "FR", "BL", "BR"};
  Serial.print("[PIT] Sensors with NO ground: ");
  for (uint8_t i = 0; i < 4; i++) {
    if (bitmask & (1 << i)) { Serial.print(names[i]); Serial.print(' '); }
  }
  Serial.println();
}

// ── Override callback ─────────────────────────────────────────
void override_cb(const void *msgin) {
  g_override = ((const std_msgs__msg__Bool *)msgin)->data;
  Serial.printf("[OVERRIDE] %s\n", g_override ? "ON (pit stop disabled)" : "OFF");
}

// ── micro-ROS lifecycle ───────────────────────────────────────
bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "pit_detector_node", "", &support) != RCL_RET_OK) return false;

  RCCHECK(rclc_publisher_init_default(&pub_detected, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),    "pit_detected"));
  RCCHECK(rclc_publisher_init_default(&pub_status,   &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),   "pit_status"));
  RCCHECK(rclc_publisher_init_default(&pub_cmd_vel,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_safe"));
  RCCHECK(rclc_subscription_init_default(&sub_override, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),    "pit_override"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_override,
    &msg_override_in, &override_cb, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_publisher_fini(&pub_detected, &node);
  rcl_publisher_fini(&pub_status,   &node);
  rcl_publisher_fini(&pub_cmd_vel,  &node);
  rcl_subscription_fini(&sub_override, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ── Pit detection ─────────────────────────────────────────────
void handlePitDetection() {
  unsigned long now = millis();
  if (now - lastPoll < POLL_MS) return;
  lastPoll = now;

  uint8_t status = 0, raw = 0;
  if (!readArduino(status, raw)) {
    Serial.println("[WARN] Arduino I2C read failed");
    return;
  }

  // Danger = ANY sensor did not detect ground (bit 7 set by Arduino)
  // AND operator hasn't overridden
  bool danger = (status & BIT_ANY) && !g_override;

  // Publish detection flag
  msg_detected.data = danger;
  RCSOFTCHECK(rcl_publish(&pub_detected, &msg_detected, NULL));

  // Publish per-sensor bitmask
  msg_status.data = raw;
  RCSOFTCHECK(rcl_publish(&pub_status, &msg_status, NULL));

  if (danger) {
    if (!g_pit_active) {
      g_pit_active = true;
      Serial.printf("[STOP] No ground detected! STATUS=0x%02X\n", status);
      logSensors(raw);
    }
    g_stop_until = now + STOP_HOLD_MS;
    publishStop();
  } else {
    if (g_pit_active) {
      g_pit_active = false;
      Serial.println("[SAFE] Ground detected on all sensors. Resuming after hold.");
    }
    // Hold stop for STOP_HOLD_MS after danger clears (safety buffer)
    if (now < g_stop_until) publishStop();
  }
}

// ── Setup / Loop ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQ);
  Serial.println("[ESP32-S3] I2C master on SDA=8 SCL=9");

  // msg_stop is zero-initialized by default — all Twist fields = 0.0
  // This is the full-stop command sent to /cmd_vel_safe

  set_microros_serial_transports(Serial);
  delay(2000);
}

void loop() {
  switch (agent) {
    case WAIT:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) agent = AVAILABLE;
      break;
    case AVAILABLE:
      if (create_entities()) {
        agent = CONNECTED;
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("[ROS2] Agent connected!");
      } else { destroy_entities(); agent = WAIT; }
      break;
    case CONNECTED:
      if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        agent = DISCONNECTED; digitalWrite(LED_BUILTIN, LOW);
        Serial.println("[WARN] Agent lost, reconnecting...");
        break;
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      handlePitDetection();
      break;
    case DISCONNECTED:
      destroy_entities();
      delay(RECONNECT_MS);
      agent = WAIT;
      break;
  }
}
