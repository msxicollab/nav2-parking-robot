#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <car1_msgs/msg/odom_lite.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

static void promisc_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type);

// Wi-Fi and micro-ROS Agent settings (WIFI_SSID, WIFI_PASS, AGENT_IP) live in
// secrets.h, which is NOT committed to git. Copy secrets.example.h to
// secrets.h and fill in your own values.
#include "secrets.h"
#define AGENT_PORT 8888

// ── RAW UDP cmd_vel bypass ─────────────────────────────────────────────
#define RAW_CMD_PORT 9999
#define RAW_CMD_MAGIC 0xC1A1
#define RAW_CMD_VERSION 1
#define RAW_CMD_TIMEOUT_MS 250UL

struct __attribute__((packed)) RawCmdPacket {
  uint16_t magic;
  uint8_t  version;
  uint8_t  reserved;
  uint32_t seq;
  float    vx;
  float    omega;
  uint32_t sender_ms;
  uint16_t checksum;
};

WiFiUDP raw_cmd_udp;
static uint32_t raw_cmd_rx_count = 0;
static unsigned long raw_cmd_last_rx_ms = 0;
static unsigned long raw_cmd_max_gap_ms = 0;
static uint32_t raw_cmd_last_seq = 0;
static volatile bool raw_cmd_active = false;
static TaskHandle_t raw_cmd_task_handle = nullptr;
static portMUX_TYPE raw_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t raw_cmd_checksum(const uint8_t* b, size_t len) {
  uint16_t c = 0xA5A5;
  for (size_t i = 0; i < len; ++i) {
    c = (uint16_t)((c << 5) ^ (c >> 11) ^ b[i]);
  }
  return c;
}

#define MOT_L_IN1  21
#define MOT_L_IN2  47
#define MOT_L_ENA  20
#define MOT_R_IN3  12
#define MOT_R_IN4  13
#define MOT_R_ENB  14
#define ENC_L_A     45
#define ENC_L_B     48
#define ENC_R_A     4
#define ENC_R_B     5
#define SERVO_PIN  17
#define IMU_SDA    8
#define IMU_SCL    9
#define LIDAR_RX   39
#define LIDAR_TX   38
#define LIDAR_MCTR 40

#define PWM_FREQ  1000
#define PWM_RES      8

const float WHEEL_RADIUS   = 0.06525f;
const float TRACK_WIDTH    = 0.235f;
const float WHEELBASE      = 0.35f;
const int   PPR            = 655;
const float STEER_ODOM_GAIN = 1.63f;
const float RAD_PER_PULSE  = (2.0f * M_PI) / PPR;
const float DIST_PER_PULSE = RAD_PER_PULSE * WHEEL_RADIUS;
const float MAX_VEL        = 1.0f;
const float MAX_STEER      = 0.568f;

// ── IMU heading assist ─────────────────────────────────────────────
const float HEADING_KP = 0.0f;  // disabled for now; re-tune once Nav2 path
                                 // tracking interaction is confirmed clean
const float MAX_HEADING_CORRECTION = 0.15f;
const float FF_PWM_PER_MPS          = 255.0f / MAX_VEL;  // ~85, seed only
const float SPEED_PI_KP             = 150.0f;             // pwm per (m/s) error -- TUNE
const float SPEED_PI_KI             = 200.0f;              // pwm per (m/s*s) accumulated -- TUNE
const float SPEED_PI_INTEGRAL_CLAMP = 150.0f;              // anti-windup limit (pwm units)

static long          prev_enc_l_ctrl = 0, prev_enc_r_ctrl = 0;
static unsigned long last_ctrl_ms    = 0;
static float          integral_l = 0.0f, integral_r = 0.0f;   // PI state, per wheel
static float          meas_vl = 0.0f, meas_vr = 0.0f;         // latest measured wheel speeds (diag)

int SERVO_CENTER    = 80;
int SERVO_MAX_LEFT  = 125;
int SERVO_MAX_RIGHT = 30;

#define MPU_ADDR    0x68
#define ACCEL_XOUT  0x3B
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

#define LIDAR_BAUD     115200
#define LIDAR_MAX_PTS  110
#define LIDAR_SCAN_HZ  100
#define SCAN_INTERVAL  100UL
#define PUB_INTERVAL   100UL

// ── IMU state ────────────────────────────────────────────────────────
static float imu_ax = 0, imu_ay = 0, imu_az = 0;
static float imu_gx = 0, imu_gy = 0, imu_gz = 0;
static float gz_bias     = 0.0f;
static float accel_bias_x = 0.0f;
static float accel_bias_y = 0.0f;
static float gz_accum    = 0.0f;
static int   gz_count    = 0;
static float gz_idle_accum = 0.0f;
static int   gz_idle_count = 0;
static long  idle_enc_l_ref  = 0;
static long  idle_enc_r_ref  = 0;
static float idle_ax_min     = 1e9f, idle_ax_max = -1e9f;
static float idle_ay_min     = 1e9f, idle_ay_max = -1e9f;
static bool  idle_window_open = false;
const  float IDLE_ACCEL_BAND  = 0.15f;
static float vx_filtered   = 0.0f;
static float vyaw_filtered = 0.0f;
const  float ODOM_FILTER_ALPHA = 1.0f;
static int64_t epoch_offset_ms = 0;

// ── Encoder ──────────────────────────────────────────────────────────
volatile long enc_l = 0, enc_r = 0;
long prev_enc_l = 0, prev_enc_r = 0;
unsigned long prev_ms = 0;

// ── LiDAR ────────────────────────────────────────────────────────────
static float lidar_shadow[LIDAR_MAX_PTS];
portMUX_TYPE lidar_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t pkt_buf[512];
static int     pkt_len      = 0;
static bool    pkt_sync     = false;
static uint8_t prev_byte    = 0;
static int     pkt_expected = 0;

// Raw YDLIDAR packet for PC (published on /lidar_raw)
static uint8_t lidar_raw_shadow[512];
static int     lidar_raw_len = 0;

// ── Timing / diag ────────────────────────────────────────────────────
static uint32_t      cnt_bytes = 0, cnt_ok = 0, cnt_bad = 0, cnt_pub = 0;
static unsigned long max_loop_us = 0;
static unsigned long diag_ms       = 0;
static unsigned long last_scan_pub = 0;
static unsigned long last_imu_read = 0;
static unsigned long last_odom_pub = 0;
static unsigned long last_imu_pub  = 0;
static unsigned long last_cmd_ms   = 0;
static unsigned long last_apply_ms = 0;
static unsigned long last_resync_ms = 0;
static bool was_connected = false;

static uint32_t      cmd_rx_count   = 0;
static unsigned long max_cmd_gap_ms = 0;
static unsigned long last_cmd_rx_ms = 0;
static unsigned long spin_sum = 0, spin_max = 0;
static uint32_t      spin_samples = 0;

// ── RF-level sniffer counters (promiscuous mode) ───────────────────────
// data_to_us: any data frame addressed to our own MAC (addr1) — accurate
//   regardless of WPA2/3 since the MAC header itself isn't encrypted.
// udp8888_to_us: subset of the above that parses as IPv4/UDP to port 8888.
//   May under-count if a frame doesn't match the parser's assumptions —
//   treat data_to_us as the primary reference if this looks too low.
// Accessed from the WiFi driver task (not a true ISR); plain uint32_t
// increments are fine for a diagnostic counter.
static volatile uint32_t promisc_data_to_us    = 0;
static volatile uint32_t promisc_udp8888_to_us = 0;
static uint8_t my_mac[6];

static int agent_ping_fail_count = 0;
const  int AGENT_PING_FAIL_THRESHOLD = 3;  // consecutive fails (~3s) before treating the agent as truly gone

static bool estop_active = false;

const  unsigned long PING_SUPPRESS_ACTIVE_MS = 500;
static unsigned long last_ping_attempt_ms    = 0;  // diagnostics only

// ── Motion ───────────────────────────────────────────────────────────
static float last_vx    = 0.0f;
static float last_omega = 0.0f;
static float ros_last_vx = 0.0f;
static float ros_last_omega = 0.0f;
static float applied_vx    = 0.0f;
static float applied_omega = 0.0f;
static float pwm_l_current = 0.0f;
static float pwm_r_current = 0.0f;
const float PWM_RAMP_UP   = 22.0f;
const float PWM_RAMP_DOWN = 25.0f;
const  float VX_RAMP       = 0.1f;
const  float OMEGA_RAMP    = 0.06f;
const  float STEER_RAMP_RATE = 4.0f;
float current_steer = 0.0f;
static float heading_correction = 0.0f;

TaskHandle_t lidar_task_handle;

TaskHandle_t ros_spin_task_handle;
static SemaphoreHandle_t ros_mutex;
static volatile bool entities_ready = false;   // spin_task checks this before touching the executor

// ── ROS ──────────────────────────────────────────────────────────────
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
states state = WAITING_AGENT;

rcl_node_t         node;
rcl_publisher_t    joint_pub, imu_pub, scan_pub, odom_lite_pub, lidar_raw_pub;
rcl_subscription_t cmd_sub;
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;

sensor_msgs__msg__JointState joint_msg;
sensor_msgs__msg__Imu        imu_msg;
sensor_msgs__msg__LaserScan  scan_msg;
std_msgs__msg__UInt8MultiArray lidar_raw_msg;
car1_msgs__msg__OdomLite     odom_lite_msg;
geometry_msgs__msg__Twist    cmd_msg;

static double joint_pos[6] = {0,0,0,0,0,0};
static double joint_vel[6] = {0,0,0,0,0,0};
static double joint_eff[6] = {0,0,0,0,0,0};
static rosidl_runtime_c__String joint_names[6];

Servo steer_servo;

static bool epoch_offset_initialized = false;

// NOTE: called from two places:
//   1) create_entities(), called from AGENT_AVAILABLE where the caller
//      (loop()) already holds ros_mutex — do NOT take it again here or it
//      will deadlock (the mutex is non-recursive).
//   2) the periodic resync in AGENT_CONNECTED, where the caller likewise
//      wraps ros_mutex around this call itself.
// So this function never takes the mutex itself — each call site handles it.
void sync_epoch_offset() {
  unsigned long t0 = millis();
  if (rmw_uros_sync_session(150) == RMW_RET_OK) {
    unsigned long t1 = millis();
    int64_t mid = (t0 + t1) / 2;
    int64_t new_offset = rmw_uros_epoch_millis() - (int64_t)mid;

    if (epoch_offset_initialized && llabs(new_offset - epoch_offset_ms) > 300) {
      Serial.printf("[SYNC] rejected jump: old=%lld new=%lld diff=%lld ms\n",
                    epoch_offset_ms, new_offset, new_offset - epoch_offset_ms);
      return;
    }

    epoch_offset_ms = new_offset;
    epoch_offset_initialized = true;
    Serial.printf("[SYNC] offset=%lld  rtt=%lu ms\n", epoch_offset_ms, t1 - t0);
  } else {
    Serial.println("[SYNC] failed, keeping previous offset");
  }
}

// ── ISR ──────────────────────────────────────────────────────────────

void IRAM_ATTR enc_l_isr() {
  enc_r += (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) ? 1 : -1;
}
void IRAM_ATTR enc_r_isr() {
  enc_l += (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) ? -1 : 1;
}

// ── Motor ─────────────────────────────────────────────────────────────
void motors_init() {
  ledcAttach(MOT_L_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(MOT_R_ENB, PWM_FREQ, PWM_RES);
  ledcWrite(MOT_L_ENA, 0);
  ledcWrite(MOT_R_ENB, 0);
}

// ── Closed-loop PI speed control ────────────────────────────────────
// Returns a signed PWM value in [-255,255] for one wheel, given its
// commanded speed, most recently measured speed, and the persistent PI
// integral state for that wheel (owned by the caller so left/right stay
// independent). Combines a linear feedforward seed with PI correction so
// it converges faster than pure PI from zero, while the PI term corrects
// whatever the feedforward's linear assumption gets wrong -- including
// the low-PWM/breakaway behavior that the old hard floor always hid.
float pi_speed_to_pwm(float target_v, float measured_v, float &integral, float dt) {
  if (fabsf(target_v) < 0.01f) {
    // Commanded to stop -- don't let integral wind up while stationary.
    integral = 0.0f;
    return 0.0f;
  }
  if (dt <= 0.0f) return constrain(integral, -255.0f, 255.0f);  // guard bad dt

  float error = target_v - measured_v;
  float ff    = target_v * FF_PWM_PER_MPS;

  // Basic anti-windup: only accept the integration step if the resulting
  // PI contribution would still be within its clamp.
  float candidate_integral = integral + error * dt;
  float pi_term_candidate  = SPEED_PI_KI * candidate_integral;
  if (fabsf(pi_term_candidate) <= SPEED_PI_INTEGRAL_CLAMP) {
    integral = candidate_integral;
  }
  float pi_term = constrain(SPEED_PI_KI * integral, -SPEED_PI_INTEGRAL_CLAMP, SPEED_PI_INTEGRAL_CLAMP);

  float output = ff + SPEED_PI_KP * error + pi_term;
  return constrain(output, -255.0f, 255.0f);
}

void set_motor_l(float target_v, float measured_v, float dt) {
  target_v = constrain(target_v, -MAX_VEL, MAX_VEL);
  float target_pwm = pi_speed_to_pwm(target_v, measured_v, integral_l, dt);

  float diff = target_pwm - pwm_l_current;
  if (fabsf(diff) <= PWM_RAMP_UP) {
    pwm_l_current = target_pwm;
  } else {
    float step = (fabsf(target_pwm) > fabsf(pwm_l_current)) ? PWM_RAMP_UP : PWM_RAMP_DOWN;
    pwm_l_current += constrain(diff, -step, step);
  }

  if (fabsf(pwm_l_current) < 1.0f) {
    pwm_l_current = 0.0f;
    digitalWrite(MOT_L_IN1, HIGH);
    digitalWrite(MOT_L_IN2, HIGH);
    ledcWrite(MOT_L_ENA, 255);
    return;
  }

  bool forward = pwm_l_current > 0.0f;
  digitalWrite(MOT_L_IN1, forward ? HIGH : LOW);
  digitalWrite(MOT_L_IN2, forward ? LOW : HIGH);

  int p = constrain((int)fabsf(pwm_l_current), 0, 255);
  // No hard min_pwm floor anymore -- the PI integral term (in
  // pi_speed_to_pwm) is what gets the wheel moving from a standstill,
  // winding up until it actually breaks away, instead of an artificial
  // floor that always forced ~2 m/s regardless of the requested speed.
  ledcWrite(MOT_L_ENA, p);
}

void set_motor_r(float target_v, float measured_v, float dt) {
  target_v = constrain(target_v, -MAX_VEL, MAX_VEL);
  float target_pwm = pi_speed_to_pwm(target_v, measured_v, integral_r, dt);

  float diff = target_pwm - pwm_r_current;
  if (fabsf(diff) <= PWM_RAMP_UP) {
    pwm_r_current = target_pwm;
  } else {
    float step = (fabsf(target_pwm) > fabsf(pwm_r_current)) ? PWM_RAMP_UP : PWM_RAMP_DOWN;
    pwm_r_current += constrain(diff, -step, step);
  }

  if (fabsf(pwm_r_current) < 1.0f) {
    pwm_r_current = 0.0f;
    digitalWrite(MOT_R_IN3, HIGH);
    digitalWrite(MOT_R_IN4, HIGH);
    ledcWrite(MOT_R_ENB, 255);
    return;
  }

  bool forward = pwm_r_current > 0.0f;
  digitalWrite(MOT_R_IN3, forward ? HIGH : LOW);
  digitalWrite(MOT_R_IN4, forward ? LOW : HIGH);

  int p = constrain((int)fabsf(pwm_r_current), 0, 255);
  // No hard min_pwm floor anymore -- see set_motor_l() comment above.
  ledcWrite(MOT_R_ENB, p);
}

void set_steering(float rad) {
  rad = constrain(rad, -MAX_STEER, MAX_STEER);
  float hr  = (SERVO_MAX_LEFT - SERVO_MAX_RIGHT) / 2.0f;
  int   deg = SERVO_CENTER + (int)(rad / MAX_STEER * hr);
  steer_servo.write(constrain(deg, SERVO_MAX_RIGHT, SERVO_MAX_LEFT));
}

// ── Emergency stop ───────────────────────────────────────────────────
void emergency_stop_motors() {
  pwm_l_current = 0.0f;
  pwm_r_current = 0.0f;
  integral_l = 0.0f;
  integral_r = 0.0f;
  digitalWrite(MOT_L_IN1, HIGH);
  digitalWrite(MOT_L_IN2, HIGH);
  ledcWrite(MOT_L_ENA, 255);
  digitalWrite(MOT_R_IN3, HIGH);
  digitalWrite(MOT_R_IN4, HIGH);
  ledcWrite(MOT_R_ENB, 255);
  current_steer = 0.0f;
  set_steering(0.0f);
}

// ── MPU6050 ───────────────────────────────────────────────────────────
void mpu_init() {
  Wire.setTimeOut(20);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
}

void mpu_read(float* ax, float* ay, float* az,
              float* gx, float* gy, float* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  int16_t a[3], g[3];
  for (int i = 0; i < 3; i++) a[i] = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  for (int i = 0; i < 3; i++) g[i] = (Wire.read() << 8) | Wire.read();
  *ax = -a[0] / ACCEL_SCALE * 9.81f;
  *ay = -a[1] / ACCEL_SCALE * 9.81f;
  *az = a[2] / ACCEL_SCALE * 9.81f;
  *gx = g[0] / GYRO_SCALE * (M_PI / 180.f);
  *gy = -g[1] / GYRO_SCALE * (M_PI / 180.f);
  *gz = g[2] / GYRO_SCALE * (M_PI / 180.f);
}

// ── LiDAR ─────────────────────────────────────────────────────────────
bool lidar_checksum_ok(uint8_t* buf, int len) {
  uint8_t qty    = buf[3];
  int     needed = 10 + qty * 2;
  if (len < needed) return false;
  uint16_t cs_recv = (uint16_t)(buf[8] | (buf[9] << 8));
  uint16_t cs_calc = 0x55AA;
  cs_calc ^= (uint16_t)(buf[2] | (buf[3] << 8));
  cs_calc ^= (uint16_t)(buf[4] | (buf[5] << 8));
  cs_calc ^= (uint16_t)(buf[6] | (buf[7] << 8));
  for (int i = 10; i < needed; i += 2)
    cs_calc ^= (uint16_t)(buf[i] | (buf[i + 1] << 8));
  return cs_calc == cs_recv;
}

void lidar_parse_pkt(uint8_t* buf, int len) {
  uint8_t ct  = buf[2];
  uint8_t qty = buf[3];
  if (ct & 0x01) { cnt_ok++; return; }
  if (qty == 0)  { cnt_bad++; return; }
  if (!lidar_checksum_ok(buf, len)) { cnt_bad++; return; }
  cnt_ok++;

  if (len <= (int)sizeof(lidar_raw_shadow)) {
    portENTER_CRITICAL(&lidar_mux);
    memcpy(lidar_raw_shadow, buf, len);
    lidar_raw_len = len;
    portEXIT_CRITICAL(&lidar_mux);
  }

  float sa_deg = ((uint16_t)(buf[4] | (buf[5] << 8)) >> 1) / 64.0f;
  float ea_deg = ((uint16_t)(buf[6] | (buf[7] << 8)) >> 1) / 64.0f;
  if (ea_deg < sa_deg) ea_deg += 360.0f;
  float step = (qty > 1) ? (ea_deg - sa_deg) / (qty - 1) : 0.0f;

  portENTER_CRITICAL(&lidar_mux);
  for (int i = 0; i < qty; i++) {
    int bi = 10 + i * 2;
    if (bi + 1 >= len) break;
    uint16_t d_raw  = (uint16_t)(buf[bi] | (buf[bi + 1] << 8));
    float    dist_m = (d_raw >> 2) / 1000.0f;
    if (dist_m < 0.1f || dist_m > 12.0f) dist_m = INFINITY;
    float angle      = fmodf(sa_deg + step * i, 360.0f);
    float angle_flip = fmodf(360.0f - angle + 180.0f, 360.0f);;
    int   idx        = (int)(angle_flip / 360.0f * LIDAR_MAX_PTS) % LIDAR_MAX_PTS;
    lidar_shadow[idx] = dist_m;
  }
  portEXIT_CRITICAL(&lidar_mux);
}

void lidar_update() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    cnt_bytes++;
    if (!pkt_sync) {
      if (prev_byte == 0xAA && b == 0x55) {
        pkt_buf[0] = 0xAA; pkt_buf[1] = 0x55;
        pkt_len = 2; pkt_expected = 0; pkt_sync = true;
      }
      prev_byte = b;
      continue;
    }
    if (pkt_len < (int)sizeof(pkt_buf)) {
      pkt_buf[pkt_len++] = b;
    } else {
      cnt_bad++;
      pkt_sync = false; pkt_len = 0; pkt_expected = 0; prev_byte = 0;
      continue;
    }
    if (pkt_len == 4) {
      uint8_t qty = pkt_buf[3];
      if (qty == 0 || qty > 200) {
        cnt_bad++;
        pkt_sync = false; pkt_len = 0; prev_byte = 0;
        continue;
      }
      pkt_expected = 10 + qty * 2;
    }
    if (pkt_expected > 0 && pkt_len >= pkt_expected) {
      lidar_parse_pkt(pkt_buf, pkt_expected);
      pkt_sync = false; pkt_len = 0; pkt_expected = 0; prev_byte = 0;
    }
  }
}

void lidar_task(void* param) {
  for (;;) { lidar_update(); vTaskDelay(1); }
}

// ── ros_spin_task ────────────────────────────────────────────────────
// Delay is dynamic: 1ms while driving (cmd_vel/UDP seen within
// PING_SUPPRESS_ACTIVE_MS, via last_cmd_rx_ms — same signal ping uses), so
// the task gets more chances to grab ros_mutex when it matters most; 2ms
// otherwise.
void ros_spin_task(void* param) {
  for (;;) {
    if (entities_ready) {
      if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        unsigned long t0 = micros();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
        unsigned long dur = micros() - t0;
        spin_sum += dur;
        spin_samples++;
        if (dur > spin_max) spin_max = dur;
        xSemaphoreGive(ros_mutex);
      }
    }
    bool driving_now = raw_cmd_active ||
                        ((last_cmd_rx_ms > 0) &&
                         (millis() - last_cmd_rx_ms < PING_SUPPRESS_ACTIVE_MS));
    vTaskDelay(driving_now ? 1 : 2);
  }
}

// ── Ackermann drive ───────────────────────────────────────────────────
void apply_ackermann(float vx, float steer_rad, float mvl, float mvr, float dt) {
  if (fabsf(vx) < 0.01f && fabsf(steer_rad) < 0.001f) {
    set_motor_l(0, mvl, dt); set_motor_r(0, mvr, dt); return;
  }
  if (fabsf(steer_rad) < 0.001f) {
    set_motor_l(vx, mvl, dt); set_motor_r(vx, mvr, dt); return;
  }
  float R   = WHEELBASE / tanf(fabsf(steer_rad));
  float v_l = vx * (R - TRACK_WIDTH * 0.5f) / R;
  float v_r = vx * (R + TRACK_WIDTH * 0.5f) / R;
  if (steer_rad < 0) { float tmp = v_l; v_l = v_r; v_r = tmp; }
  set_motor_l(v_l, mvl, dt);
  set_motor_r(v_r, mvr, dt);
}

// ── cmd_vel callback ──────────────────────────────────────────────────
void cmd_cb(const void* msg_in) {
  auto* cmd = (const geometry_msgs__msg__Twist*)msg_in;

  unsigned long now = millis();
  if (last_cmd_rx_ms > 0) {
    unsigned long gap = now - last_cmd_rx_ms;
    if (gap > max_cmd_gap_ms) max_cmd_gap_ms = gap;
  }
  last_cmd_rx_ms = now;
  cmd_rx_count++;

  // ROS /cmd_vel is diagnostic/backup only in this build.
  // Do NOT let the slow micro-ROS callback overwrite the UDP motor command.
  ros_last_vx    = (float)cmd->linear.x;
  ros_last_omega = (float)cmd->angular.z;
}

// ── RAW UDP command receiver ─────────────────────────────────────────
// Reads all queued packets and keeps only the newest valid command.
// This runs in its own FreeRTOS task independently of the micro-ROS executor.
void raw_cmd_task(void* param) {
  for (;;) {
    int packet_size;
    while ((packet_size = raw_cmd_udp.parsePacket()) > 0) {
      if (packet_size != (int)sizeof(RawCmdPacket)) {
        while (raw_cmd_udp.available()) raw_cmd_udp.read();
        continue;
      }

      RawCmdPacket p;
      int n = raw_cmd_udp.read((uint8_t*)&p, sizeof(p));
      if (n != (int)sizeof(p)) continue;
      if (p.magic != RAW_CMD_MAGIC || p.version != RAW_CMD_VERSION) continue;
      if (p.checksum != raw_cmd_checksum(
          reinterpret_cast<const uint8_t*>(&p),
          sizeof(RawCmdPacket) - sizeof(uint16_t))) continue;
      if (!isfinite(p.vx) || !isfinite(p.omega)) continue;
      if (fabsf(p.vx) > MAX_VEL || fabsf(p.omega) > 20.0f) continue;

      unsigned long now = millis();
      portENTER_CRITICAL(&raw_cmd_mux);
      if (raw_cmd_last_rx_ms > 0) {
        unsigned long gap = now - raw_cmd_last_rx_ms;
        if (gap > raw_cmd_max_gap_ms) raw_cmd_max_gap_ms = gap;
      }
      raw_cmd_last_rx_ms = now;
      raw_cmd_last_seq = p.seq;
      raw_cmd_rx_count++;
      last_vx = p.vx;
      last_omega = p.omega;
      last_cmd_ms = now;
      raw_cmd_active = true;
      portEXIT_CRITICAL(&raw_cmd_mux);
    }

    // Separate UDP safety timeout. Bridge sends at 30Hz, so 250ms tolerates
    // a few lost packets but still stops the car quickly if the bridge dies.
    unsigned long now = millis();
    portENTER_CRITICAL(&raw_cmd_mux);
    if (raw_cmd_active && (now - raw_cmd_last_rx_ms > RAW_CMD_TIMEOUT_MS)) {
      raw_cmd_active = false;
      last_vx = 0.0f;
      last_omega = 0.0f;
      last_cmd_ms = 0;
    }
    portEXIT_CRITICAL(&raw_cmd_mux);

    vTaskDelay(1);
  }
}

// ── ROS entity lifecycle ────────────────────────────────────────────
bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;

  epoch_offset_initialized = false;
  sync_epoch_offset();

  if (rclc_node_init_default(&node, "car1_esp32", "", &support) != RCL_RET_OK) return false;

  rclc_publisher_init_best_effort(&odom_lite_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(car1_msgs, msg, OdomLite), "/odom_lite");
  rclc_publisher_init_best_effort(&joint_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states");
  rclc_publisher_init_best_effort(&imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");
  rclc_publisher_init_best_effort(&scan_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  rclc_publisher_init_best_effort(&lidar_raw_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "/lidar_raw");
  rclc_subscription_init_best_effort(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_cb, ON_NEW_DATA);

  Serial.println("[ROS] entities created");
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&odom_lite_pub, &node);
  rcl_publisher_fini(&joint_pub, &node);
  rcl_publisher_fini(&imu_pub, &node);
  rcl_publisher_fini(&scan_pub, &node);
  rcl_publisher_fini(&lidar_raw_pub, &node);
  rcl_subscription_fini(&cmd_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  Serial.println("[ROS] entities destroyed");
}

// ── RF-level sniffer (promiscuous mode) ─────────────────────────────────
// Listens for 802.11 data frames addressed to our own MAC (addr1 == my_mac)
// to measure how many packets actually reach the ESP32 antenna per second,
// compared against cmd_rx_count (counted when the ROS callback actually
// fires). The MAC header (addr1/addr2/addr3 etc.) is never encrypted even
// under WPA2/3, so filtering on addr1 is always reliable; parsing further
// into LLC/IP/UDP only succeeds for traffic within our own BSS (which the
// hardware already decrypts before handing it to this callback, since we
// hold our own session key). If parsing fails, only data_to_us increments —
// treat that as the primary reference in that case.
typedef struct {
  uint16_t frame_ctrl;
  uint16_t duration;
  uint8_t  addr1[6];
  uint8_t  addr2[6];
  uint8_t  addr3[6];
  uint16_t seq_ctrl;
} __attribute__((packed)) wifi_ieee80211_mac_hdr_t;

static void promisc_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;

  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  int len = pkt->rx_ctrl.sig_len;
  if (len < (int)sizeof(wifi_ieee80211_mac_hdr_t)) return;

  const uint8_t* p = pkt->payload;

  // addr1 = receiver address, at byte offset 4 (after 2-byte frame_ctrl +
  // 2-byte duration) — must match our own MAC.
  if (memcmp(p + 4, my_mac, 6) != 0) return;

  promisc_data_to_us++;

  // Find where the payload starts after the MAC header: base header is 24
  // bytes; QoS Data frames add 2 bytes of QoS Control; ToDS&&FromDS (WDS,
  // rare for a normal STA) adds a 6-byte addr4.
  uint8_t fc0 = p[0];
  uint8_t fc1 = p[1];
  uint8_t subtype = (fc0 >> 4) & 0x0F;
  bool to_ds   = (fc1 & 0x01) != 0;
  bool from_ds = (fc1 & 0x02) != 0;
  bool is_qos  = (subtype & 0x08) != 0;

  int hdr_len = 24;
  if (to_ds && from_ds) hdr_len += 6;
  if (is_qos) hdr_len += 2;

  if (len < hdr_len + 8 + 20 + 8) return;  // need room for LLC/SNAP + IP + UDP headers

  const uint8_t* llc = p + hdr_len;
  // Standard SNAP header for IPv4: AA AA 03 00 00 00 08 00
  if (llc[0] != 0xAA || llc[1] != 0xAA || llc[2] != 0x03) return;
  uint16_t ethertype = (llc[6] << 8) | llc[7];
  if (ethertype != 0x0800) return;

  const uint8_t* ip = llc + 8;
  uint8_t ver_ihl = ip[0];
  if ((ver_ihl >> 4) != 4) return;
  int ip_hdr_len = (ver_ihl & 0x0F) * 4;
  uint8_t proto = ip[9];
  if (proto != 17) return;

  const uint8_t* udp = ip + ip_hdr_len;
  if ((int)(udp - p) + 4 > len) return;
  uint16_t dst_port = (udp[2] << 8) | udp[3];
  if (dst_port == AGENT_PORT) {
    promisc_udp8888_to_us++;
  }
}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("[car1] v41-FIX9 (raw UDP cmd_vel bypass) boot");

  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__JointState__init(&joint_msg);
  car1_msgs__msg__OdomLite__init(&odom_lite_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);
  std_msgs__msg__UInt8MultiArray__init(&lidar_raw_msg);

  memset(&scan_msg, 0, sizeof(scan_msg));
  scan_msg.ranges.data     = (float*)malloc(sizeof(float) * LIDAR_MAX_PTS);
  scan_msg.ranges.size     = LIDAR_MAX_PTS;
  scan_msg.ranges.capacity = LIDAR_MAX_PTS;
  scan_msg.intensities.data     = nullptr;
  scan_msg.intensities.size     = 0;
  scan_msg.intensities.capacity = 0;

  lidar_raw_msg.data.data = lidar_raw_shadow;
  lidar_raw_msg.data.size = 0;
  lidar_raw_msg.data.capacity = sizeof(lidar_raw_shadow);

  for (int i = 0; i < LIDAR_MAX_PTS; i++) {
    scan_msg.ranges.data[i] = INFINITY;
    lidar_shadow[i]         = INFINITY;
  }

  pinMode(MOT_L_IN1, OUTPUT); pinMode(MOT_L_IN2, OUTPUT);
  pinMode(MOT_L_ENA, OUTPUT); pinMode(MOT_R_IN3, OUTPUT);
  pinMode(MOT_R_IN4, OUTPUT); pinMode(MOT_R_ENB, OUTPUT);
  for (auto p : {ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B}) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_l_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_r_isr, CHANGE);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  steer_servo.setPeriodHertz(50);
  steer_servo.attach(SERVO_PIN, 500, 2500);
  steer_servo.write(SERVO_CENTER);
  motors_init();

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  mpu_init();

  Serial.println("[CAL] warming up IMU...");
  delay(3000);
  Serial.println("[CAL] calibrating...");
  {
    float s_gz = 0, s_ax = 0, s_ay = 0;
    float ax, ay, az, gx, gy, gz;
    for (int i = 0; i < 1000; i++) {
      mpu_read(&ax, &ay, &az, &gx, &gy, &gz);
      s_gz += gz; s_ax += ax; s_ay += ay;
      delay(5);
    }
    gz_bias      = s_gz / 1000.0f;
    accel_bias_x = s_ax / 1000.0f;
    accel_bias_y = s_ay / 1000.0f;
    Serial.printf("[CAL] gz_bias=%.5f  ax_bias=%.4f  ay_bias=%.4f\n",
                  gz_bias, accel_bias_x, accel_bias_y);
  }

  pinMode(LIDAR_MCTR, OUTPUT);
  digitalWrite(LIDAR_MCTR, HIGH);
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  delay(1500);

  // lidar_task: reads/parses the LiDAR UART stream on core 0.
  xTaskCreatePinnedToCore(lidar_task, "lidar", 4096, NULL, 1,
                        &lidar_task_handle, 0);
  Serial.println("[car1] lidar_task created — /scan and /lidar_raw active");

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  delay(5000);

  // RF-level sniffer: channel locks to whatever AP the STA is already
  // connected to, so no need to set it manually. Runs alongside the normal
  // STA connection without disrupting it.
  WiFi.macAddress(my_mac);
  {
    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&promisc_rx_cb);
    esp_wifi_set_promiscuous(true);
  }
  Serial.printf("[car1] promiscuous sniffer armed, my_mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
    my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);

  // Raw UDP motor-command bypass.
  raw_cmd_udp.begin(RAW_CMD_PORT);
  Serial.printf("[UDP] raw cmd listener ready on port %d, local_ip=%s, timeout=%lums\n",
                RAW_CMD_PORT, WiFi.localIP().toString().c_str(), RAW_CMD_TIMEOUT_MS);

  // Dedicated FreeRTOS task: UDP receive is NOT dependent on the Arduino loop
  // or the micro-ROS executor, so a slow rcl_publish()/ping cannot starve it.
  xTaskCreatePinnedToCore(raw_cmd_task, "raw_cmd", 4096, NULL, 3,
                          &raw_cmd_task_handle, 1);

  for (int i = 0; i < 6; i++) rosidl_runtime_c__String__init(&joint_names[i]);
  rosidl_runtime_c__String__assign(&joint_names[0], "RLwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[1], "RRwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[2], "FLwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[3], "FRwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[4], "steeringFL_joint");
  rosidl_runtime_c__String__assign(&joint_names[5], "steeringFR_joint");
  joint_msg.name.data = joint_names;
  joint_msg.name.size = joint_msg.name.capacity = 6;
  joint_msg.position.data = joint_pos;
  joint_msg.position.size = joint_msg.position.capacity = 6;
  joint_msg.velocity.data = joint_vel;
  joint_msg.velocity.size = joint_msg.velocity.capacity = 6;
  joint_msg.effort.data   = joint_eff;
  joint_msg.effort.size   = joint_msg.effort.capacity   = 6;

  static char f_base[]  = "base_link";
  static char f_laser[] = "laser_frame";
  static char f_imu[]   = "imu_link";

  joint_msg.header.frame_id = {f_base,  strlen(f_base),  sizeof(f_base)};
  scan_msg.header.frame_id  = {f_laser, strlen(f_laser), sizeof(f_laser)};
  imu_msg.header.frame_id   = {f_imu,   strlen(f_imu),   sizeof(f_imu)};

  scan_msg.angle_min       = 0.0f;
  scan_msg.angle_max       = 2.0f * M_PI;
  scan_msg.angle_increment = 2.0f * M_PI / LIDAR_MAX_PTS;
  scan_msg.time_increment  = (1.0f / LIDAR_SCAN_HZ) / LIDAR_MAX_PTS;
  scan_msg.scan_time       = 1.0f / LIDAR_SCAN_HZ;
  scan_msg.range_min       = 0.1f;
  scan_msg.range_max       = 12.0f;

  imu_msg.orientation_covariance[0]         = -1.0;
  imu_msg.angular_velocity_covariance[0]    = 0.002;
  imu_msg.angular_velocity_covariance[4]    = 0.002;
  imu_msg.angular_velocity_covariance[8]    = 0.002;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  ros_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ros_spin_task, "ros_spin", 8192, NULL, 2,
                      &ros_spin_task_handle, 1);

  delay(1000);
  diag_ms       = millis();
  prev_ms       = millis();
  last_imu_read = millis();
  last_imu_pub  = millis();
  last_odom_pub = millis() + 33;
  last_scan_pub = millis() + 66;
  Serial.printf("[car1] v41-FIX9 ready (raw UDP cmd_vel bypass)  heap=%d\n", ESP.getFreeHeap());
}

#define CHECKPOINT(name) { \
  unsigned long now__ = micros(); \
  unsigned long delta__ = now__ - t_prev__; \
  if (delta__ > 50000) { \
    Serial.printf("[SLOW] %-15s took %6lu us  RSSI=%d\n", name, delta__, WiFi.RSSI()); \
  } \
  t_prev__ = now__; \
}

// ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long loop_t0 = micros();
  unsigned long t_prev__ = loop_t0;

  bool now_connected = (WiFi.status() == WL_CONNECTED);
  if (!was_connected && now_connected)
    Serial.println("[WIFI] reconnected");
  was_connected = now_connected;

  switch (state) {

    case WAITING_AGENT: {
      static unsigned long last_ping = 0;
      if (millis() - last_ping >= 500) {
        last_ping = millis();
        bool ok = false;
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          ok = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);
          xSemaphoreGive(ros_mutex);
        }
        state = ok ? AGENT_AVAILABLE : WAITING_AGENT;
      }
      break;
    }

    case AGENT_AVAILABLE: {
      emergency_stop_motors();
      estop_active = true;
      entities_ready = false;
      bool created = false;
      if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        created = create_entities();
        xSemaphoreGive(ros_mutex);
      } else {
        Serial.println("[WARN] ros_mutex timeout in create_entities");
      }
      if (created) {
        diag_ms        = millis();
        prev_ms        = millis();
        last_imu_read  = millis();
        last_imu_pub   = millis();
        last_odom_pub  = millis() + 33;
        last_scan_pub  = millis() + 66;
        last_resync_ms = millis();
        last_cmd_ms    = millis();
        agent_ping_fail_count = 0;
        last_ping_attempt_ms  = millis();
        entities_ready = true;
        state = AGENT_CONNECTED;
      } else {
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          destroy_entities();
          xSemaphoreGive(ros_mutex);
        }
        state = WAITING_AGENT;
      }
      break;
    }

    case AGENT_DISCONNECTED: {
      Serial.println("[ROS] agent lost — waiting...");
      entities_ready = false;
      emergency_stop_motors();
      estop_active = true;
      applied_vx = 0; applied_omega = 0;
      last_vx = 0; last_omega = 0;
      agent_ping_fail_count = 0;
      if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        destroy_entities();
        xSemaphoreGive(ros_mutex);
      }
      state = WAITING_AGENT;
      break;
    }

    case AGENT_CONNECTED: {
      static unsigned long last_ping = 0;
      if (millis() - last_ping >= 1000) {

        // Ping is skipped entirely while driving continuously — see header
        // comment for why this doesn't reduce safety (the 300ms cmd
        // timeout reacts faster than any ping schedule would anyway).
        bool recently_driving = raw_cmd_active ||
          ((last_cmd_rx_ms > 0) &&
           (millis() - last_cmd_rx_ms < PING_SUPPRESS_ACTIVE_MS));

        if (recently_driving) {
          // No ping while driving — not counted as a fail, last_ping is
          // left untouched so this check re-runs next cycle. Do NOT break
          // here; fall through to the rest of AGENT_CONNECTED as normal.
        } else {
          last_ping = millis();
          last_ping_attempt_ms = millis();

          bool ping_ok = false;
          bool got_mutex = false;
          if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            got_mutex = true;
            ping_ok = (rmw_uros_ping_agent(50, 2) == RMW_RET_OK);
            xSemaphoreGive(ros_mutex);
          }

          if (!got_mutex) {
            Serial.println("[WARN] ros_mutex timeout in ping_agent (skipped this cycle, not counted as fail)");
          } else if (!ping_ok) {
            agent_ping_fail_count++;
            Serial.printf("[PING] agent ping failed (%d/%d)\n",
                          agent_ping_fail_count, AGENT_PING_FAIL_THRESHOLD);
            if (agent_ping_fail_count >= AGENT_PING_FAIL_THRESHOLD) {
              state = AGENT_DISCONNECTED;
              break;   // state changed — must skip the rest of
                       // AGENT_CONNECTED for this cycle
            }
          } else {
            agent_ping_fail_count = 0;
          }
        }
      }

      CHECKPOINT("resync");

      // ── IMU
      if (millis() - last_imu_read >= 20) {
        last_imu_read = millis();
        mpu_read(&imu_ax, &imu_ay, &imu_az, &imu_gx, &imu_gy, &imu_gz);
        gz_accum += imu_gz;
        gz_count++;

        noInterrupts(); long cur_enc_l = enc_l, cur_enc_r = enc_r; interrupts();

        bool commanded_still = (fabsf(applied_vx) < 0.005f && fabsf(applied_omega) < 0.005f);

        if (!commanded_still) {
          idle_window_open = false;
          gz_idle_accum = 0; gz_idle_count = 0;
        } else if (!idle_window_open) {
          idle_enc_l_ref = cur_enc_l;
          idle_enc_r_ref = cur_enc_r;
          idle_ax_min = idle_ax_max = imu_ax;
          idle_ay_min = idle_ay_max = imu_ay;
          gz_idle_accum = imu_gz;
          gz_idle_count = 1;
          idle_window_open = true;
        } else {
          idle_ax_min = min(idle_ax_min, imu_ax);
          idle_ax_max = max(idle_ax_max, imu_ax);
          idle_ay_min = min(idle_ay_min, imu_ay);
          idle_ay_max = max(idle_ay_max, imu_ay);

          bool encoder_truly_still = (cur_enc_l == idle_enc_l_ref) &&
                                      (cur_enc_r == idle_enc_r_ref);
          bool accel_stable = ((idle_ax_max - idle_ax_min) < IDLE_ACCEL_BAND) &&
                               ((idle_ay_max - idle_ay_min) < IDLE_ACCEL_BAND);

          if (!encoder_truly_still || !accel_stable) {
            idle_window_open = false;
            gz_idle_accum = 0; gz_idle_count = 0;
          } else {
            gz_idle_accum += imu_gz;
            gz_idle_count++;
            if (gz_idle_count >= 500) {
              gz_bias = gz_idle_accum / gz_idle_count;
              gz_idle_accum = 0; gz_idle_count = 0;
              idle_window_open = false;
            }
          }
        }
      }
      CHECKPOINT("imu_read");

      // ── IMU publish 10Hz ─────────────────────────────────────────────
      if (millis() - last_imu_pub >= 100) {
        last_imu_pub = millis();

        float gz_avg = (gz_count > 0) ? (gz_accum / gz_count) : imu_gz;
        gz_accum = 0.0f;
        gz_count = 0;

        int64_t t = (int64_t)millis() + epoch_offset_ms;
        imu_msg.header.stamp.sec     = (int32_t)(t / 1000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)((t % 1000LL) * 1000000LL);

        imu_msg.linear_acceleration.x = imu_ax - accel_bias_x;
        imu_msg.linear_acceleration.y = imu_ay - accel_bias_y;
        imu_msg.linear_acceleration.z = imu_az;
        imu_msg.angular_velocity.x    = imu_gx;
        imu_msg.angular_velocity.y    = imu_gy;
        imu_msg.angular_velocity.z    = gz_avg - gz_bias;

        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          rcl_publish(&imu_pub, &imu_msg, NULL);
          xSemaphoreGive(ros_mutex);
        }
      }
      CHECKPOINT("imu_pub");

      // ── Odom + Joint 10Hz ────────────────────────────────────────────
      if (millis() - last_odom_pub >= PUB_INTERVAL) {
        last_odom_pub = millis();
        unsigned long now = millis();
        float dt = (now - prev_ms) / 1000.f;
        if (dt <= 0 || dt > 0.5f) dt = 0.1f;
        prev_ms = now;

        noInterrupts(); long cl = enc_l, cr = enc_r; interrupts();
        long dl = cl - prev_enc_l;
        long dr = cr - prev_enc_r;
        prev_enc_l = cl; prev_enc_r = cr;

        float vl = (dl * DIST_PER_PULSE) / dt;
        float vr = (dr * DIST_PER_PULSE) / dt;
        float vx_raw = (vl + vr) * 0.5f;

        float vyaw_raw = (fabsf(vx_raw) > 0.001f && fabsf(current_steer) > 0.001f)
                 ? vx_raw * tanf(current_steer) * STEER_ODOM_GAIN / WHEELBASE
                 : 0.0f;

        vx_filtered   += ODOM_FILTER_ALPHA * (vx_raw   - vx_filtered);
        vyaw_filtered += ODOM_FILTER_ALPHA * (vyaw_raw - vyaw_filtered);

        float vx   = vx_filtered;
        float vyaw = vyaw_filtered;

        int64_t t = (int64_t)millis() + epoch_offset_ms;
        odom_lite_msg.stamp.sec     = (int32_t)(t / 1000LL);
        odom_lite_msg.stamp.nanosec = (uint32_t)((t % 1000LL) * 1000000LL);
        odom_lite_msg.vx   = vx;
        odom_lite_msg.vyaw = vyaw;

        joint_pos[0] += dl * RAD_PER_PULSE;
        joint_pos[1] += dr * RAD_PER_PULSE;
        joint_pos[2]  = joint_pos[0];
        joint_pos[3]  = joint_pos[1];
        joint_pos[4]  = current_steer;
        joint_pos[5]  = current_steer;
        joint_msg.header.stamp.sec     = odom_lite_msg.stamp.sec;
        joint_msg.header.stamp.nanosec = odom_lite_msg.stamp.nanosec;

        // Separate take/give per publisher (rather than one cycle covering
        // both) so ros_spin_task has a chance to interleave between the two
        // publishes — each rcl_publish() can block on the WiFi socket for
        // tens to hundreds of ms under poor conditions even at best_effort.
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          rcl_publish(&odom_lite_pub, &odom_lite_msg, NULL);
          xSemaphoreGive(ros_mutex);
        }
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          rcl_publish(&joint_pub, &joint_msg, NULL);
          xSemaphoreGive(ros_mutex);
        }
      }
      CHECKPOINT("odom_pub");

      // ── Scan + RAW LiDAR packet ───────────────────────────────────────
      if (millis() - last_scan_pub >= SCAN_INTERVAL) {
        last_scan_pub = millis();

        static float scan_copy[LIDAR_MAX_PTS];
        static uint8_t raw_copy[512];
        int raw_len = 0;

        portENTER_CRITICAL(&lidar_mux);
        memcpy(scan_copy, lidar_shadow, sizeof(float) * LIDAR_MAX_PTS);
        raw_len = lidar_raw_len;
        if (raw_len > 0 && raw_len <= (int)sizeof(raw_copy))
          memcpy(raw_copy, lidar_raw_shadow, raw_len);
        portEXIT_CRITICAL(&lidar_mux);

        memcpy(scan_msg.ranges.data, scan_copy, sizeof(float) * LIDAR_MAX_PTS);

        int64_t t = (int64_t)millis() + epoch_offset_ms;
        scan_msg.header.stamp.sec     = (int32_t)(t / 1000LL);
        scan_msg.header.stamp.nanosec = (uint32_t)((t % 1000LL) * 1000000LL);

        // Separate take/give per publisher here too, same reasoning as
        // odom_lite_pub/joint_pub above.
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          rcl_publish(&scan_pub, &scan_msg, NULL);
          xSemaphoreGive(ros_mutex);
        }

        if (raw_len > 0) {
          lidar_raw_msg.data.data = raw_copy;
          lidar_raw_msg.data.size = raw_len;
          lidar_raw_msg.data.capacity = sizeof(raw_copy);
          if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            rcl_publish(&lidar_raw_pub, &lidar_raw_msg, NULL);
            xSemaphoreGive(ros_mutex);
          }
          lidar_raw_msg.data.data = lidar_raw_shadow;
          lidar_raw_msg.data.capacity = sizeof(lidar_raw_shadow);
        }

        cnt_pub++;
      }
      CHECKPOINT("scan_pub");

      // ── Safety timeout / motor apply ─────────────────────────────────
      // last_vx/last_omega are written by raw_cmd_task. Snapshot them briefly
      // so a packet cannot change one value halfway through this cycle.
      float cmd_vx_snapshot, cmd_omega_snapshot;
      unsigned long cmd_ms_snapshot;
      portENTER_CRITICAL(&raw_cmd_mux);
      cmd_vx_snapshot = last_vx;
      cmd_omega_snapshot = last_omega;
      cmd_ms_snapshot = last_cmd_ms;
      portEXIT_CRITICAL(&raw_cmd_mux);

      if (millis() - cmd_ms_snapshot > 300) {
        if (!estop_active) {
          emergency_stop_motors();
          last_vx = 0; last_omega = 0;
          applied_vx = 0; applied_omega = 0;
          estop_active = true;
        }
      } else if (millis() - last_apply_ms >= 20) {
        estop_active = false;
        unsigned long apply_now = millis();
        float dt_ctrl = (apply_now - last_apply_ms) / 1000.0f;
        if (dt_ctrl <= 0.0f || dt_ctrl > 0.5f) dt_ctrl = 0.02f;
        last_apply_ms = apply_now;

        float tvx = (fabsf(cmd_vx_snapshot) > 0.01f || fabsf(cmd_omega_snapshot) > 0.01f) ? cmd_vx_snapshot    : 0.0f;
        float tow = (fabsf(cmd_vx_snapshot) > 0.01f || fabsf(cmd_omega_snapshot) > 0.01f) ? cmd_omega_snapshot : 0.0f;

        applied_vx    += constrain(tvx - applied_vx,    -VX_RAMP,    VX_RAMP);
        applied_omega += constrain(tow - applied_omega,  -OMEGA_RAMP, OMEGA_RAMP);

        // ── Measure real per-wheel speed since the last control cycle ──
        // Feeds the closed-loop PI in set_motor_l/r() below (via
        // apply_ackermann). Separate from the 10Hz odom encoder tracking
        // above -- this one runs at the same 20ms cadence as the control
        // loop itself.
        noInterrupts(); long cur_enc_l = enc_l, cur_enc_r = enc_r; interrupts();
        meas_vl = (cur_enc_l - prev_enc_l_ctrl) * DIST_PER_PULSE / dt_ctrl;
        meas_vr = (cur_enc_r - prev_enc_r_ctrl) * DIST_PER_PULSE / dt_ctrl;
        prev_enc_l_ctrl = cur_enc_l;
        prev_enc_r_ctrl = cur_enc_r;

        if (fabsf(applied_vx) > 0.01f || fabsf(applied_omega) > 0.01f) {
          float base_steer_rad;
          float drive_vx;

          if (fabsf(applied_vx) > 0.01f) {
            // Real forward-drive intent. The closed-loop PI now makes the
            // REAL speed track applied_vx directly (instead of the old
            // hard min_pwm=175 floor always forcing ~2 m/s regardless of
            // what was commanded), so the steering angle can go back to
            // using the commanded speed as-is -- no floor-correction
            // hack needed anymore.
            // Bicycle-model steering angle: delta = atan(omega*L / v) --
            // v's SIGN must be kept (not fabsf()'d) here. Dropping the
            // sign made the steering angle identical for forward and
            // reverse given the same omega, which is wrong: to trace the
            // same curvature in reverse, the steering angle must mirror
            // the forward-direction angle. Using fabsf(applied_vx) was
            // the cause of "reversing while turning steers the wrong way".
            base_steer_rad = atanf(applied_omega * WHEELBASE / applied_vx);
            base_steer_rad = constrain(base_steer_rad, -MAX_STEER, MAX_STEER);
            drive_vx = applied_vx;
          } else {
            // applied_vx ~ 0: this is an omega-only (or idle-noise) command.
            // This is an Ackermann car -- it cannot rotate in place without
            // forward motion -- so do NOT force any drive speed here. Keep
            // the original behavior: just aim the steering with the old
            // nominal-vx fallback formula, and pass the (near-zero)
            // applied_vx through unchanged so apply_ackermann()/set_motor_l/r()
            // correctly leave the wheels stopped (PI sees target<0.01 and
            // returns 0, per pi_speed_to_pwm()).
            base_steer_rad = constrain(applied_omega * WHEELBASE / 0.05f, -MAX_STEER, MAX_STEER);
            drive_vx = applied_vx;
          }

          heading_correction = 0.0f;
          if (fabsf(applied_vx) > 0.05f) {
            float actual_yaw_rate = imu_gz - gz_bias;
            float yaw_rate_error  = applied_omega - actual_yaw_rate;
            heading_correction = constrain(HEADING_KP * yaw_rate_error,
                                            -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
          }

          float steer_rad = constrain(base_steer_rad + heading_correction, -MAX_STEER, MAX_STEER);

          float steer_step = STEER_RAMP_RATE * 0.2f;
          float steer_err  = steer_rad - current_steer;
          current_steer += constrain(steer_err, -steer_step, steer_step);
          set_steering(current_steer);

          apply_ackermann(constrain(drive_vx, -MAX_VEL, MAX_VEL), current_steer,
                           meas_vl, meas_vr, dt_ctrl);
        } else {
          emergency_stop_motors();
          applied_vx = 0; applied_omega = 0;
          heading_correction = 0.0f;
        }
      }
      CHECKPOINT("safety_apply");

      // ── DIAG 5s ──────────────────────────────────────────────────────
      if (millis() - diag_ms >= 5000) {
        diag_ms = millis();
        Serial.printf("[DIAG] bytes=%lu ok=%lu bad=%lu pub=%lu ratio=%.1f%% heap=%d\n",
          cnt_bytes, cnt_ok, cnt_bad, cnt_pub,
          (cnt_ok + cnt_bad > 0) ? 100.0f * cnt_ok / (cnt_ok + cnt_bad) : 0.0f,
          ESP.getFreeHeap());
        Serial.printf("[IMU]  gz_raw=%.5f  gz_corr=%.5f  bias=%.5f\n",
          imu_gz, imu_gz - gz_bias, gz_bias);
        Serial.printf("[HDG]  heading_correction=%.4f rad  actual_yaw_rate=%.4f\n",
          heading_correction, imu_gz - gz_bias);
        Serial.printf("[ENC]  enc_l=%ld enc_r=%ld  steer=%.3f rad\n",
          enc_l, enc_r, current_steer);
        Serial.printf("[MOTOR] pwm_l=%.1f pwm_r=%.1f meas_vl=%.3f meas_vr=%.3f int_l=%.1f int_r=%.1f applied_vx=%.3f applied_omega=%.3f udp_vx=%.3f udp_omega=%.3f ros_vx=%.3f ros_omega=%.3f\n",
          pwm_l_current, pwm_r_current, meas_vl, meas_vr, integral_l, integral_r,
          applied_vx, applied_omega, last_vx, last_omega,
          ros_last_vx, ros_last_omega);
        Serial.printf("[LOOP] max_loop_us=%lu\n", max_loop_us);
        Serial.printf("[WIFI] RSSI=%d dBm  status=%d\n", WiFi.RSSI(), WiFi.status());
        Serial.printf("[CMD]  rx_count=%lu  max_gap_ms=%lu\n", cmd_rx_count, max_cmd_gap_ms);
        Serial.printf("[UDP]  rx_count=%lu  max_gap_ms=%lu  seq=%lu  active=%d  udp_gap=%lu ms\n",
          raw_cmd_rx_count, raw_cmd_max_gap_ms, raw_cmd_last_seq, raw_cmd_active,
          raw_cmd_last_rx_ms ? millis() - raw_cmd_last_rx_ms : 0UL);
        Serial.printf("[RF]   data_to_us=%lu  udp8888_to_us=%lu  (cmd_rx_count=%lu)\n",
          promisc_data_to_us, promisc_udp8888_to_us, cmd_rx_count);
        Serial.printf("[SPIN] avg=%lu us  max=%lu us  n=%lu\n",
          spin_samples ? spin_sum / spin_samples : 0, spin_max, spin_samples);
        Serial.printf("[PING] current_fail_count=%d/%d\n",
          agent_ping_fail_count, AGENT_PING_FAIL_THRESHOLD);
        Serial.printf("[PING] ms_since_last_attempt=%lu  suppress_active_ms=%lu\n",
          millis() - last_ping_attempt_ms, PING_SUPPRESS_ACTIVE_MS);

        cnt_bytes = 0; cnt_ok = 0; cnt_bad = 0; cnt_pub = 0;
        max_loop_us = 0;
        cmd_rx_count = 0; max_cmd_gap_ms = 0;
        raw_cmd_rx_count = 0; raw_cmd_max_gap_ms = 0;
        promisc_data_to_us = 0; promisc_udp8888_to_us = 0;
        spin_sum = 0; spin_max = 0; spin_samples = 0;
      }

      break;
    }
  }

  unsigned long loop_dur = micros() - loop_t0;
  if (loop_dur > max_loop_us) max_loop_us = loop_dur;

  taskYIELD();
}
