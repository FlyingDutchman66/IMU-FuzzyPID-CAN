/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* printf("%f") needs no "Use float with printf" (-u _printf_float) project setting: USER CODE 0 links it. */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mcp2515.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct { float r, psi, e, dk[3], k[3], delta_cmd, u; int16_t acc[3]; uint32_t isr_us; uint8_t failsafe; } telem_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Hardware map. Pins are assigned in IMU-FuzzyPID.ioc; change them there and these follow. ---- */
#define IMU_I2C               hi2c1                 // I2C1: PB8 SCL, PB9 SDA, 400 kHz
#define IMU_ADDR              (0x68 << 1)           // 0xD0, HAL expects the 8-bit address
#define PWM_TIM               htim1                 // TIM1_CH1 = PA8 -> L298N ENA, 72 MHz / 3600 = 20 kHz
#define PWM_CH                TIM_CHANNEL_1
#define CTRL_TIM              htim3                 // 72 MHz / 7200 / 50 = 200 Hz update IRQ
#define IN1_PORT              MOTOR_IN1_GPIO_Port   // PB0
#define IN1_PIN               MOTOR_IN1_Pin
#define IN2_PORT              MOTOR_IN2_GPIO_Port   // PB1
#define IN2_PIN               MOTOR_IN2_Pin

/* ---- MPU6050 ---- */
#define MPU6050_CONFIG        0x1A
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75
#define GYRO_LSB_PER_DPS      131.0f        // ±250 °/s
#define GYRO_CAL_SAMPLES      200           // x 5 ms = ~1 s at rest
#define GYRO_DEADBAND_DPS     0.5f          // |r| below this reads 0 so psi doesn't creep at rest; 0 disables
#define IMU_TIMEOUT_MS        2
#define IMU_MAX_FAILS         20            // 100 ms of bad reads in a row -> motor off

/* ---- Loop timing ---- */
#define TS_INNER              0.005f        // 200 Hz
#define OUTER_DIV             10            // outer loop every 10th tick
#define TS_OUTER              (TS_INNER * OUTER_DIV)   // 0.05 s = 20 Hz
#define TELEM_PERIOD_MS       100           // one ~1.8 kB frame takes ~77 ms at 230400 baud
#define CAN_TELEM_ID          0x101         // 8-byte big-endian telemetry frame, layout in CAN_SendTelemetry()
#define CAN_PERIOD_MS         50            // 20 Hz

/* ---- What drives the motor ---- */
#define MOTOR_MODE            2             // 0: autopilot (the controller), 1: keyboard test, 2: follow-the-turn demo
#define MOTOR_TEST_MAX        1.00f         // mode 1: hard duty cap. 1.00 = bare-motor test only (no propeller)!
#define MOTOR_TEST_STEP       0.10f         // mode 1: '+' / '-' step
#define MOTOR_TEST_RAMP       0.01f         // mode 1: max duty increase per 5 ms tick: 0 -> 30 % in 150 ms
#define DEMO_RATE_MIN         5.0f          // mode 2: |yaw rate| [deg/s] below this = motor off (ignores hand tremor)
#define DEMO_RATE_FULL        60.0f         // mode 2: |yaw rate| where the duty reaches DEMO_U_MAX
#define DEMO_U_START          0.40f         // mode 2: duty at DEMO_RATE_MIN (bare motor starts at 30-40 % on 9 V)
#define DEMO_U_MAX            0.70f         // mode 2: duty cap
#define DEMO_TRAVEL_TICKS     100           // mode 2: soft end stop, net 5 ms ticks of running each way (100 = 0.5 s)
#define DEMO_SIGN             (-1.0f)       // mode 2: -1 = counter-steer (turning right drives the motor REV); flip if wrong

/* ---- Outer heading loop + fuzzy scheduler ---- */
#define PSI_D                 0.0f          // heading setpoint [deg]; boot heading is 0
#define DELTA_MAX             30.0f         // steering command limit [deg]
#define GE                    (3.0f / 30.0f)   // e    [deg]   -> fuzzy universe [-3, 3]
#define GEC                   (3.0f / 15.0f)   // edot [deg/s] -> fuzzy universe [-3, 3]

/* ---- Inner steering loop ---- */
#define BENCH_TEST_NO_ENCODER 1             // 1: delta = 0, so u follows delta_cmd (no steering sensor yet)
#define IN_KP                 (1.0f / DELTA_MAX)   // full duty at full-lock error
#define IN_KI                 (BENCH_TEST_NO_ENCODER ? 0.0f : 0.10f)  // bench: no delta feedback, an I-term would only ramp u to the rail
#define IN_KD                 0.002f
#define IN_D_ALPHA            0.2f          // derivative LPF: Ts / (Tf + Ts), Tf = 20 ms
#define IN_DEADBAND           0.5f          // |e_delta| below this [deg]: motor off, integrator frozen
#define BOOST                 0.3f          // backlash kick on direction reversal...
#define BOOST_TICKS           4             // ...for this many 5 ms ticks
#define U_MIN                 0.15f         // smallest non-zero duty (motor stiction)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t fuzzy_enabled = 1;     // 0 = fixed BASE_GAINS. Flip it live from Live Expressions.

/* Outer gains {Kp [deg/deg], Ki [deg/(deg*s)], Kd [deg/(deg/s)]}: K = K_MIN + dK * (K_MAX - K_MIN) */
static const float K_MIN[3]      = {0.5f, 0.00f, 0.05f};
static const float K_MAX[3]      = {2.0f, 0.20f, 0.30f};
static const float BASE_GAINS[3] = {1.0f, 0.05f, 0.15f};

/* Fuzzy rules: row = e set, column = edot set, both in order NL NM NS ZE PS PM PL.
   Entry n is the crisp singleton 0.25 * n: 0 = 0.0, 1 = 0.25, 2 = 0.5, 3 = 0.75, 4 = 1.0 */
static const uint8_t RULE_KP[7][7] = {  // push hard while |e| is large and growing, ease off as it closes
  {4, 4, 4, 3, 3, 2, 2},
  {4, 4, 3, 3, 2, 2, 1},
  {3, 3, 3, 2, 2, 1, 1},
  {2, 2, 1, 1, 1, 2, 2},
  {1, 1, 2, 2, 3, 3, 3},
  {1, 2, 2, 3, 3, 4, 4},
  {2, 2, 3, 3, 4, 4, 4},
};
static const uint8_t RULE_KI[7][7] = {  // integrate only near the setpoint, never on big errors
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 1, 1, 0, 0},
  {1, 1, 2, 3, 2, 1, 1},
  {1, 2, 3, 4, 3, 2, 1},
  {1, 1, 2, 3, 2, 1, 1},
  {0, 0, 1, 1, 1, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
};
static const uint8_t RULE_KD[7][7] = {  // damp hardest near the setpoint while still turning fast
  {1, 1, 0, 0, 0, 1, 2},
  {1, 1, 1, 0, 1, 2, 2},
  {2, 1, 1, 1, 2, 3, 3},
  {4, 3, 2, 1, 2, 3, 4},
  {3, 3, 2, 1, 1, 1, 2},
  {2, 2, 1, 0, 1, 1, 1},
  {2, 1, 0, 0, 0, 1, 1},
};

static float gyro_bias_dps;
static volatile telem_t telem;          // ISR writes it, main loop snapshots it with IRQs off
static uint8_t can_ready;               // set once the MCP2515 passed its self-test and the loop runs
static volatile float test_u;           // mode 1: signed target duty from the keys; control_step ramps toward it
static volatile uint8_t demo_armed;     // mode 2: 'g' arms, SPACE (or an IMU failsafe) disarms; boots disarmed
static volatile int demo_travel;        // mode 2: net 5 ms ticks driven (+/-), shown on the dashboard
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void can_service(void);
static void keys_service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Link newlib-nano's float formatter so printf("%f") works. Same effect as the
// project setting "Use float with printf from newlib-nano (-u _printf_float)".
__asm(".global _printf_float");

// printf -> USART2 (the ST-LINK virtual COM port). syscalls.c _write() calls this.
int __io_putchar(int ch)
{
  can_service();  // a dashboard frame blocks ~77 ms here, longer than the CAN period
  keys_service(); // ...and keys typed meanwhile must still get through (SPACE = stop)
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static float clampf(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }

// Dashboard gauges. Each returns its own static buffer: use at most once per printf.
// [#######---]  x in [0, 1]
static const char *bar_uni(float x)
{
  static char s[] = "[----------]";
  int n = lrintf(clampf(x, 0.0f, 1.0f) * 10.0f);
  for (int i = 0; i < 10; i++)
    s[1 + i] = i < n ? '#' : '-';
  return s;
}

// [   <<<<<<<|          ]  x in [-full, +full]: negative fills left of the centre, positive right
static const char *bar_bi(float x, float full)
{
  static char s[] = "[          |          ]";
  int n = lrintf(clampf(fabsf(x) / full, 0.0f, 1.0f) * 10.0f);
  for (int i = 0; i < 10; i++)
  {
    s[10 - i] = x < 0.0f && i < n ? '<' : ' ';
    s[12 + i] = x > 0.0f && i < n ? '>' : ' ';
  }
  return s;
}

/* ---- Top view: 39 x 10 character ship outline, N (boot heading = setpoint) up. Main loop only. ---- */
#define CV_W          39
#define CV_H          10
#define CV_CY         4             // canvas row of the ship centre (W and E sit on it)
#define D2R           0.01745329f

// Body frame seen from above: x to the bow, y to starboard. Units are terminal columns.
static const float SHIP_V[6][2] = {
  {6, 0},                                                   // 0 bow
  {1.5f, -3.5f}, {-6, -3.5f}, {-6, 3.5f}, {1.5f, 3.5f},          // 1-4 port shoulder, port quarter, stbd quarter, stbd shoulder
  {-6, 0},                                                  // 5 rudder hinge (6, the tip, is computed)
};
static const uint8_t SHIP_E[6][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}, {5, 6}};

static void cv_plot(char cv[CV_H][CV_W + 1], int x, int y, char ch)
{
  if (x >= 0 && x < CV_W && y >= 0 && y < CV_H)
    cv[y][x] = ch;
}

// Bresenham. One character per edge, picked from its on-screen slope (a row is ~2 columns tall).
static void cv_line(char cv[CV_H][CV_W + 1], int x0, int y0, int x1, int y1)
{
  int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
  char ch = dx > -3 * dy ? '-' : -dy > dx ? '|' : sx == sy ? '\\' : '/';

  for (;;)
  {
    cv_plot(cv, x0, y0, ch);
    if (x0 == x1 && y0 == y1)
      return;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// Body (x, y) -> canvas, yawed by psi (clockwise). North is up; rows are ~2 columns tall.
static void ship_project(float x, float y, float s, float c, int *col, int *row)
{
  *col = lrintf(CV_W / 2 + x * s + y * c);
  *row = lrintf(CV_CY - (x * c - y * s) * 0.5f);
}

// psi [deg] turns the hull (+ = clockwise), delta [deg] swings the rudder (+ = trailing edge to
// starboard = bow turns to starboard). frame animates the wake.
static void ship_render(char cv[CV_H][CV_W + 1], float psi, float delta, uint32_t frame)
{
  float s = sinf(psi * D2R), c = cosf(psi * D2R), d = clampf(delta, -DELTA_MAX, DELTA_MAX) * D2R;
  float v[7][2];
  int px[7], py[7], wx, wy;

  for (int i = 0; i < CV_H; i++)
  {
    memset(cv[i], ' ', CV_W);
    cv[i][CV_W] = '\0';
  }
  cv[0][CV_W / 2] = 'N';  // compass letters sit under the ship
  cv[CV_H - 1][CV_W / 2] = 'S';
  cv[CV_CY][0] = 'W';
  cv[CV_CY][CV_W - 1] = 'E';
  for (int k = 0; k < 2; k++)  // wake: a V of ~ sliding aft one column per frame
    for (int side = -1; side <= 1; side += 2)
    {
      float a = 2.0f + 3.0f * k + frame % 3;
      ship_project(-8.0f - a, side * 0.8f * a, s, c, &wx, &wy);
      cv_plot(cv, wx, wy, '~');
    }

  memcpy(v, SHIP_V, sizeof SHIP_V);
  v[6][0] = -6.0f - 3.0f * cosf(d);  // rudder tip: 3 long, swung by delta about the hinge
  v[6][1] = 3.0f * sinf(d);
  for (int i = 0; i < 7; i++)
    ship_project(v[i][0], v[i][1], s, c, &px[i], &py[i]);
  for (int i = 0; i < 6; i++)
    cv_line(cv, px[SHIP_E[i][0]], py[SHIP_E[i][0]], px[SHIP_E[i][1]], py[SHIP_E[i][1]]);
  for (int i = 1; i <= 4; i++)
    cv_plot(cv, px[i], py[i], '+');
  int bx = px[0] - CV_W / 2, by = 2 * (py[0] - CV_CY);  // bow marker points where the bow points
  cv_plot(cv, px[0], py[0], abs(bx) > abs(by) ? (bx > 0 ? '>' : '<') : (by < 0 ? '^' : 'v'));
}

// Tilt from gravity [deg]: roll + = starboard down, pitch + = bow down (MPU X arrow at the bow,
// chip face up). Any common scale (raw counts, sums of samples) cancels inside atan2.
static void accel_tilt(float ax, float ay, float az, float *roll, float *pitch)
{
  *roll = atan2f(ay, az) / D2R;
  *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) / D2R;
}

static HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t who = 0, wake = 0x00, dlpf = 0x03;  // DLPF 42 Hz: stops 200 Hz sampling aliasing gyro noise

  if (HAL_I2C_Mem_Read(&IMU_I2C, IMU_ADDR, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 100) != HAL_OK)
    return HAL_ERROR;
  printf("WHO_AM_I = 0x%02X (0x68 genuine, 0x70 clone)\r\n", who);

  // Clear the SLEEP bit: PWR_MGMT_1 = 0x00
  if (HAL_I2C_Mem_Write(&IMU_I2C, IMU_ADDR, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &wake, 1, 100) != HAL_OK)
    return HAL_ERROR;
  return HAL_I2C_Mem_Write(&IMU_I2C, IMU_ADDR, MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &dlpf, 1, 100);
}

// One 14-byte burst from ACCEL_XOUT_H: AX AY AZ TEMP GX GY GZ, big-endian pairs.
// r = bias-corrected yaw rate [deg/s], clockwise-positive; acc = raw accel (16384 LSB/g).
// HAL_ERROR on a bus fault or a railed (stuck/saturated) gyro reading.
// ponytail: a bus stuck BUSY costs HAL's fixed 25 ms per call; add SCL-toggle bus recovery if that ever shows up.
static HAL_StatusTypeDef imu_read_yaw_rate_dps(float *r, int16_t acc[3])
{
  uint8_t b[14];

  if (HAL_I2C_Mem_Read(&IMU_I2C, IMU_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, b, 14, IMU_TIMEOUT_MS) != HAL_OK)
    return HAL_ERROR;
  for (int i = 0; i < 3; i++)
    acc[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]);
  int16_t gz = (int16_t)(b[12] << 8 | b[13]);
  if (gz == INT16_MAX || gz == INT16_MIN)
    return HAL_ERROR;
  *r = -gz / GYRO_LSB_PER_DPS - gyro_bias_dps;  // chip Z points up (az = +1 g at rest), so +gz is counter-clockwise
  return HAL_OK;
}

// L298N: u > 0 FWD, u < 0 REV, u == 0 both IN low (coast). Duty = |u| * ARR on ENA.
static void motor_set(float u)
{
  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, u > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, u < 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH, (uint32_t)(fabsf(u) * __HAL_TIM_GET_AUTORELOAD(&PWM_TIM)));
}

// 7 triangular sets NL..PL, unit-spaced on [-3, 3]. Any input sits in at most 2 adjacent sets,
// so only 2 x 2 = 4 rules fire. min() AND, weighted average of singletons (centroid). dk[] in [0, 1].
static void fuzzy_schedule(float e, float edot, float dk[3])
{
  float x = clampf(e * GE, -3.0f, 3.0f) + 3.0f;      // 0..6, set index space
  float y = clampf(edot * GEC, -3.0f, 3.0f) + 3.0f;
  int i = x < 6.0f ? (int)x : 5, j = y < 6.0f ? (int)y : 5;
  float mx[2] = {i + 1 - x, x - i}, my[2] = {j + 1 - y, y - j};  // membership in set i / i+1
  float wsum = 0.0f, acc[3] = {0};

  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
    {
      float w = fminf(mx[a], my[b]);
      wsum   += w;
      acc[0] += w * RULE_KP[i + a][j + b];
      acc[1] += w * RULE_KI[i + a][j + b];
      acc[2] += w * RULE_KD[i + a][j + b];
    }
  for (int k = 0; k < 3; k++)
    dk[k] = 0.25f * acc[k] / wsum;  // wsum >= 0.5: each input has one set with membership >= 0.5
}

// Boot check: a grid point returns its table entry, a midpoint blends its two neighbours,
// out-of-range inputs clamp to the edge sets, and the heading wrap folds 190 to -170.
static int fuzzy_selftest(void)
{
  float dk[3];
  int ok;

  fuzzy_schedule(0.0f, 0.0f, dk);
  ok = dk[0] == 0.25f * RULE_KP[3][3] && dk[1] == 0.25f * RULE_KI[3][3] && dk[2] == 0.25f * RULE_KD[3][3];
  fuzzy_schedule(15.0f, 0.0f, dk);  // e = 15 deg -> 1.5 = halfway PS..PM
  ok &= fabsf(dk[0] - 0.25f * (RULE_KP[4][3] + RULE_KP[5][3]) / 2.0f) < 1e-6f;
  fuzzy_schedule(1e6f, -1e6f, dk);  // e = PL, edot = NL
  ok &= dk[0] == 0.25f * RULE_KP[6][0];
  ok &= remainderf(190.0f, 360.0f) == -170.0f;
  return ok;
}

// 20 Hz heading PID -> steering command delta_cmd [deg]. D on measurement (-Kd*r),
// conditional-integration anti-windup.
static float outer_step(float e, float r, float dk[3], float k[3])
{
  static float integ;

  fuzzy_schedule(e, -r, dk);  // edot = -r because psi_d is constant
  for (int n = 0; n < 3; n++)
    k[n] = fuzzy_enabled ? K_MIN[n] + dk[n] * (K_MAX[n] - K_MIN[n]) : BASE_GAINS[n];
#if BENCH_TEST_NO_ENCODER
  k[1] = 0.0f;  // bench: no hull turns the heading back, so an I-term only winds the rudder onto its stop
#endif

  float di = k[1] * e * TS_OUTER;
  float u = k[0] * e + integ + di - k[2] * r;
  if (fabsf(u) < DELTA_MAX || u * e < 0.0f)  // integrate only when unsaturated or unwinding
    integ += di;
  return clampf(k[0] * e + integ - k[2] * r, -DELTA_MAX, DELTA_MAX);
}

// 200 Hz steering PID -> signed motor command u in [-1, 1].
static float inner_step(float delta_cmd, float delta)
{
  static float integ, d_filt, delta_prev;
  static int8_t last_dir;
  static uint8_t boost_left;
  float e = delta_cmd - delta;

  d_filt += IN_D_ALPHA * (-(delta - delta_prev) / TS_INNER - d_filt);  // D on measurement, low-passed
  delta_prev = delta;
  if (fabsf(e) < IN_DEADBAND)
    return 0.0f;  // deadband: motor off, integrator frozen

  float di = IN_KI * e * TS_INNER;
  float u = IN_KP * e + integ + di + IN_KD * d_filt;
  if (fabsf(u) < 1.0f || u * e < 0.0f)
    integ += di;
  u = clampf(IN_KP * e + integ + IN_KD * d_filt, -1.0f, 1.0f);

  int8_t dir = (u > 0.0f) - (u < 0.0f);
  if (dir && last_dir && dir != last_dir)
    boost_left = BOOST_TICKS;  // reversal: kick through gear backlash
  if (dir)
    last_dir = dir;
  u = dir * (U_MIN + (1.0f - U_MIN) * fabsf(u));  // lift past stiction, keep the slope
  if (boost_left)
  {
    boost_left--;
    u += dir * BOOST;
  }
  return clampf(u, -1.0f, 1.0f);
}

static void put_x100(uint8_t *p, float x)  // x * 100 -> big-endian int16, saturated
{
  int16_t v = (int16_t)lrintf(clampf(x * 100.0f, -32768.0f, 32767.0f));
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)v;
}

// Frame 0x101, 8 bytes, big-endian fixed point (no struct packing, so both ends agree by construction):
//   [0-1] int16 r x100 [0.01 deg/s]   [2-3] int16 psi x100 [0.01 deg]   [4-5] int16 delta_cmd x100 [0.01 deg]
//   [6] int8 u x100 [%]   [7] bits 7:4 rolling counter, bit 1 fuzzy_enabled, bit 0 sensors_ok
static void CAN_SendTelemetry(void)
{
  static uint8_t counter;

  __disable_irq();
  telem_t t = telem;
  __enable_irq();

  uint8_t d[8];
  put_x100(&d[0], t.r);
  put_x100(&d[2], t.psi);
  put_x100(&d[4], t.delta_cmd);
  d[6] = (uint8_t)(int8_t)lrintf(clampf(t.u, -1.0f, 1.0f) * 100.0f);
  d[7] = (uint8_t)((counter++ & 0x0F) << 4 | (fuzzy_enabled ? 0x02 : 0) | (t.failsafe ? 0 : 0x01));
  MCP2515_Send(CAN_TELEM_ID, d, 8);  // HAL_BUSY = previous frame unACKed: this one is skipped, the counter shows it
}

// 20 Hz CAN from main context only, never the TIM3 ISR. Polled from the idle wait and from __io_putchar.
static void can_service(void)
{
  static uint32_t t_can;

  if (!can_ready || HAL_GetTick() - t_can < CAN_PERIOD_MS)
    return;
  t_can = HAL_GetTick();
  CAN_SendTelemetry();
}

// Mode 1, 5 ms ISR: duty toward 0 at once (SPACE stops instantly), away from 0 by at most
// MOTOR_TEST_RAMP per tick. A reversal cuts to 0 first, then ramps the other way.
static float test_ramp(float target)
{
  static float u;

  if (u * target < 0.0f)
    u = 0.0f;
  if (fabsf(target) <= fabsf(u))
    u = target;
  else
    u += copysignf(fminf(MOTOR_TEST_RAMP, fabsf(target - u)), target);
  return u;
}

// Mode 2, 5 ms ISR: drive the motor only WHILE the IMU turns. Duty grows with |yaw rate| from DEMO_U_START
// to DEMO_U_MAX. No rudder encoder yet, so a soft end stop allows at most DEMO_TRAVEL_TICKS of net running
// each way from the boot position (boot with the rudder centred); driving back is always allowed.
static float demo_step(float r)
{
  static int travel;
  float u = 0.0f;

  if (demo_armed && fabsf(r) >= DEMO_RATE_MIN)
  {
    float k = clampf((fabsf(r) - DEMO_RATE_MIN) / (DEMO_RATE_FULL - DEMO_RATE_MIN), 0.0f, 1.0f);
    u = copysignf(DEMO_U_START + k * (DEMO_U_MAX - DEMO_U_START), DEMO_SIGN * r);
  }
  if ((u > 0.0f && travel >= DEMO_TRAVEL_TICKS) || (u < 0.0f && travel <= -DEMO_TRAVEL_TICKS))
    u = 0.0f;
  travel += (u > 0.0f) - (u < 0.0f);
  demo_travel = travel;
  return u;
}

// COM7 keys, polled like can_service (idle wait + __io_putchar), so a key gets in within one character time.
//   mode 1: + / - change the duty in 10 % steps up to the cap, f / r pick the direction, SPACE stops.
//   mode 2: g arms the demo, SPACE disarms it (motor off).
static void keys_service(void)
{
  static int8_t dir = 1;

  if (MOTOR_MODE == 0 || !__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
    return;
  char c = (char)(huart2.Instance->DR & 0xFF);
  if (MOTOR_MODE == 2)
  {
    if (c == 'g' || c == 'G')
      demo_armed = 1;
    else if (c == ' ')
      demo_armed = 0;
    return;
  }
  float lvl = fabsf(test_u);

  if (c == ' ')
    lvl = 0.0f;
  else if (c == '+' || c == '=')
    lvl += MOTOR_TEST_STEP;
  else if (c == '-' || c == '_')
    lvl -= MOTOR_TEST_STEP;
  else if (c == 'f' || c == 'F')
    dir = 1;
  else if (c == 'r' || c == 'R')
    dir = -1;
  else
    return;
  test_u = dir * clampf(roundf(lvl * 100.0f) / 100.0f, 0.0f, MOTOR_TEST_MAX);  // whole percent, never past the cap
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);  // MPU6050 power-up time (datasheet: 100 ms max)
  if (MPU6050_Init() != HAL_OK)
  {
    printf("MPU6050 not responding at 0x68. I2C bus scan:\r\n");
    for (uint8_t a = 1; a < 128; a++)
      if (HAL_I2C_IsDeviceReady(&IMU_I2C, a << 1, 1, 10) == HAL_OK)
        printf("  device at 0x%02X\r\n", a);
    printf("Scan done - nothing listed = check SDA/SCL swap, 0x69 = AD0 is high\r\n");
    Error_Handler();
  }
  printf("MPU6050 awake\r\n");

  if (!fuzzy_selftest())
  {
    printf("Fuzzy self-test FAILED\r\n");
    Error_Handler();
  }

  if (MCP2515_Init() != HAL_OK || MCP2515_SelfTest() != HAL_OK)
  {
    printf("MCP2515 self-test FAILED - check SPI1 PA5/PA6/PA7, CS PB6, module VCC/GND\r\n");
    Error_Handler();
  }
  printf("MCP2515 loopback OK, normal mode, 500 kbps, frame 0x%03X every %d ms\r\n", CAN_TELEM_ID, CAN_PERIOD_MS);

  // IMU calibration, board still and level: average the gyro Z zero-rate bias (gyro_bias_dps is
  // still 0 here) and the accel, whose tilt becomes the level reference for roll/pitch.
  printf("Calibrating IMU - keep the board still and level...\r\n");
  HAL_Delay(100);  // let the gyro settle after wake-up
  float sum = 0.0f, r, a[3] = {0}, roll0, pitch0;
  int16_t acc[3];
  for (int n = 0; n < GYRO_CAL_SAMPLES; n++)
  {
    if (imu_read_yaw_rate_dps(&r, acc) != HAL_OK)
    {
      printf("IMU read error during calibration\r\n");
      Error_Handler();
    }
    sum += r;
    for (int i = 0; i < 3; i++)
      a[i] += acc[i];
    HAL_Delay(5);
  }
  gyro_bias_dps = sum / GYRO_CAL_SAMPLES;
  // ponytail: subtracting angles is exact only for a small mount tilt; rotate the gravity vector instead if the board sits >10 deg off level
  accel_tilt(a[0], a[1], a[2], &roll0, &pitch0);
  printf("Gyro Z bias %+.3f deg/s, level offset roll %+.1f / pitch %+.1f deg removed. Heading now 0.\r\n",
         gyro_bias_dps, roll0, pitch0);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // DWT cycle counter, for the ISR load readout
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  printf("\033[2J\033[?25l");  // clear screen, hide cursor: from here the dashboard redraws in place
  HAL_TIM_PWM_Start(&PWM_TIM, PWM_CH);
  HAL_TIM_Base_Start_IT(&CTRL_TIM);  // 200 Hz loop now runs in HAL_TIM_PeriodElapsedCallback
  can_ready = 1;
  uint32_t t0 = HAL_GetTick(), frame = 0;
  float roll = 0.0f, pitch = 0.0f;  // accel tilt [deg], low-passed per frame
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    while (HAL_GetTick() - t0 < TELEM_PERIOD_MS)
    {
      can_service();
      keys_service();
    }
    t0 = HAL_GetTick();

    __disable_irq();
    telem_t t = telem;  // consistent snapshot: the ISR can't run mid-copy
    __enable_irq();

    float rl, pt;
    accel_tilt(t.acc[0], t.acc[1], t.acc[2], &rl, &pt);
    roll  += 0.25f * (rl - roll0 - roll);    // relative to the boot level, low-passed
    pitch += 0.25f * (pt - pitch0 - pitch);  // (alpha 0.25) to reject motor vibration

    char cv[CV_H][CV_W + 1];
    ship_render(cv, t.psi, t.delta_cmd, frame++);
    const char *mode = t.failsafe ? "IMU FAILSAFE" : MOTOR_MODE == 1 ? "MOTOR TEST" : MOTOR_MODE == 2 ? "RUDDER DEMO"
                     : fuzzy_enabled ? "FUZZY ACTIVE" : "FIXED GAINS";
    const char *dir = t.u > 0.0f ? "FWD" : t.u < 0.0f ? "REV" : "STOP";

    printf("\033[H"
           "+================================================================================+\r\n"
           "|          STM32F446RE FUZZY CASCADE PID - TOP VIEW & BENCH TELEMETRY            |\r\n");
    printf("| Mode: [%-12s] | Loop: 200Hz/20Hz | ISR Load: %4lu us / 5000 us          |\r\n", mode, t.isr_us);
    printf("+-----------------------------------------+--------------------------------------+\r\n"
           "| TOP VIEW: HEADING  (N = 0 deg, setpoint)| 1. IMU & ATTITUDE (MPU6050)          |\r\n");
    printf("|                                         |  Yaw Rate (r)   : %+7.2f deg/s      |\r\n", t.r);
    printf("| %s |  Heading (psi)  : %+7.2f deg        |\r\n", cv[0], t.psi);
    printf("| %s |  Roll / Pitch   : %+6.1f / %+5.1f deg |\r\n", cv[1], roll, pitch);
    printf("| %s |  Setpoint (psid): %7.2f deg        |\r\n", cv[2], PSI_D);
    printf("| %s |  Error (e)      : %+7.2f deg        |\r\n", cv[3], t.e);
    printf("| %s |  Err: %s        |\r\n", cv[4], bar_bi(t.e, 30.0f));
    printf("| %s +--------------------------------------+\r\n", cv[5]);
    printf("| %s | 2. FUZZY GAIN SCHEDULER (20 Hz)      |\r\n", cv[6]);
    printf("| %s |  dKp: %4.2f %s Kp: %4.2f     |\r\n", cv[7], t.dk[0], bar_uni(t.dk[0]), t.k[0]);
    printf("| %s |  dKi: %4.2f %s Ki: %4.2f     |\r\n", cv[8], t.dk[1], bar_uni(t.dk[1]), t.k[1]);
    printf("| %s |  dKd: %4.2f %s Kd: %4.2f     |\r\n", cv[9], t.dk[2], bar_uni(t.dk[2]), t.k[2]);
    printf("+-----------------------------------------+--------------------------------------+\r\n"
           "| 3. CASCADE ACTUATOR & L298N MOTOR DRIVER (20 kHz PWM)                          |\r\n");
    printf("|  Steering Cmd (delta_cmd) : %+6.2f deg  | L298N PWM (PA8): %5.1f %% [ %-4s]     |\r\n",
           t.delta_cmd, fabsf(t.u) * 100.0f, dir);
    printf("|  Signed Command (u)       : %+6.2f      | Drive REV %s FWD|\r\n", t.u, bar_bi(t.u, 1.0f));
    printf("+================================================================================+\r\n");
    if (MOTOR_MODE == 1)
      printf("| MOTOR TEST  target %+4.0f %% (cap %2.0f)  keys: + - 10 %%, f r dir, SPACE stop        |\r\n",
             test_u * 100.0f, MOTOR_TEST_MAX * 100.0f);
    else if (MOTOR_MODE == 2)
      printf("| RUDDER DEMO %-8s  yaw %+6.1f deg/s  travel %+4d/%d  keys: g arm, SPACE off|\r\n",
             demo_armed ? "ARMED" : "DISARMED", t.r, demo_travel, DEMO_TRAVEL_TICKS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin|MOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin MCP2515_CS_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin|MCP2515_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// One 5 ms control step: read r, integrate psi, outer loop every 10th step, inner loop, drive the L298N.
static void control_step(void)
{
  static float r, psi, delta_cmd, dk[3], k[3];
  static int16_t acc[3];
  static uint32_t tick, fails;

  if (imu_read_yaw_rate_dps(&r, acc) == HAL_OK)
    fails = 0;
  else if (++fails >= IMU_MAX_FAILS)  // keep the last r for a few bad reads, then stop rather than steer blind
  {
    motor_set(0.0f);
    test_u = 0.0f;  // modes 1/2: don't spin back up on their own when the IMU recovers
    demo_armed = 0;
    telem.u = 0.0f;
    telem.failsafe = 1;
    return;
  }
  if (fabsf(r) < GYRO_DEADBAND_DPS)
    r = 0.0f;

  psi = remainderf(psi + r * TS_INNER, 360.0f);  // heading estimate, wrapped to +-180
  float e = remainderf(PSI_D - psi, 360.0f);

  if (++tick >= OUTER_DIV)
  {
    tick = 0;
    delta_cmd = outer_step(e, r, dk, k);
  }

#if BENCH_TEST_NO_ENCODER
  float delta = 0.0f;
#else
#error "No steering angle sensor yet: read it into delta [deg] here"
#endif
  float u = inner_step(delta_cmd, delta);
  if (MOTOR_MODE == 1)
    u = test_ramp(clampf(test_u, -MOTOR_TEST_MAX, MOTOR_TEST_MAX));  // the keys drive the motor; the controller only feeds the dashboard
  else if (MOTOR_MODE == 2)
    u = demo_step(r);  // follow the turn
  motor_set(u);

  telem = (telem_t){r, psi, e, {dk[0], dk[1], dk[2]}, {k[0], k[1], k[2]}, delta_cmd, u, {acc[0], acc[1], acc[2]}};
}

// TIM3 200 Hz, prio 5. Blocking I2C is fine here: SysTick (prio 0) preempts it, so HAL timeouts
// still advance, and nothing else touches I2C1 once the loop is started.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != CTRL_TIM.Instance)
    return;
  uint32_t t0 = DWT->CYCCNT;
  control_step();
  telem.isr_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);  // both IN low: L298N coasts whatever ENA does
  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
