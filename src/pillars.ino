/*
  WRO Future Engineers 2026 - FINAL Open + Obstacle Challenge
  Arduino Mega 2560

  YAW TIMING FIX VERSION:
    - trapezoidal MPU6050 gyro integration
    - no lost yaw during UART/ultrasonic delays
    - frequent yaw sampling around slow operations

  COMMUNICATION:
    MPU6050 / GY-521 IMU -> I2C
    HUSKYLENS 2         -> UART Serial1

  This is based on the communication test that worked on your hardware.

  Required libraries:
    DFRobot_HuskylensV2
    Adafruit MPU6050
    Adafruit Unified Sensor
    Adafruit BusIO
    Wire
    Servo

  ------------------------------------------------------------
  PIN MAP
  ------------------------------------------------------------

  Encoder A              D2
  Encoder B              D3

  BTS7960 RPWM           D5
  BTS7960 LPWM           D7
  BTS7960 R_EN/L_EN      5V

  Steering servo         D9

  URM09 Front            D22
  URM09 Left             D23
  URM09 Right            D24
  URM09 Rear             D25
  URM09 Rear-Left        D27
  URM09 Rear-Right       D28

  Start button           D26 -> GND
  Buzzer                 D29

  ------------------------------------------------------------
  MPU6050 / GY-521 - I2C
  ------------------------------------------------------------

  MPU6050 SDA            Mega D20 / SDA
  MPU6050 SCL            Mega D21 / SCL
  MPU6050 VCC            5V (common GY-521 breakout)
  MPU6050 GND            GND

  Normal address         0x68
  If AD0 is HIGH         0x69

  ------------------------------------------------------------
  HUSKYLENS 2 - UART
  ------------------------------------------------------------

  HUSKYLENS TX            Mega D19 / RX1
  HUSKYLENS RX            Mega D18 / TX1
  HUSKYLENS GND           Mega GND

  HUSKYLENS Protocol:
    Serial 9600

  USB Serial Monitor:
    115200 baud

  ------------------------------------------------------------
  COLOR RECOGNITION
  ------------------------------------------------------------

  ID 0 = unlearned / white box -> ignored
  ID 1 = BLUE   -> LEFT
  ID 2 = ORANGE -> RIGHT

  After the first completed turn, the direction is permanently
  latched. Opposite-color intersections are ignored.

  3 laps = 12 turns.

  After the 12th completed turn, the robot continues straight
  with yaw + wall centering until it sees the NEXT intersection
  of the latched color, then stops.

  ------------------------------------------------------------
  SERIAL COMMANDS while WAIT or STOP
  ------------------------------------------------------------

  help
  i2c
  us
  imu
  husky
  enc
  button
  motorf
  motorb
  servol
  servoc
  servor
  buzz
  recal
  all
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DFRobot_HuskylensV2.h>

#define VELOCITY_TEMP(temp) (((331.5 + 0.6 * (float)(temp)) * 100.0) / 1000000.0)

// ===================== PINS =====================
const uint8_t PIN_ENCODER_A = 2;
const uint8_t PIN_ENCODER_B = 3;
const uint8_t PIN_BTS_RPWM = 5;
const uint8_t PIN_BTS_LPWM = 7;
const uint8_t PIN_STEERING = 9;
const uint8_t PIN_US_FRONT = 22;
const uint8_t PIN_US_LEFT  = 23;
const uint8_t PIN_US_RIGHT = 24;
const uint8_t PIN_US_REAR  = 25;
const uint8_t PIN_START_BUTTON = 26;

// NEW rear-side sensors for wide rear-wheel clearance.
const uint8_t PIN_US_REAR_LEFT  = 27;
const uint8_t PIN_US_REAR_RIGHT = 28;

const uint8_t PIN_BUZZER = 29;

// ===================== TUNING =====================
unsigned long TELEMETRY_INTERVAL_MS = 100;

int DRIVE_PWM          = 255;
int APPROACH_PWM       = 255;
int TURN_PWM_FAST      = 255;
int TURN_PWM_SLOW      = 255;
int ESCAPE_BACK_PWM    = 255;
int ESCAPE_FORWARD_PWM = 255;
int MOTOR_DIRECTION    = +1;

int SERVO_CENTER_DEG = 98;
// Reversed because your LEFT test went RIGHT.
int SERVO_DIRECTION = -1;
// Increased steering authority.
// With center=98 and SERVO_DIRECTION=-1, +/-50 gives a large
// correction range while keeping the commanded servo angle in range.
float MAX_STEER_DEG       = 60.0;
float CORRIDOR_MAX_STEER_DEG = 55.0;
float TURN_STEER_DEG      = 55.0;
float TEST_STEER_DEG      = 40.0;

float YAW_SIGN = +1.0;          // physical LEFT should increase yaw
float GYRO_Z_BIAS_DPS = 0.0;

// Normal MPU6050 / GY-521
// Read every ~4 ms when the main loop is free.
const unsigned long IMU_READ_INTERVAL_US = 4000UL;

// Gyro calibration at startup.
const uint16_t QUICK_GYRO_SAMPLES = 300;
const uint8_t QUICK_GYRO_DELAY_MS = 3;

// Yaw correction tuning.
// Start with 1.000. If a real 90 deg rotation reads 84 deg:
// YAW_SCALE = 90.0 / 84.0 = 1.0714
float YAW_SCALE = 1.0000;

// Suppress tiny stationary noise after bias subtraction.
// Keep small so slow real turns are still detected.
float YAW_GYRO_DEADBAND_DPS = 0.12;

// ==========================================================
// AGGRESSIVE CORRIDOR CORRECTION
// ==========================================================
//
// Increase YAW_KP for faster heading recovery.
// Increase WALL_KP for faster left/right centering.
float YAW_KP = 4.20;
float YAW_KD = 0.16;

// SIDE ULTRASONIC CENTERING
// Positive software steering = physical RIGHT with SERVO_DIRECTION=-1.
float WALL_KP = 4.60;
float WALL_MAX_CORRECTION_DEG = 50.0;

float SIDE_VALID_MIN_CM = 3.0;
float SIDE_VALID_MAX_CM = 250.0;

// Smaller deadband = reacts sooner to side-distance difference.
float SIDE_CENTER_DEADBAND_CM = 0.5;

// HARD WALL AVOID:
// If a side wall is closer than this distance, use a strong fixed
// steering command away from it.
float WALL_AVOID_TRIGGER_CM = 25.0;
float WALL_AVOID_STEER_DEG  = 50.0;

// For Serial telemetry.
float lastWallCorrectionDeg = 0.0;
bool wallAvoidActive = false;

// ==========================================================
// CORNER TURN TRIGGERS
// ==========================================================
//
// After HUSKYLENS sees the intersection, the robot keeps going
// straight on the OLD yaw until a corner trigger becomes true.
//
// Encoder trigger, separately tunable for Corner 1..4:
float CORNER_FORWARD_CM[4] = {
  0.0,   // Corner 1
  58.0,   // Corner 2
  58.0,   // Corner 3
  0.0    // Corner 4
};

// FRONT ULTRASONIC TURN DISTANCE, separately tunable for Corner 1..4.
// Example: 16.0 means START THE 90-DEG TURN when front <= 16 cm.
float CORNER_FRONT_TURN_CM[4] = {
  155.0,   // Corner 1
  41.0,   // Corner 2
  41.0,   // Corner 3
  155.0    // Corner 4
};

// Keep both true to preserve the current behavior:
// turn when EITHER encoder distance OR front distance is reached.
bool USE_ENCODER_CORNER_TRIGGER = true;
bool USE_FRONT_CORNER_TRIGGER   = true;

// ==========================================================
// OPTIONAL BACKWARD DISTANCE BEFORE NORMAL CORNER TURN
// ==========================================================
//
// These 4 values repeat every lap:
//   Turn 1, 5, 9  -> index 0 / Corner 1
//   Turn 2, 6, 10 -> index 1 / Corner 2
//   Turn 3, 7, 11 -> index 2 / Corner 3
//   Turn 4, 8     -> index 3 / Corner 4
//
// NOTE:
// Turn 12 is handled by the SPECIAL FINAL PARKING sequence and
// continues to use FINAL_BACK_CM. It is intentionally NOT given
// an extra reverse here, so it does not reverse twice.
//
// Set false to disable ALL normal-corner pre-turn reversing.
bool ENABLE_BACK_BEFORE_TURN = true;

// Encoder reverse distance before each of the 4 normal corner positions.
// Set any individual value to 0.0 to disable reverse only for that corner.
float BACK_BEFORE_TURN_CM[4] = {
  17.0,   // Corner 1
  0.0,   // Corner 2
  0.0,   // Corner 3
  17.0    // Corner 4
};

// Reverse speed used only for this pre-turn movement.
int BACK_BEFORE_TURN_PWM = 150;

// Safety timeout so a bad encoder cannot leave the robot reversing forever.
unsigned long BACK_BEFORE_TURN_TIMEOUT_MS = 2000;

// ==========================================================
// SIDE OPENING REQUIRED BEFORE EACH TURN
// ==========================================================
//
// LEFT turn:
//   left ultrasonic must be >= TURN_SIDE_OPEN_MIN_CM
//
// RIGHT turn:
//   right ultrasonic must be >= TURN_SIDE_OPEN_MIN_CM
//
// The robot will keep moving forward in APPROACH until BOTH:
//   1) normal corner trigger is reached
//      (encoder and/or front ultrasonic)
//   2) the side of the intended turn is open enough
//
// Set USE_SIDE_OPEN_TURN_GATE=false to disable this condition.
bool USE_SIDE_OPEN_TURN_GATE = true;
float TURN_SIDE_OPEN_MIN_CM = 30.0;

// ==========================================================
// ULTRASONIC FALLBACK CORNER TURN
// ==========================================================
//
// Backup when HUSKYLENS misses the blue/orange intersection.
//
// Turn starts when BOTH:
//   1) front <= CORNER_FRONT_TURN_CM[current corner]
//   2) turn-side distance >= ULTRASONIC_FALLBACK_SIDE_MIN_CM
//
// Once round direction is known:
//   LEFT round  -> only LEFT side can trigger fallback.
//   RIGHT round -> only RIGHT side can trigger fallback.
//
// Before first direction is known:
//   one open side -> use it
//   both open -> use the larger side distance
bool ENABLE_ULTRASONIC_CORNER_FALLBACK = true;

// User-requested default.
float ULTRASONIC_FALLBACK_SIDE_MIN_CM = 100.0;

// Stops the same corner from immediately retriggering.
unsigned long ULTRASONIC_FALLBACK_COOLDOWN_MS = 900;

// ==========================================================
// POST-CORNER ONE-SHOT LOCK / REARM
// ==========================================================
//
// FIX FOR REPEATED/ENDLESS CORNER TURNS:
// After one 90-degree turn completes, do NOT allow the same physical
// corner to trigger another turn immediately.
//
// The corner system re-arms after:
//   - minimum time has passed,
//   - robot has moved at least CORNER_REARM_FORWARD_CM,
//   - AND the old turn-side opening has closed,
//     OR the robot has moved CORNER_REARM_FORCE_CM as a fallback.
unsigned long CORNER_REARM_MIN_MS = 600;
float CORNER_REARM_FORWARD_CM = 35.0;
float CORNER_REARM_FORCE_CM = 60.0;
float CORNER_REARM_SIDE_CLOSED_CM = 85.0;

// ==========================================================
// FINAL STOP AFTER TURN 12
// ==========================================================
//
// After the 12th completed turn the robot keeps driving with
// normal yaw + side-wall centering.
//
// It stops when:
//      front ultrasonic <= FINAL_STOP_FRONT_CM
//
// Tune this value freely.
float FINAL_STOP_FRONT_CM = 70.0;

float TURN_ANGLE_DEG = 90.0;
float TURN_TOLERANCE_DEG = 3.0;
float TURN_SLOW_ZONE_DEG = 25.0;
unsigned long TURN_SETTLE_MS = 50;
unsigned long TURN_TIMEOUT_MS = 7000;

const int HUSKY_BLUE_ID   = 1;
const int HUSKY_ORANGE_ID = 2;
int HUSKY_LINE_MIN_Y = 80;
long HUSKY_LINE_MIN_AREA = 100;
unsigned long HUSKY_POLL_INTERVAL_MS = 100;   // UART 9600
unsigned long LINE_COOLDOWN_MS = 850;
unsigned long HUSKY_FRAME_STALE_MS = 350;

// ==========================================================
// PILLAR DETECTION - SAME CODE FOR OPEN + OBSTACLE
// ==========================================================
//
// Open Challenge:
//   no RED/GREEN pillar is present -> this logic does nothing.
//
// Obstacle Challenge:
//   RED/GREEN pillars are detected automatically.
//   RED pillars are avoided to the RIGHT.
//   GREEN pillars are avoided to the LEFT.
//   Multiple learned IDs can be assigned to each pillar color.
//
// MULTI-ID PILLAR TRAINING
//
// Keep your current IDs and add more learned IDs inside the braces.
//
// Example:
//   RED   -> {5, 7, 9}
//   GREEN -> {4, 6, 8}
//
// Current IDs from your file:
const int HUSKY_RED_PILLAR_IDS[] = {
  5
  ,
  3
};

const int HUSKY_GREEN_PILLAR_IDS[] = {
  4
  ,
  6
};

const uint8_t HUSKY_RED_PILLAR_ID_COUNT =
  sizeof(HUSKY_RED_PILLAR_IDS) /
  sizeof(HUSKY_RED_PILLAR_IDS[0]);

const uint8_t HUSKY_GREEN_PILLAR_ID_COUNT =
  sizeof(HUSKY_GREEN_PILLAR_IDS) /
  sizeof(HUSKY_GREEN_PILLAR_IDS[0]);

