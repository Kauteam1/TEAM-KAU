bool isRedPillarID(int id) {
  for (
    uint8_t i = 0;
    i < HUSKY_RED_PILLAR_ID_COUNT;
    i++
  ) {
    if (id == HUSKY_RED_PILLAR_IDS[i])
      return true;
  }

  return false;
}

bool isGreenPillarID(int id) {
  for (
    uint8_t i = 0;
    i < HUSKY_GREEN_PILLAR_ID_COUNT;
    i++
  ) {
    if (id == HUSKY_GREEN_PILLAR_IDS[i])
      return true;
  }

  return false;
}

// ==========================================================
// SMART PILLAR AVOIDANCE - ONLY 5 MAIN TUNES
// ==========================================================
//
// 1) DETECTION DISTANCE
// Higher = pillar must look larger -> detect later/closer.
// Lower  = detect earlier/farther.
float PILLAR_TUNE_NEAR_AREA = 1000.0;

// 2) MAXIMUM AVOIDANCE YAW
// Higher = more sideways clearance from dangerous pillars.
float PILLAR_TUNE_MAX_TURN_DEG = 50.0;

// 3) PILLAR PASS SPEED
// This is the normal upper PWM during the maneuver.
// The code automatically slows down more when danger is high.
int PILLAR_TUNE_PASS_PWM = 255;

// 4) STEERING SMOOTHNESS
// Lower = softer/slower servo changes.
// Higher = faster/more aggressive response.
float PILLAR_TUNE_SMOOTHNESS_DPS = 130.0;

// 5) REAR-WHEEL SAFETY DISTANCE
// AFTER the rear-side ultrasonic confirms the pillar has passed,
// continue this small encoder distance before returning to corridor yaw.
float PILLAR_TUNE_REAR_SAFETY_CM = 1.0;


// ==========================================================
// INTERNAL SMART-PILLAR SETTINGS
// Normally DO NOT tune below this line.
// ==========================================================

bool ENABLE_PILLAR_AVOIDANCE = true;

// RED   -> robot passes on RIGHT -> pillar is on LEFT side.
// GREEN -> robot passes on LEFT  -> pillar is on RIGHT side.

// Camera near gate.
long HUSKY_PILLAR_NEAR_MIN_AREA =
  (long)PILLAR_TUNE_NEAR_AREA;

int HUSKY_PILLAR_NEAR_MIN_Y = 115;
int HUSKY_PILLAR_NEAR_MIN_BOTTOM_Y = 165;
unsigned long HUSKY_PILLAR_FRAME_STALE_MS = 180;

// Camera confidence:
// pillar must be geometrically consistent over multiple frames.
int PILLAR_CONFIDENCE_TRIGGER = 2;
int PILLAR_CONFIDENCE_MAX = 12;

// X-position calibration.
float PILLAR_RED_NO_AVOID_X = 180.0;
float PILLAR_RED_FULL_AVOID_X = 430.0;

float PILLAR_GREEN_NO_AVOID_X = 500.0;
float PILLAR_GREEN_FULL_AVOID_X = 350.0;

// Close visual cues only BOOST an already dangerous X position.
long PILLAR_FORCE_AVOID_AREA = 800;
int PILLAR_FORCE_AVOID_HEIGHT = 55;
int PILLAR_FORCE_AVOID_BOTTOM_Y = 190;

float PILLAR_CLOSE_BOOST_START_SEVERITY = 0.25;
float PILLAR_CLOSE_SEVERITY_BOOST = 0.25;

// Keep the short stop before the maneuver.
bool PILLAR_STOP_BEFORE_AVOID = true;
unsigned long PILLAR_STOP_MS = 120;

// Avoidance yaw.
float PILLAR_MIN_TURN_OUT_DEG =
  PILLAR_TUNE_MAX_TURN_DEG * 0.10;

float PILLAR_MAX_TURN_OUT_DEG =
  PILLAR_TUNE_MAX_TURN_DEG;

float PILLAR_SKIP_BELOW_DEG = 7.0;

// Smooth steering.
float PILLAR_TURN_STEER_DEG = 50.0;
float PILLAR_TURN_MIN_STEER_DEG = 12.0;
float PILLAR_TURN_SLOW_ZONE_DEG = 25.0;

float PILLAR_STEER_SLEW_DEG_PER_SEC =
  PILLAR_TUNE_SMOOTHNESS_DPS;

// Adaptive speed.
// High danger slows the robot automatically.
int PILLAR_SPEED_MAX_PWM =
  PILLAR_TUNE_PASS_PWM;

int PILLAR_SPEED_MIN_PWM =
  (int)(PILLAR_TUNE_PASS_PWM * 0.68);

// Side-ultrasonic automatic pass detection.
// No fixed "pillar distance" is required.
float PILLAR_SIDE_ENTER_RATIO = 0.82;
float PILLAR_SIDE_EXIT_RATIO = 1.45;
int PILLAR_SIDE_CONFIRM_READS = 3;

// Require a meaningful drop from the starting wall/background distance
// before deciding a side sensor has actually seen the pillar.
float PILLAR_SIDE_MIN_DROP_CM = 4.0;

// Rear wheel protection.
float PILLAR_REAR_SAFETY_CM =
  PILLAR_TUNE_REAR_SAFETY_CM;

// Encoder fallback only if a side sensor is unavailable/noisy.
float PILLAR_PASS_FALLBACK_CM = 4.0;

// Yaw-only correction while beside the pillar.
float PILLAR_YAW_KP = 4.20;
float PILLAR_YAW_KD = 0.10;
float PILLAR_FORWARD_MAX_STEER_DEG = 50.0;

float PILLAR_YAW_TOLERANCE_DEG = 3.0;
unsigned long PILLAR_TURN_TIMEOUT_MS = 3500;
unsigned long PILLAR_PASS_TIMEOUT_MS = 5000;
unsigned long PILLAR_COOLDOWN_MS = 1200;
unsigned long PILLAR_SAFE_RECHECK_MS = 120;


float ULTRASONIC_TEMP_C = 20.0;
unsigned long URM09_PULSE_TIMEOUT_US = 18000UL;
unsigned long SONAR_SLOT_INTERVAL_MS = 12;
float SONAR_FILTER_OLD_WEIGHT = 0.15;

// Calibrate for your NEW motor/encoder.
float ENCODER_COUNTS_PER_WHEEL_REV = 700.0;
float WHEEL_DIAMETER_CM = 6.5;

int STUCK_MIN_MOTOR_PWM = 90;
unsigned long STUCK_CHECK_WINDOW_MS = 500;
long STUCK_MIN_ENCODER_COUNTS = 3;
float ESCAPE_BACK_CM = 10.0;
float ESCAPE_FORWARD_CM = 14.0;
float ESCAPE_STEER_DEG = 18.0;
unsigned long ESCAPE_BACK_TIMEOUT_MS = 1600;
unsigned long ESCAPE_FORWARD_TIMEOUT_MS = 1800;

int TOTAL_TURNS = 12;

// ==========================================================
// PARKING EXIT + FINAL PARKING
// ==========================================================
//
// Direction is determined BEFORE moving:
//   LEFT ultrasonic > RIGHT ultrasonic -> LEFT race (+1)
//   RIGHT ultrasonic > LEFT ultrasonic -> RIGHT race (-1)
//
// Straight parking movements use encoder distance.
// Parking/final alignment uses yaw only: NO wall centering.

// ---------- Start behavior ----------
//
// NO parking-escape routine.
// After Start:
//   - yaw is reset to 0
//   - robot drives straight with yaw correction
//   - BLUE camera marker decides LEFT first turn
//   - ORANGE camera marker decides RIGHT first turn
//
// The race direction is therefore learned from the first camera corner,
// not from the side ultrasonics.

// ---------- Special final corner / parking ----------
float FINAL_WALL_FRONT_CM = 90.0;

// After reaching 5 cm from the wall, reverse this encoder distance.
float FINAL_BACK_CM = 35.0;
int FINAL_BACK_PWM = 150;

// This IS the final normal corner turn.
float FINAL_CORRIDOR_TURN_DEG = 90.0;

// After the final 90-degree turn:
// go forward with YAW ONLY until front <= this value.
float FINAL_CENTER_FRONT_CM = 250.0;

// Small S-curve/positioning maneuver before parking.
// Default direction is opposite to the normal corner direction.
// Change -1 to +1 if your physical parking is on the other side.
float FINAL_SHIFT_ANGLE_DEG = 0.0;

// After reaching the 45-degree shift angle, move forward this
// encoder distance WHILE HOLDING the 45-degree yaw.
float FINAL_SHIFT_FORWARD_CM = 58.0;

// SAME direction as race:
// LEFT race -> 45 LEFT
// RIGHT race -> 45 RIGHT
int FINAL_SHIFT_DIRECTION_MULTIPLIER = +1;

// After returning to the main corridor yaw, move this encoder distance.
float FINAL_BEFORE_PARK_CM = 0.0;

// Turn into the parking slot in the OPPOSITE direction:
// LEFT race -> parking entry RIGHT
// RIGHT race -> parking entry LEFT
float FINAL_PARK_ENTRY_DEG = 90.0;
int FINAL_PARK_ENTRY_DIRECTION_MULTIPLIER = -1;

// Stop inside parking when front <= this distance.
float FINAL_PARK_STOP_FRONT_CM = 10.0;

int FINAL_APPROACH_PWM = 160;
int FINAL_FORWARD_PWM = 160;
int FINAL_TURN_PWM = 165;

float FINAL_YAW_KP = 4.0;
float FINAL_YAW_MAX_STEER_DEG = 35.0;

float FINAL_TURN_MIN_STEER_DEG = 16.0;
float FINAL_TURN_MAX_STEER_DEG = 45.0;
float FINAL_TURN_SLOW_ZONE_DEG = 25.0;
float FINAL_TURN_TOLERANCE_DEG = 3.0;

// ===================== OBJECTS =====================
Servo steeringServo;
HuskylensV2 huskylens;
Adafruit_MPU6050 mpu;

// ===================== STATES =====================
enum RobotState {
  STATE_WAIT_START,

  STATE_CORRIDOR,
  STATE_APPROACH_CORNER,
  STATE_PRE_TURN_BACK,
  STATE_TURNING,
  STATE_ESCAPE_BACK,
  STATE_ESCAPE_FORWARD,

  // Pillar avoidance states
  STATE_PILLAR_STOP,
  STATE_PILLAR_TURN_OUT,
  STATE_PILLAR_FORWARD,
  STATE_PILLAR_RETURN_YAW,

  // Special final corner + return to parking
  STATE_FINAL_TO_WALL,
  STATE_FINAL_BACK,
  STATE_FINAL_CORRIDOR_TURN,
  STATE_FINAL_CENTER_FORWARD,
  STATE_FINAL_SHIFT_TURN,
  STATE_FINAL_SHIFT_FORWARD,
  STATE_FINAL_RETURN_MAIN_YAW,
  STATE_FINAL_BEFORE_PARK,
  STATE_FINAL_PARK_ENTRY_TURN,
  STATE_FINAL_PARK_FORWARD,

  // Kept for compatibility; special final parking replaces old behavior.
  STATE_FINAL_FIND_INTERSECTION,

  STATE_STOPPED
};
RobotState state = STATE_WAIT_START;
RobotState stateBeforeEscape = STATE_CORRIDOR;

// ===================== FINAL PARK GLOBALS =====================
// Set from the first camera-detected corner:
//   +1 = LEFT race (BLUE)
//   -1 = RIGHT race (ORANGE)
int parkingDirection = 0;

// Final parking sequence
float finalPreCornerYaw = 0.0;
float finalMainCorridorYaw = 0.0;
float finalShiftYaw = 0.0;
float finalParkEntryYaw = 0.0;

int finalShiftDirection = 0;
int finalParkEntryDirection = 0;

long finalStageStartTicks = 0;

// Forward declaration because ultrasonic fallback can start the
// special final sequence before its full function definition.
void startFinalParkingSequence();

bool imuOK = false;
bool huskyOK = false;
unsigned long lastReconnectAttemptMs = 0;

// ===================== ENCODER =====================
volatile long encoderTicks = 0;
void encoderISR() {
  if (digitalRead(PIN_ENCODER_B)) encoderTicks++;
  else encoderTicks--;
}
long getEncoderTicks() {
  noInterrupts();
  long v = encoderTicks;
  interrupts();
  return v;
}
void resetEncoderTicks() {
  noInterrupts(); encoderTicks = 0; interrupts();
}
float encoderCountsPerCM() {
  float c = PI * WHEEL_DIAMETER_CM;
  return (c > 0.0) ? ENCODER_COUNTS_PER_WHEEL_REV / c : 1.0;
}
float encoderDistanceCMFrom(long startTicks) {
  return (float)labs(getEncoderTicks() - startTicks) / encoderCountsPerCM();
}

// ===================== MOTOR =====================
int lastMotorCommand = 0;
void setMotor(int command) {
  command = constrain(command, -255, 255) * MOTOR_DIRECTION;
  lastMotorCommand = command;
  if (command > 0) {
    analogWrite(PIN_BTS_RPWM, command);
    analogWrite(PIN_BTS_LPWM, 0);
  } else if (command < 0) {
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, -command);
  } else {
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);
  }
}
void stopMotor() { setMotor(0); }

// ===================== STEERING =====================
float lastSteeringCommand = 0.0;
void setSteering(float commandDeg) {
  commandDeg = constrain(commandDeg, -MAX_STEER_DEG, +MAX_STEER_DEG);
  lastSteeringCommand = commandDeg;
  float angle = SERVO_CENTER_DEG + SERVO_DIRECTION * commandDeg;
  angle = constrain(angle, 0.0, 180.0);
  steeringServo.write((int)angle);
}
void centerSteering() { setSteering(0.0); }

// ===================== BUZZER =====================
void buzzerStart()        { tone(PIN_BUZZER, 2200, 90); }

// Two-tone signal: initialization is complete and robot is waiting
// for the physical START button.
void buzzerReady() {
  tone(PIN_BUZZER, 2400, 120);
  delay(160);
  tone(PIN_BUZZER, 3000, 180);
  delay(220);
}
void buzzerBlue()         { tone(PIN_BUZZER, 2600, 90); }
void buzzerOrange()       { tone(PIN_BUZZER, 1800, 90); }
void buzzerTurnComplete() { tone(PIN_BUZZER, 2900, 90); }
void buzzerFinish()       { tone(PIN_BUZZER, 3200, 300); }
void buzzerStuck()        { tone(PIN_BUZZER, 900, 250); }
unsigned long lastTurnBuzzerMs = 0;
void buzzerTurningTick() {
  if (millis() - lastTurnBuzzerMs >= 250) {
    lastTurnBuzzerMs = millis();
    tone(PIN_BUZZER, 1500, 25);
  }
}

// ===================== ULTRASONIC =====================
//
// Existing:
//   frontCM, leftCM, rightCM, rearCM
//
// New:
//   rearLeftCM, rearRightCM
//
// The new rear-side sensors should point SIDEWAYS and be mounted
// close to / just ahead of the widest rear-wheel area.
float frontCM = 999.0;
float leftCM = 999.0;
float rightCM = 999.0;
float rearCM = 999.0;
float rearLeftCM = 999.0;
float rearRightCM = 999.0;

float readURM09(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);

  uint32_t pulseWidthUs =
    pulseIn(
      pin,
      HIGH,
      URM09_PULSE_TIMEOUT_US
    );

  if (pulseWidthUs == 0)
    return -1.0;

  float distance =
    pulseWidthUs *
    VELOCITY_TEMP(ULTRASONIC_TEMP_C) /
    2.0;

  if (
    distance < 2.0 ||
    distance > 500.0
  ) {
    return -1.0;
  }

  return distance;
}

float filterSonar(
  float oldValue,
  float newValue
) {
  if (newValue < 0.0)
    return oldValue;

  if (oldValue > 500.0)
    return newValue;

  return
    oldValue *
      SONAR_FILTER_OLD_WEIGHT +
    newValue *
      (1.0 - SONAR_FILTER_OLD_WEIGHT);
}

uint8_t sonarSlot = 0;
unsigned long lastSonarSlotMs = 0;

void updateUltrasonics() {
  if (
    millis() -
    lastSonarSlotMs <
    SONAR_SLOT_INTERVAL_MS
  ) {
    return;
  }

  lastSonarSlotMs =
    millis();

  float m = -1.0;

  switch (sonarSlot) {
    case 0:
      m = readURM09(PIN_US_FRONT);
      frontCM = filterSonar(frontCM, m);
      break;

    case 1:
      m = readURM09(PIN_US_LEFT);
      leftCM = filterSonar(leftCM, m);
      break;

    case 2:
      m = readURM09(PIN_US_RIGHT);
      rightCM = filterSonar(rightCM, m);
      break;

    case 3:
      m = readURM09(PIN_US_REAR_LEFT);
      rearLeftCM = filterSonar(rearLeftCM, m);
      break;

    case 4:
      m = readURM09(PIN_US_REAR_RIGHT);
      rearRightCM = filterSonar(rearRightCM, m);
      break;

    case 5:
      m = readURM09(PIN_US_REAR);
      rearCM = filterSonar(rearCM, m);
      break;
  }

  sonarSlot =
    (sonarSlot + 1) % 6;
}

void readAllUltrasonicsNow() {
  float v;

  v = readURM09(PIN_US_FRONT);
  if (v > 0) frontCM = v;
  delay(18);

  v = readURM09(PIN_US_LEFT);
  if (v > 0) leftCM = v;
  delay(18);

  v = readURM09(PIN_US_RIGHT);
  if (v > 0) rightCM = v;
  delay(18);

  v = readURM09(PIN_US_REAR_LEFT);
  if (v > 0) rearLeftCM = v;
  delay(18);

  v = readURM09(PIN_US_REAR_RIGHT);
  if (v > 0) rearRightCM = v;
  delay(18);

  v = readURM09(PIN_US_REAR);
  if (v > 0) rearCM = v;
}

// ===================== MPU6050 / YAW =====================
// Uses the exact Adafruit MPU6050 method that already worked
// on your Arduino Mega.

float currentYaw = 0.0;
float targetYaw = 0.0;

float latestAccelX = 0.0;
float latestAccelY = 0.0;
float latestAccelZ = 0.0;

float latestGyroX = 0.0;
float latestGyroY = 0.0;
float latestGyroZ = 0.0;

float latestMPUTempC = 0.0;

unsigned long lastIMUMicros = 0;
unsigned long lastIMUReadMicros = 0;

float previousCorrectedGyroZ = 0.0;
bool havePreviousGyroSample = false;

bool initializeIMU() {
  Serial.println(F("# Starting MPU6050..."));

  // Same simple begin() method from your working test.
  if (!mpu.begin()) {
    imuOK = false;
    Serial.println(F("# MPU6050 begin FAILED"));
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  imuOK = true;
  Serial.println(F("# MPU6050 CONNECTED"));

  return true;
}

bool readIMUData() {
  if (!imuOK)
    return false;

  sensors_event_t a, g, temp;

  if (!mpu.getEvent(&a, &g, &temp))
    return false;

  latestAccelX = a.acceleration.x;
  latestAccelY = a.acceleration.y;
  latestAccelZ = a.acceleration.z;

  // Adafruit returns rad/s. Convert to deg/s.
  latestGyroX = g.gyro.x * 180.0 / PI;
  latestGyroY = g.gyro.y * 180.0 / PI;
  latestGyroZ = g.gyro.z * 180.0 / PI;

  latestMPUTempC = temp.temperature;

  return true;
}

bool readGyroZ(float &gz) {
  if (!readIMUData())
    return false;

  gz = latestGyroZ;
  return true;
}

void calibrateGyroQuick() {
  if (!imuOK)
    return;

  Serial.println(F("# MPU CALIBRATION - KEEP ROBOT STILL"));

  float sum = 0.0;
  uint16_t good = 0;

  for (uint16_t i = 0; i < QUICK_GYRO_SAMPLES; i++) {
    float gz;

    if (readGyroZ(gz)) {
      sum += gz;
      good++;
    }

    delay(QUICK_GYRO_DELAY_MS);
  }

  if (good > 0)
    GYRO_Z_BIAS_DPS = sum / (float)good;
  else
    GYRO_Z_BIAS_DPS = 0.0;

  currentYaw = 0.0;

  lastIMUMicros = micros();
  lastIMUReadMicros = lastIMUMicros;

  previousCorrectedGyroZ = 0.0;
  havePreviousGyroSample = false;

  Serial.print(F("# MPU GYRO_Z_BIAS="));
  Serial.println(GYRO_Z_BIAS_DPS, 5);
}

void resetYaw() {
  currentYaw = 0.0;
  targetYaw = 0.0;

  unsigned long now = micros();

  lastIMUMicros = now;
  lastIMUReadMicros = now;

  previousCorrectedGyroZ = 0.0;
  havePreviousGyroSample = false;
}

void updateYaw() {
  if (!imuOK)
    return;

  unsigned long scheduleNow = micros();

  // Rate limit only when the loop is running very fast.
  if ((unsigned long)(scheduleNow - lastIMUReadMicros) <
      IMU_READ_INTERVAL_US)
    return;

  lastIMUReadMicros = scheduleNow;

  float gz;

  if (!readGyroZ(gz))
    return;

  // Timestamp AFTER the sensor read.
  unsigned long now = micros();

  float correctedGyroZ =
    gz - GYRO_Z_BIAS_DPS;

  // Remove only very small stationary noise.
  if (fabs(correctedGyroZ) < YAW_GYRO_DEADBAND_DPS)
    correctedGyroZ = 0.0;

  if (!havePreviousGyroSample) {
    previousCorrectedGyroZ = correctedGyroZ;
    havePreviousGyroSample = true;
    lastIMUMicros = now;
    return;
  }

  float dt =
    (float)(now - lastIMUMicros) /
    1000000.0f;

  lastIMUMicros = now;

  // In the full robot, UART/ultrasonic operations can make dt larger
  // than in the standalone MPU test. Do NOT throw away a normal
  // 100-300 ms interval, otherwise yaw loses part of the turn.
  //
  // Ignore only an abnormally huge pause.
  if (dt <= 0.0f || dt > 0.50f) {
    previousCorrectedGyroZ = correctedGyroZ;
    return;
  }

  // Trapezoidal integration:
  // use both the previous and current gyro samples.
  // This is much more accurate when the main loop has uneven timing.
  float averageGyroZ =
    0.5f *
    (previousCorrectedGyroZ + correctedGyroZ);

  currentYaw +=
    YAW_SIGN *
    YAW_SCALE *
    averageGyroZ *
    dt;

  previousCorrectedGyroZ =
    correctedGyroZ;
}

// ===================== CORRIDOR CONTROL =====================
float previousYawError = 0.0;
unsigned long lastControlMicros = 0;
bool validSideDistance(float cm) {
  return cm >= SIDE_VALID_MIN_CM && cm <= SIDE_VALID_MAX_CM;
}

// Dedicated validity check for detecting a large corner opening.
// Do NOT use SIDE_VALID_MAX_CM here because that limit is for normal
// corridor-wall centering. A real opening may be much farther away.
bool validTurnOpeningDistance(float cm) {
  return cm > 0.0 && cm < 500.0;
}
float headingCorrection() {
  float error = targetYaw - currentYaw;
  unsigned long now = micros();
  float dt = (float)(now - lastControlMicros) / 1000000.0f;
  if (dt <= 0.0 || dt > 0.20) dt = 0.02;
  lastControlMicros = now;
  float derivative = (error - previousYawError) / dt;
  previousYawError = error;
  return -(YAW_KP * error + YAW_KD * derivative);
}
float wallCorrection() {
  bool leftValid =
    validSideDistance(leftCM);

  bool rightValid =
    validSideDistance(rightCM);

  wallAvoidActive = false;

  // --------------------------------------------------------
  // HARD WALL AVOID
  // --------------------------------------------------------
  // Too close to LEFT wall -> physical steer RIGHT
  // -> positive software command because SERVO_DIRECTION=-1.
  if (
    leftValid &&
    leftCM <= WALL_AVOID_TRIGGER_CM &&
    (
      !rightValid ||
      leftCM < rightCM
    )
  ) {
    wallAvoidActive = true;

    lastWallCorrectionDeg =
      +WALL_AVOID_STEER_DEG;

    return lastWallCorrectionDeg;
  }

  // Too close to RIGHT wall -> physical steer LEFT
  // -> negative software command.
  if (
    rightValid &&
    rightCM <= WALL_AVOID_TRIGGER_CM &&
    (
      !leftValid ||
      rightCM < leftCM
    )
  ) {
    wallAvoidActive = true;

    lastWallCorrectionDeg =
      -WALL_AVOID_STEER_DEG;

    return lastWallCorrectionDeg;
  }

  // --------------------------------------------------------
  // NORMAL CORRIDOR CENTERING
  // --------------------------------------------------------
  if (!leftValid || !rightValid) {
    lastWallCorrectionDeg = 0.0;
    return 0.0;
  }

  // right > left means robot is closer to LEFT wall,
  // therefore steer RIGHT.
  float error =
    rightCM -
    leftCM;

  if (
    fabs(error) <
    SIDE_CENTER_DEADBAND_CM
  ) {
    error = 0.0;
  }

  lastWallCorrectionDeg =
    constrain(
      WALL_KP * error,
      -WALL_MAX_CORRECTION_DEG,
      +WALL_MAX_CORRECTION_DEG
    );

  return lastWallCorrectionDeg;
}

float corridorSteering() {
  float command =
    headingCorrection() +
    wallCorrection();

  return constrain(
    command,
    -CORRIDOR_MAX_STEER_DEG,
    +CORRIDOR_MAX_STEER_DEG
  );
}

// ===================== HUSKYLENS =====================
bool initializeHusky() {
  // HUSKYLENS is NOT on I2C anymore.
  // Mega hardware UART:
  // RX1 = D19
  // TX1 = D18
  Serial1.begin(9600);

  delay(500);

  huskyOK =
    huskylens.begin(
      Serial1
    );

  if (!huskyOK)
    return false;

  huskyOK =
    huskylens.switchAlgorithm(
      ALGORITHM_COLOR_RECOGNITION
    );

  return huskyOK;
}

int huskyResultCount = 0;
int huskyLearnedCount = 0;
int huskyBestLearnedID = 0;
int huskyBestX = 0, huskyBestY = 0, huskyBestW = 0, huskyBestH = 0;
int huskyBestIntersectionDirection = 0;
unsigned long lastHuskyPollMs = 0;
unsigned long lastGoodHuskyFrameMs = 0;

// Best valid RED/GREEN pillar from the latest HUSKYLENS frame.
int huskyBestPillarID = 0;
int huskyBestPillarX = 0;
int huskyBestPillarY = 0;
int huskyBestPillarW = 0;
int huskyBestPillarH = 0;
long huskyBestPillarArea = 0;
unsigned long lastGoodPillarFrameMs = 0;

// Smart pillar confidence. Avoidance does not start from one noisy frame.
int pillarConfidenceScore = 0;
int pillarConfidenceID = 0;
int pillarConfidencePrevX = 0;
int pillarConfidencePrevBottomY = 0;
long pillarConfidencePrevArea = 0;

void updateHusky() {
  if (!huskyOK) return;
  if (millis() - lastHuskyPollMs < HUSKY_POLL_INTERVAL_MS) return;
  lastHuskyPollMs = millis();

  // Actual current DFRobot source: -1 means communication failure,
  // 0 means no results, >0 means number of returned results.
  int8_t count = huskylens.getResult(ALGORITHM_COLOR_RECOGNITION);

  if (count < 0) {
    huskyOK = false;
    huskyResultCount = -1;
    huskyLearnedCount = 0;
    huskyBestLearnedID = 0;
    huskyBestIntersectionDirection = 0;

    huskyBestPillarID = 0;
    huskyBestPillarX = 0;
    huskyBestPillarY = 0;
    huskyBestPillarW = 0;
    huskyBestPillarH = 0;
    huskyBestPillarArea = 0;

    pillarConfidenceScore = 0;
    pillarConfidenceID = 0;

    return;
  }

  huskyResultCount = count;
  huskyLearnedCount =
    huskylens.getCachedResultLearnedNum(
      ALGORITHM_COLOR_RECOGNITION
    );

  huskyBestLearnedID = 0;
  huskyBestX = 0;
  huskyBestY = 0;
  huskyBestW = 0;
  huskyBestH = 0;
  huskyBestIntersectionDirection = 0;

  // Build the best pillar for THIS frame locally. If one frame is
  // missed, keep the last valid pillar briefly instead of flickering
  // instantly to NONE.
  int frameBestPillarID = 0;
  int frameBestPillarX = 0;
  int frameBestPillarY = 0;
  int frameBestPillarW = 0;
  int frameBestPillarH = 0;
  long frameBestPillarArea = 0;

  long bestLearnedScore = -1;
  long bestIntersectionScore = -1;

  // Nearest pillar selection:
  // 1) largest box area wins
  // 2) if areas are equal, the box lower in the image wins
  long bestPillarArea = -1;
  int bestPillarBottomY = -1;

  while (huskylens.available(ALGORITHM_COLOR_RECOGNITION)) {
    Result *r =
      huskylens.popCachedResult(
        ALGORITHM_COLOR_RECOGNITION
      );

    if (r == NULL) break;

    // ID 0 = white/unlearned box. Ignore completely.
    if (r->ID == 0) continue;

    long area =
      (long)r->width *
      (long)r->height;

    // Objects lower in the frame receive a larger score because
    // they are generally closer to the robot.
    long score =
      (long)r->yCenter * 3L +
      (long)r->height;

    if (score > bestLearnedScore) {
      bestLearnedScore = score;
      huskyBestLearnedID = r->ID;
      huskyBestX = r->xCenter;
      huskyBestY = r->yCenter;
      huskyBestW = r->width;
      huskyBestH = r->height;
    }

    // --------------------------------------------------------
    // RED / GREEN PILLAR DETECTION
    // --------------------------------------------------------
    // Select the nearest valid learned pillar. The corridor state
    // then starts the adaptive pillar-avoidance maneuver.
    bool isRedPillar =
      isRedPillarID(r->ID);

    bool isGreenPillar =
      isGreenPillarID(r->ID);

    if (isRedPillar || isGreenPillar) {
      int bottomY =
        r->yCenter +
        r->height / 2;

      // FAR pillars are ignored completely.
      bool areaNear =
        area >= HUSKY_PILLAR_NEAR_MIN_AREA;

      bool yNear =
        r->yCenter >= HUSKY_PILLAR_NEAR_MIN_Y;

      bool bottomNear =
        bottomY >= HUSKY_PILLAR_NEAR_MIN_BOTTOM_Y;

      // Earlier but still reliable near-pillar gate:
      // area must be large enough, and either the center or bottom
      // of the box must already be low enough in the image.
      //
      // This avoids the previous late detection caused by waiting
      // for ALL THREE conditions at the same time.
      bool pillarNearEnough =
        areaNear &&
        (
          yNear ||
          bottomNear
        );

      if (
        pillarNearEnough
      ) {
        // Choose ONLY the nearest valid pillar.
        //
        // Largest apparent area is the primary indicator.
        // If two boxes have the same area, choose the one whose
        // bottom edge is lower in the image.
        bool nearer =
          area > bestPillarArea ||
          (
            area == bestPillarArea &&
            bottomY > bestPillarBottomY
          );

        if (nearer) {
          bestPillarArea = area;
          bestPillarBottomY = bottomY;

          frameBestPillarID = r->ID;
          frameBestPillarX = r->xCenter;
          frameBestPillarY = r->yCenter;
          frameBestPillarW = r->width;
          frameBestPillarH = r->height;
          frameBestPillarArea = area;
        }
      }

      // RED/GREEN results are never used as intersection colors.
      continue;
    }

    // --------------------------------------------------------
    // EXISTING BLUE / ORANGE INTERSECTION DETECTION
    // --------------------------------------------------------
    int direction = 0;

    if (r->ID == HUSKY_BLUE_ID)
      direction = +1;
    else if (r->ID == HUSKY_ORANGE_ID)
      direction = -1;
    else
      continue;

    if (area < HUSKY_LINE_MIN_AREA)
      continue;

    if (r->yCenter < HUSKY_LINE_MIN_Y)
      continue;

    if (score > bestIntersectionScore) {
      bestIntersectionScore = score;
      huskyBestIntersectionDirection = direction;
    }
  }

  lastGoodHuskyFrameMs = millis();

  if (frameBestPillarID != 0) {
    int bottomY =
      frameBestPillarY +
      frameBestPillarH / 2;

    // --------------------------------------------------------
    // MULTI-FRAME CONFIDENCE SCORE
    // --------------------------------------------------------
    // Same ID + stable/increasing geometry builds confidence.
    // A single bad frame cannot immediately trigger avoidance.
    if (
      frameBestPillarID ==
      pillarConfidenceID
    ) {
      pillarConfidenceScore += 2;

      // Area should normally stay similar or grow as we approach.
      if (
        pillarConfidencePrevArea <= 0 ||
        frameBestPillarArea >=
          (long)(
            pillarConfidencePrevArea *
            0.82
          )
      ) {
        pillarConfidenceScore += 1;
      }

      // Bottom of box should stay similar or move lower.
      if (
        bottomY >=
        pillarConfidencePrevBottomY - 10
      ) {
        pillarConfidenceScore += 1;
      }

      // X should not teleport across the frame.
      if (
        abs(
          frameBestPillarX -
          pillarConfidencePrevX
        ) <= 120
      ) {
        pillarConfidenceScore += 1;
      }
    }
    else {
      pillarConfidenceID =
        frameBestPillarID;

      pillarConfidenceScore = 2;
    }

    pillarConfidenceScore =
      constrain(
        pillarConfidenceScore,
        0,
        PILLAR_CONFIDENCE_MAX
      );

    pillarConfidencePrevX =
      frameBestPillarX;

    pillarConfidencePrevBottomY =
      bottomY;

    pillarConfidencePrevArea =
      frameBestPillarArea;

    huskyBestPillarID =
      frameBestPillarID;

    huskyBestPillarX =
      frameBestPillarX;

    huskyBestPillarY =
      frameBestPillarY;

    huskyBestPillarW =
      frameBestPillarW;

    huskyBestPillarH =
      frameBestPillarH;

    huskyBestPillarArea =
      frameBestPillarArea;

    lastGoodPillarFrameMs =
      millis();
  }
  else {
    // Decay confidence rather than instantly dropping it.
    if (pillarConfidenceScore > 0)
      pillarConfidenceScore -= 2;

    if (
      millis() -
      lastGoodPillarFrameMs >
      HUSKY_PILLAR_FRAME_STALE_MS
    ) {
      huskyBestPillarID = 0;
      huskyBestPillarX = 0;
      huskyBestPillarY = 0;
      huskyBestPillarW = 0;
      huskyBestPillarH = 0;
      huskyBestPillarArea = 0;

      pillarConfidenceScore = 0;
      pillarConfidenceID = 0;
    }
  }
}

