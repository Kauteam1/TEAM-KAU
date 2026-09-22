bool pillarVisibleNow() {
  if (huskyBestPillarID == 0)
    return false;

  return
    millis() - lastGoodPillarFrameMs <=
    HUSKY_PILLAR_FRAME_STALE_MS;
}

const char* pillarName() {
  if (!pillarVisibleNow())
    return "NONE";

  if (isRedPillarID(huskyBestPillarID))
    return "RED";

  if (isGreenPillarID(huskyBestPillarID))
    return "GREEN";

  return "UNKNOWN";
}

void maintainSensorConnections() {
  // Intentionally disabled.
  //
  // MPU6050 calibration and HUSKYLENS initialization happen ONCE
  // in setup(). Re-running mpu.begin()/calibration can reset yaw and
  // disturb the robot during WAIT/RUN.
}

// ===================== INTERSECTION =====================
unsigned long lastAcceptedLineMs = 0;
int firstTurnDirection = 0; // +1 blue/left, -1 orange/right
int latchedDirection = 0;   // permanent after first completed turn

int readIntersectionEvent() {
  if (millis() - lastAcceptedLineMs < LINE_COOLDOWN_MS) return 0;
  if (millis() - lastGoodHuskyFrameMs > HUSKY_FRAME_STALE_MS) return 0;

  int direction = huskyBestIntersectionDirection;
  if (direction == 0) return 0;

  if (latchedDirection != 0 && direction != latchedDirection) return 0;

  lastAcceptedLineMs = millis();
  return direction;
}

// ===================== CORNER =====================
int turnDirection = 0;
int turnCounter = 0;
int activeCornerIndex = 0;
long cornerApproachStartTicks = 0;
unsigned long turnToleranceStartMs = 0;
unsigned long cornerTurnStartMs = 0;
bool turnTargetReached = false;

// One physical corner = one completed 90-degree turn.
bool cornerTriggerLocked = false;
long cornerRearmStartTicks = 0;
unsigned long cornerFinishedMs = 0;
int lastCompletedTurnDirection = 0;

// Pre-turn reverse runtime.
long preTurnBackStartTicks = 0;
unsigned long preTurnBackStartMs = 0;
float preTurnBackSavedYaw = 0.0;

void acceptIntersection(int detectedDirection) {
  if (turnCounter == 0 && latchedDirection == 0) {
    firstTurnDirection =
      detectedDirection;

    turnDirection =
      firstTurnDirection;

    // Save camera-decided race direction for the final parking routine.
    parkingDirection =
      detectedDirection;

    Serial.print(
      F("# FIRST CAMERA DIRECTION = ")
    );

    Serial.println(
      detectedDirection > 0
        ? F("LEFT / BLUE")
        : F("RIGHT / ORANGE")
    );
  } else {
    if (latchedDirection != 0 && detectedDirection != latchedDirection) return;
    turnDirection = latchedDirection;
  }

  activeCornerIndex = turnCounter % 4;
  cornerApproachStartTicks = getEncoderTicks();

  // One-shot lock starts as soon as this physical corner is accepted.
  cornerTriggerLocked = true;

  if (turnDirection > 0) buzzerBlue();
  else buzzerOrange();

  // Corner mode has priority. Forget any pillar candidate collected
  // before the corner so it cannot interfere with the turn.
  clearPillarCandidateForCorner();

  state = STATE_APPROACH_CORNER;
}

void beginCornerTurn() {
  targetYaw += turnDirection * TURN_ANGLE_DEG;

  previousYawError = 0.0;
  turnToleranceStartMs = 0;
  cornerTurnStartMs = millis();
  turnTargetReached = false;
  lastTurnBuzzerMs = 0;

  Serial.print(F("# TURN START dir="));
  Serial.print(turnDirection > 0 ? F("LEFT") : F("RIGHT"));
  Serial.print(F(" current="));
  Serial.print(currentYaw, 2);
  Serial.print(F(" target="));
  Serial.println(targetYaw, 2);

  state = STATE_TURNING;
}

float preTurnBackYawSteering() {
  // Forward heading correction uses negative Kp for this robot.
  // While reversing, steering/yaw response is inverted.
  float error =
    preTurnBackSavedYaw -
    currentYaw;

  float command =
    YAW_KP *
    error;

  return constrain(
    command,
    -CORRIDOR_MAX_STEER_DEG,
    +CORRIDOR_MAX_STEER_DEG
  );
}

void beginPreTurnBackOrTurn() {
  // The special 12th/final corner already has FINAL_BACK_CM inside
  // startFinalParkingSequence(), so do not add a second reverse.
  if (
    turnCounter >=
    TOTAL_TURNS - 1
  ) {
    startFinalParkingSequence();
    return;
  }

  float backCM =
    BACK_BEFORE_TURN_CM[
      activeCornerIndex
    ];

  if (
    !ENABLE_BACK_BEFORE_TURN ||
    backCM <= 0.0
  ) {
    beginCornerTurn();
    return;
  }

  // Hold the OLD corridor yaw during the reverse.
  preTurnBackSavedYaw =
    targetYaw;

  preTurnBackStartTicks =
    getEncoderTicks();

  preTurnBackStartMs =
    millis();

  stopMotor();
  centerSteering();

  Serial.print(
    F("# PRE-TURN BACK START corner=")
  );
  Serial.print(
    activeCornerIndex + 1
  );

  Serial.print(
    F(" cm=")
  );
  Serial.print(
    backCM,
    1
  );

  Serial.print(
    F(" yaw=")
  );
  Serial.println(
    preTurnBackSavedYaw,
    2
  );

  state =
    STATE_PRE_TURN_BACK;
}

void runPreTurnBack() {
  updateYaw();

  float neededCM =
    BACK_BEFORE_TURN_CM[
      activeCornerIndex
    ];

  float traveledCM =
    encoderDistanceCMFrom(
      preTurnBackStartTicks
    );

  bool done =
    traveledCM >=
    neededCM;

  bool timeout =
    millis() -
    preTurnBackStartMs >=
    BACK_BEFORE_TURN_TIMEOUT_MS;

  if (
    !ENABLE_BACK_BEFORE_TURN ||
    neededCM <= 0.0 ||
    done ||
    timeout
  ) {
    stopMotor();
    centerSteering();

    // Restore exact old yaw reference before creating the new 90-deg target.
    targetYaw =
      preTurnBackSavedYaw;

    Serial.print(
      F("# PRE-TURN BACK END corner=")
    );
    Serial.print(
      activeCornerIndex + 1
    );

    Serial.print(
      F(" traveled=")
    );
    Serial.print(
      traveledCM,
      1
    );

    Serial.print(
      F(" timeout=")
    );
    Serial.println(
      timeout ? 1 : 0
    );

    beginCornerTurn();
    return;
  }

  setMotor(
    -BACK_BEFORE_TURN_PWM
  );

  // Reverse while maintaining the old corridor heading.
  setSteering(
    preTurnBackYawSteering()
  );
}


void finishCornerTurn() {
  // Do NOT stop after a completed corner.
  // Immediately continue with corridor yaw + wall centering.
  setMotor(DRIVE_PWM);
  setSteering(
    corridorSteering()
  );

  turnCounter++;

  previousYawError = 0.0;
  turnToleranceStartMs = 0;
  turnTargetReached = false;

  // IMPORTANT:
  // Keep cornerTriggerLocked=true after the turn.
  // Rearm only after the robot has actually left this corner.
  lastCompletedTurnDirection = turnDirection;
  cornerFinishedMs = millis();
  cornerRearmStartTicks = getEncoderTicks();
  cornerTriggerLocked = true;

  lastAcceptedLineMs = millis();
  buzzerTurnComplete();

  if (turnCounter == 1 && latchedDirection == 0)
    latchedDirection = firstTurnDirection;

  Serial.print(F("# TURN COMPLETE yaw="));
  Serial.print(currentYaw, 2);
  Serial.print(F(" target="));
  Serial.print(targetYaw, 2);
  Serial.print(F(" turns="));
  Serial.println(turnCounter);

  // The 12th corner is intercepted before normal turning and handled
  // by startFinalParkingSequence(). Normal turns return to corridor.
  state =
    STATE_CORRIDOR;
}

void runCornerTurn() {
  // Safety recovery:
  // A slow/problematic turn must NOT stop the whole robot.
  if (millis() - cornerTurnStartMs >= TURN_TIMEOUT_MS) {
    centerSteering();
    setMotor(DRIVE_PWM);

    Serial.print(F("# TURN TIMEOUT RECOVERY current="));
    Serial.print(currentYaw, 2);
    Serial.print(F(" oldTarget="));
    Serial.println(targetYaw, 2);

    tone(PIN_BUZZER, 700, 220);

    // Accept the actual heading as the corridor reference, count
    // the corner, and continue instead of entering STATE_STOPPED.
    targetYaw = currentYaw;
    finishCornerTurn();
    return;
  }

  // Once the target has been crossed, immediately straighten the
  // steering and continue driving forward. This avoids the robot
  // stopping after the first (or any) turn.
  if (turnTargetReached) {
    setMotor(DRIVE_PWM);

    // Immediately resume yaw + side-ultrasonic centering.
    setSteering(
      corridorSteering()
    );

    if (millis() - turnToleranceStartMs >= TURN_SETTLE_MS)
      finishCornerTurn();

    return;
  }

  float error = targetYaw - currentYaw;
  float absError = fabs(error);

  // IMPORTANT:
  // LEFT turn  = positive yaw
  // RIGHT turn = negative yaw
  //
  // Stop directionally, so overshooting the exact tolerance
  // cannot make the robot rotate forever.
  bool reachedTarget;

  if (turnDirection > 0) {
    reachedTarget =
      currentYaw >= (targetYaw - TURN_TOLERANCE_DEG);
  } else {
    reachedTarget =
      currentYaw <= (targetYaw + TURN_TOLERANCE_DEG);
  }

  if (reachedTarget) {
    turnTargetReached = true;

    // Immediately leave the turning arc and return to corridor
    // yaw + side-ultrasonic centering.
    setMotor(DRIVE_PWM);
    setSteering(
      corridorSteering()
    );

    turnToleranceStartMs = millis();

    Serial.print(F("# TURN TARGET REACHED current="));
    Serial.print(currentYaw, 2);
    Serial.print(F(" target="));
    Serial.println(targetYaw, 2);

    return;
  }

  int pwm =
    (absError <= TURN_SLOW_ZONE_DEG)
    ? TURN_PWM_SLOW
    : TURN_PWM_FAST;

  setMotor(pwm);

  // After your servo reversal:
  // negative steering command = physical LEFT
  // positive steering command = physical RIGHT
  setSteering(-turnDirection * TURN_STEER_DEG);

  buzzerTurningTick();
}



// ===================== CORNER TRIGGER REARM =====================
//
// Called only while driving the normal corridor after a completed turn.
void updateCornerTriggerRearm() {
  if (!cornerTriggerLocked)
    return;

  // During approach/turn the lock must stay active.
  if (state != STATE_CORRIDOR)
    return;

  if (
    millis() - cornerFinishedMs <
    CORNER_REARM_MIN_MS
  ) {
    return;
  }

  float traveled =
    encoderDistanceCMFrom(
      cornerRearmStartTicks
    );

  bool minimumDistanceReady =
    traveled >=
    CORNER_REARM_FORWARD_CM;

  if (!minimumDistanceReady)
    return;

  // Check whether the opening used for the previous turn has closed.
  bool oldTurnSideClosed = false;

  if (lastCompletedTurnDirection > 0) {
    oldTurnSideClosed =
      validTurnOpeningDistance(leftCM) &&
      leftCM <=
        CORNER_REARM_SIDE_CLOSED_CM;
  }
  else if (lastCompletedTurnDirection < 0) {
    oldTurnSideClosed =
      validTurnOpeningDistance(rightCM) &&
      rightCM <=
        CORNER_REARM_SIDE_CLOSED_CM;
  }

  // Fallback release prevents the lock from lasting forever if the
  // side ultrasonic never reports a normal wall distance.
  bool forceDistanceReady =
    traveled >=
    CORNER_REARM_FORCE_CM;

  if (
    oldTurnSideClosed ||
    forceDistanceReady
  ) {
    cornerTriggerLocked = false;

    // Do not let a stale camera frame instantly fire when unlocking.
    lastAcceptedLineMs =
      millis();

    Serial.print(
      F("# CORNER REARM traveled=")
    );
    Serial.print(traveled, 1);

    Serial.print(F(" sideClosed="));
    Serial.print(
      oldTurnSideClosed ? 1 : 0
    );

    Serial.println(
      F(" -> triggers enabled")
    );
  }
}

// ===================== ULTRASONIC CORNER FALLBACK =====================
//
// This backup runs only from STATE_CORRIDOR when no valid camera
// intersection event was accepted.
//
// Return:
//   +1 = LEFT
//   -1 = RIGHT
//    0 = not ready
int getUltrasonicFallbackTurnDirection() {
  // First turn must come from BLUE/ORANGE camera detection.
  // Do not let side ultrasonics choose the initial race direction.
  if (
    firstTurnDirection == 0 &&
    latchedDirection == 0
  ) {
    return 0;
  }

  if (!ENABLE_ULTRASONIC_CORNER_FALLBACK)
    return 0;

  // Never allow another physical corner trigger until the previous
  // completed corner has been cleared/re-armed.
  if (cornerTriggerLocked)
    return 0;

  if (
    millis() - lastAcceptedLineMs <
    ULTRASONIC_FALLBACK_COOLDOWN_MS
  ) {
    return 0;
  }

  int cornerIndex =
    turnCounter % 4;

  float frontTurnThreshold =
    CORNER_FRONT_TURN_CM[
      cornerIndex
    ];

  bool frontValid =
    frontCM > 0.0 &&
    frontCM < 500.0;

  bool frontReady =
    USE_FRONT_CORNER_TRIGGER &&
    frontValid &&
    frontTurnThreshold > 0.0 &&
    frontCM <= frontTurnThreshold;

  if (!frontReady)
    return 0;

  bool leftOpen =
    validTurnOpeningDistance(leftCM) &&
    leftCM >=
      ULTRASONIC_FALLBACK_SIDE_MIN_CM;

  bool rightOpen =
    validTurnOpeningDistance(rightCM) &&
    rightCM >=
      ULTRASONIC_FALLBACK_SIDE_MIN_CM;

  // Direction already locked by previous completed first turn.
  if (latchedDirection > 0)
    return leftOpen ? +1 : 0;

  if (latchedDirection < 0)
    return rightOpen ? -1 : 0;

  // Preserve a first direction if one already exists.
  if (firstTurnDirection > 0)
    return leftOpen ? +1 : 0;

  if (firstTurnDirection < 0)
    return rightOpen ? -1 : 0;

  // First corner with no camera direction.
  if (leftOpen && !rightOpen)
    return +1;

  if (rightOpen && !leftOpen)
    return -1;

  // If both sides look open, use the more-open side.
  if (leftOpen && rightOpen) {
    return
      (leftCM >= rightCM)
      ? +1
      : -1;
  }

  return 0;
}

void startUltrasonicFallbackCorner(
  int fallbackDirection
) {
  if (
    fallbackDirection != +1 &&
    fallbackDirection != -1
  ) {
    return;
  }

  turnDirection =
    fallbackDirection;

  // One-shot lock for this physical corner.
  cornerTriggerLocked = true;

  // If camera missed the first intersection entirely, save the
  // inferred direction so the existing first-turn latch works.
  if (
    turnCounter == 0 &&
    latchedDirection == 0 &&
    firstTurnDirection == 0
  ) {
    firstTurnDirection =
      fallbackDirection;
  }

  activeCornerIndex =
    turnCounter % 4;

  cornerApproachStartTicks =
    getEncoderTicks();

  lastAcceptedLineMs =
    millis();

  Serial.print(
    F("# ULTRASONIC FALLBACK CORNER dir=")
  );

  Serial.print(
    fallbackDirection > 0
      ? F("LEFT")
      : F("RIGHT")
  );

  Serial.print(F(" front="));
  Serial.print(frontCM, 1);

  Serial.print(F(" frontLimit="));
  Serial.print(
    CORNER_FRONT_TURN_CM[
      activeCornerIndex
    ],
    1
  );

  Serial.print(F(" left="));
  Serial.print(leftCM, 1);

  Serial.print(F(" right="));
  Serial.print(rightCM, 1);

  Serial.print(F(" sideMin="));
  Serial.println(
    ULTRASONIC_FALLBACK_SIDE_MIN_CM,
    1
  );

  // Corner mode has priority over pillar avoidance.
  clearPillarCandidateForCorner();

  // Use the same optional pre-turn reverse used by camera corners.
  // The final special corner is automatically excluded inside it.
  beginPreTurnBackOrTurn();
}

// ===================== PILLAR AVOIDANCE =====================
//
// This logic is shared with Open Challenge:
// if no RED/GREEN pillar is detected, these states never activate.
//
// Direction convention:
//   +1 = LEFT
//   -1 = RIGHT
//
// RED   -> RIGHT (-1)
// GREEN -> LEFT  (+1)

int pillarAvoidDirection = 0;
int activePillarID = 0;

float pillarSavedCorridorYaw = 0.0;
float pillarOutYaw = 0.0;

// Calculated separately for every pillar from its HUSKYLENS X position.
float pillarDynamicTurnDeg = 0.0;
float pillarAvoidSeverity = 0.0;
int pillarDetectionXAtStart = 0;

// Prevent the same physical pillar from triggering repeatedly.
// It is armed again only after the pillar disappears.
bool pillarIgnoreUntilClear = false;

long pillarForwardStartTicks = 0;

unsigned long pillarStateStartMs = 0;
unsigned long lastPillarAvoidCompleteMs = 0;
unsigned long lastPillarSafeCheckMs = 0;

float pillarPreviousYawError = 0.0;
unsigned long pillarLastControlMicros = 0;

// Smooth steering command used only during pillar avoidance.
float pillarSmoothSteeringDeg = 0.0;
unsigned long pillarSmoothSteeringMicros = 0;

// ===================== SMART SIDE / REAR TRACKING =====================
//
// Pass phases inside STATE_PILLAR_FORWARD:
//   0 = waiting for front-side sensor to see/pass pillar
//   1 = waiting for rear-side sensor to see/pass pillar
//   2 = small encoder rear-wheel safety clearance
int pillarPassPhase = 0;

float pillarFrontSideStartCM = 999.0;
float pillarFrontSideMinCM = 999.0;
float pillarFrontSidePrevCM = 999.0;

float pillarRearSideStartCM = 999.0;
float pillarRearSideMinCM = 999.0;
float pillarRearSidePrevCM = 999.0;

bool pillarFrontSideEngaged = false;
bool pillarRearSideEngaged = false;

int pillarFrontRiseCount = 0;
int pillarRearRiseCount = 0;

long pillarPassStartTicks = 0;
long pillarRearSafetyStartTicks = 0;

unsigned long pillarPassStartMs = 0;

// Latest automatically selected front/rear side readings.
float pillarTrackedFrontSideCM = 999.0;
float pillarTrackedRearSideCM = 999.0;

// Explicit prototypes for smart-pillar helpers used before their
// full definitions. This avoids Arduino auto-prototype ordering issues.
bool rawPillarVisible();
bool pillarIsDangerouslyClose();
float calculatePillarSeverity(int pillarID, float x);

bool validPillarSideCM(
  float cm
) {
  return
    cm > 2.0 &&
    cm < 350.0;
}

// RED -> pillar is on LEFT side while robot passes on RIGHT.
// GREEN -> pillar is on RIGHT side while robot passes on LEFT.
float getPillarFrontSideCM() {
  if (
    isRedPillarID(
      activePillarID
    )
  ) {
    return leftCM;
  }

  if (
    isGreenPillarID(
      activePillarID
    )
  ) {
    return rightCM;
  }

  return 999.0;
}

float getPillarRearSideCM() {
  if (
    isRedPillarID(
      activePillarID
    )
  ) {
    return rearLeftCM;
  }

  if (
    isGreenPillarID(
      activePillarID
    )
  ) {
    return rearRightCM;
  }

  return 999.0;
}

int pillarAdaptivePWM() {
  float s =
    constrain(
      pillarAvoidSeverity,
      0.0,
      1.0
    );

  // Higher danger -> slower speed for better control.
  float pwm =
    PILLAR_SPEED_MAX_PWM -
    s *
    (
      PILLAR_SPEED_MAX_PWM -
      PILLAR_SPEED_MIN_PWM
    );

  return constrain(
    (int)pwm,
    PILLAR_SPEED_MIN_PWM,
    PILLAR_SPEED_MAX_PWM
  );
}

void updateAdaptivePillarYaw() {
  // Keep updating the desired avoidance yaw while the SAME pillar
  // is still confidently visible.
  if (
    !rawPillarVisible() ||
    huskyBestPillarID !=
      activePillarID
  ) {
    return;
  }

  float newSeverity =
    calculatePillarSeverity(
      activePillarID,
      huskyBestPillarX
    );

  // Visual closeness can boost only an already-dangerous path.
  if (
    pillarIsDangerouslyClose() &&
    newSeverity >=
      PILLAR_CLOSE_BOOST_START_SEVERITY
  ) {
    newSeverity =
      newSeverity +
      PILLAR_CLOSE_SEVERITY_BOOST *
      (
        1.0 -
        newSeverity
      );
  }

  newSeverity =
    constrain(
      newSeverity,
      0.0,
      1.0
    );

  // Low-pass severity so one camera frame cannot jerk the trajectory.
  pillarAvoidSeverity =
    0.72 *
      pillarAvoidSeverity +
    0.28 *
      newSeverity;

  float wantedDeg =
    PILLAR_MIN_TURN_OUT_DEG +
    pillarAvoidSeverity *
    (
      PILLAR_MAX_TURN_OUT_DEG -
      PILLAR_MIN_TURN_OUT_DEG
    );

  wantedDeg =
    constrain(
      wantedDeg,
      PILLAR_MIN_TURN_OUT_DEG,
      PILLAR_MAX_TURN_OUT_DEG
    );

  // Smooth target yaw itself, not only the servo.
  pillarDynamicTurnDeg =
    0.75 *
      pillarDynamicTurnDeg +
    0.25 *
      wantedDeg;

  pillarOutYaw =
    pillarSavedCorridorYaw +
    pillarAvoidDirection *
    pillarDynamicTurnDeg;

  targetYaw =
    pillarOutYaw;
}

void resetPillarSideTracking() {
  pillarTrackedFrontSideCM =
    getPillarFrontSideCM();

  pillarTrackedRearSideCM =
    getPillarRearSideCM();

  pillarFrontSideStartCM =
    validPillarSideCM(
      pillarTrackedFrontSideCM
    )
      ? pillarTrackedFrontSideCM
      : 999.0;

  pillarRearSideStartCM =
    validPillarSideCM(
      pillarTrackedRearSideCM
    )
      ? pillarTrackedRearSideCM
      : 999.0;

  pillarFrontSideMinCM =
    pillarFrontSideStartCM;

  pillarFrontSidePrevCM =
    pillarFrontSideStartCM;

  pillarRearSideMinCM =
    pillarRearSideStartCM;

  pillarRearSidePrevCM =
    pillarRearSideStartCM;

  pillarFrontSideEngaged = false;
  pillarRearSideEngaged = false;

  pillarFrontRiseCount = 0;
  pillarRearRiseCount = 0;

  pillarPassPhase = 0;
  pillarPassStartTicks =
    getEncoderTicks();

  pillarPassStartMs =
    millis();
}

bool updateFrontSidePass() {
  float cm =
    getPillarFrontSideCM();

  pillarTrackedFrontSideCM =
    cm;

  if (!validPillarSideCM(cm))
    return false;

  if (
    pillarFrontSideStartCM >=
    500.0
  ) {
    pillarFrontSideStartCM =
      cm;

    pillarFrontSideMinCM =
      cm;

    pillarFrontSidePrevCM =
      cm;
  }

  if (
    cm <
    pillarFrontSideMinCM
  ) {
    pillarFrontSideMinCM =
      cm;

    pillarFrontRiseCount = 0;
  }

  float enterThreshold =
    pillarFrontSideStartCM *
    PILLAR_SIDE_ENTER_RATIO;

  bool meaningfulDrop =
    pillarFrontSideStartCM -
    pillarFrontSideMinCM >=
    PILLAR_SIDE_MIN_DROP_CM;

  if (
    !pillarFrontSideEngaged &&
    meaningfulDrop &&
    pillarFrontSideMinCM <=
      enterThreshold
  ) {
    pillarFrontSideEngaged =
      true;
  }

  if (pillarFrontSideEngaged) {
    float exitThreshold =
      pillarFrontSideMinCM *
      PILLAR_SIDE_EXIT_RATIO;

    bool rising =
      cm >
      pillarFrontSidePrevCM;

    if (
      rising &&
      cm >= exitThreshold
    ) {
      pillarFrontRiseCount++;
    }
    else if (
      cm <
      pillarFrontSidePrevCM
    ) {
      pillarFrontRiseCount = 0;
    }

    if (
      pillarFrontRiseCount >=
      PILLAR_SIDE_CONFIRM_READS
    ) {
      pillarFrontSidePrevCM =
        cm;

      return true;
    }
  }

  pillarFrontSidePrevCM =
    cm;

  return false;
}

bool updateRearSidePass() {
  float cm =
    getPillarRearSideCM();

  pillarTrackedRearSideCM =
    cm;

  if (!validPillarSideCM(cm))
    return false;

  if (
    pillarRearSideStartCM >=
    500.0
  ) {
    pillarRearSideStartCM =
      cm;

    pillarRearSideMinCM =
      cm;

    pillarRearSidePrevCM =
      cm;
  }

  if (
    cm <
    pillarRearSideMinCM
  ) {
    pillarRearSideMinCM =
      cm;

    pillarRearRiseCount = 0;
  }

  float enterThreshold =
    pillarRearSideStartCM *
    PILLAR_SIDE_ENTER_RATIO;

  bool meaningfulDrop =
    pillarRearSideStartCM -
    pillarRearSideMinCM >=
    PILLAR_SIDE_MIN_DROP_CM;

  if (
    !pillarRearSideEngaged &&
    meaningfulDrop &&
    pillarRearSideMinCM <=
      enterThreshold
  ) {
    pillarRearSideEngaged =
      true;
  }

  if (pillarRearSideEngaged) {
    float exitThreshold =
      pillarRearSideMinCM *
      PILLAR_SIDE_EXIT_RATIO;

    bool rising =
      cm >
      pillarRearSidePrevCM;

    if (
      rising &&
      cm >= exitThreshold
    ) {
      pillarRearRiseCount++;
    }
    else if (
      cm <
      pillarRearSidePrevCM
    ) {
      pillarRearRiseCount = 0;
    }

    if (
      pillarRearRiseCount >=
      PILLAR_SIDE_CONFIRM_READS
    ) {
      pillarRearSidePrevCM =
        cm;

      return true;
    }
  }

  pillarRearSidePrevCM =
    cm;

  return false;
}

bool rawPillarVisible() {
  if (
    huskyBestPillarID == 0 ||
    millis() - lastGoodPillarFrameMs >
      HUSKY_PILLAR_FRAME_STALE_MS
  ) {
    return false;
  }

  return (
    isRedPillarID(huskyBestPillarID) ||
    isGreenPillarID(huskyBestPillarID)
  );
}

bool pillarIsDangerouslyClose() {
  if (!rawPillarVisible())
    return false;

  int bottomY =
    huskyBestPillarY +
    huskyBestPillarH / 2;

  bool largeArea =
    huskyBestPillarArea >=
    PILLAR_FORCE_AVOID_AREA;

  bool largeHeight =
    huskyBestPillarH >=
    PILLAR_FORCE_AVOID_HEIGHT;

  bool lowInImage =
    bottomY >=
    PILLAR_FORCE_AVOID_BOTTOM_Y;

  // Any strong closeness cue forces avoidance.
  return
    largeArea ||
    largeHeight ||
    lowInImage;
}

// Return true when the robot is already at a corner/opening.
//
// Once a round direction is known:
//   LEFT round  -> check LEFT ultrasonic
//   RIGHT round -> check RIGHT ultrasonic
//
// The SAME 100cm value used by the ultrasonic corner fallback is used,
// so there is only one side-opening tune to change.
bool suppressPillarBecauseTurnSideIsOpen() {
  int knownDirection = 0;

  if (latchedDirection != 0)
    knownDirection = latchedDirection;
  else if (firstTurnDirection != 0)
    knownDirection = firstTurnDirection;

  // Before the first turn direction is known, do not guess here.
  // The ultrasonic fallback function handles that situation.
  if (knownDirection == 0)
    return false;

  if (knownDirection > 0) {
    return
      validTurnOpeningDistance(leftCM) &&
      leftCM >=
        ULTRASONIC_FALLBACK_SIDE_MIN_CM;
  }

  return
    validTurnOpeningDistance(rightCM) &&
    rightCM >=
      ULTRASONIC_FALLBACK_SIDE_MIN_CM;
}

bool currentPillarIsVisible() {
  if (!ENABLE_PILLAR_AVOIDANCE)
    return false;

  bool visible =
    rawPillarVisible();

  // After passing or intentionally skipping one pillar,
  // do not trigger it again while it is still visible.
  if (pillarIgnoreUntilClear) {
    if (!visible) {
      pillarIgnoreUntilClear = false;
    }

    return false;
  }

  if (!visible)
    return false;

  // Require multi-frame camera confidence.
  if (
    pillarConfidenceScore <
    PILLAR_CONFIDENCE_TRIGGER
  ) {
    return false;
  }

  // If the previous evaluation said the pillar was safe by X,
  // check it again shortly. Do not lock it out until it disappears.
  if (
    millis() -
    lastPillarSafeCheckMs <
    PILLAR_SAFE_RECHECK_MS
  ) {
    return false;
  }

  if (
    millis() -
    lastPillarAvoidCompleteMs <
    PILLAR_COOLDOWN_MS
  ) {
    return false;
  }

  return true;
}

// Returns 0 when no avoidance is needed.
// Otherwise returns an adaptive turn angle between
// PILLAR_MIN_TURN_OUT_DEG and PILLAR_MAX_TURN_OUT_DEG.
float calculatePillarSeverity(
  int pillarID,
  float x
) {
  float rawSeverity = 0.0;

  // RED -> pass on RIGHT.
  // RED farther LEFT is safer.
  if (
    isRedPillarID(
      pillarID
    )
  ) {
    if (
      x <=
      PILLAR_RED_NO_AVOID_X
    ) {
      return 0.0;
    }

    float span =
      PILLAR_RED_FULL_AVOID_X -
      PILLAR_RED_NO_AVOID_X;

    if (span <= 1.0)
      span = 1.0;

    rawSeverity =
      (
        x -
        PILLAR_RED_NO_AVOID_X
      ) /
      span;
  }

  // GREEN -> pass on LEFT.
  // GREEN farther RIGHT is safer.
  // GREEN farther LEFT is more dangerous.
  else if (
    isGreenPillarID(
      pillarID
    )
  ) {
    if (
      x >=
      PILLAR_GREEN_NO_AVOID_X
    ) {
      return 0.0;
    }

    float span =
      PILLAR_GREEN_NO_AVOID_X -
      PILLAR_GREEN_FULL_AVOID_X;

    if (span <= 1.0)
      span = 1.0;

    rawSeverity =
      (
        PILLAR_GREEN_NO_AVOID_X -
        x
      ) /
      span;
  }
  else {
    return 0.0;
  }

  rawSeverity =
    constrain(
      rawSeverity,
      0.0,
      1.0
    );

  // Smoothstep danger curve:
  //
  // raw 0.20 -> ~0.10 = softer near safe side
  // raw 0.50 ->  0.50
  // raw 0.80 -> ~0.90 = harder in dangerous position
  float shapedSeverity =
    rawSeverity *
    rawSeverity *
    (
      3.0 -
      2.0 * rawSeverity
    );

  return constrain(
    shapedSeverity,
    0.0,
    1.0
  );
}

float calculatePillarTurnDeg(
  int pillarID,
  float x
) {
  pillarAvoidSeverity =
    calculatePillarSeverity(
      pillarID,
      x
    );

  if (
    pillarAvoidSeverity <= 0.0
  ) {
    return 0.0;
  }

  float turnDeg =
    PILLAR_MIN_TURN_OUT_DEG +
    pillarAvoidSeverity *
    (
      PILLAR_MAX_TURN_OUT_DEG -
      PILLAR_MIN_TURN_OUT_DEG
    );

  if (
    turnDeg <
    PILLAR_SKIP_BELOW_DEG
  ) {
    return 0.0;
  }

  return turnDeg;
}

float pillarYawOnlySteering() {
  float error =
    targetYaw -
    currentYaw;

  unsigned long now =
    micros();

  float dt =
    (float)(now - pillarLastControlMicros) /
    1000000.0f;

  if (dt <= 0.0f || dt > 0.20f)
    dt = 0.02f;

  pillarLastControlMicros =
    now;

  float derivative =
    (error - pillarPreviousYawError) /
    dt;

  pillarPreviousYawError =
    error;

  float command =
    -(PILLAR_YAW_KP * error +
      PILLAR_YAW_KD * derivative);

  return constrain(
    command,
    -PILLAR_FORWARD_MAX_STEER_DEG,
    +PILLAR_FORWARD_MAX_STEER_DEG
  );
}

bool pillarYawReached(
  float wantedYaw,
  int direction
) {
  // LEFT (+1): yaw increases.
  if (direction > 0) {
    return currentYaw >=
      (wantedYaw - PILLAR_YAW_TOLERANCE_DEG);
  }

  // RIGHT (-1): yaw decreases.
  return currentYaw <=
    (wantedYaw + PILLAR_YAW_TOLERANCE_DEG);
}

float pillarSlewSteering(
  float desiredDeg
) {
  desiredDeg =
    constrain(
      desiredDeg,
      -MAX_STEER_DEG,
      +MAX_STEER_DEG
    );

  unsigned long now =
    micros();

  float dt =
    (float)(now - pillarSmoothSteeringMicros) /
    1000000.0f;

  if (
    pillarSmoothSteeringMicros == 0 ||
    dt <= 0.0f ||
    dt > 0.20f
  ) {
    dt = 0.02f;
  }

  pillarSmoothSteeringMicros =
    now;

  float maxStep =
    PILLAR_STEER_SLEW_DEG_PER_SEC *
    dt;

  float delta =
    desiredDeg -
    pillarSmoothSteeringDeg;

  delta =
    constrain(
      delta,
      -maxStep,
      +maxStep
    );

  pillarSmoothSteeringDeg +=
    delta;

  return pillarSmoothSteeringDeg;
}

float pillarSmoothTurnSteering(
  float wantedYaw,
  int direction
) {
  float remaining =
    fabs(
      wantedYaw -
      currentYaw
    );

  float ratio =
    constrain(
      remaining /
      PILLAR_TURN_SLOW_ZONE_DEG,
      0.0,
      1.0
    );

  float magnitude =
    PILLAR_TURN_MIN_STEER_DEG +
    ratio *
    (
      PILLAR_TURN_STEER_DEG -
      PILLAR_TURN_MIN_STEER_DEG
    );

  // LEFT (+1) -> negative software steering.
  // RIGHT (-1) -> positive software steering.
  float desired =
    -direction *
    magnitude;

  return pillarSlewSteering(
    desired
  );
}

void startPillarAvoidance() {
  activePillarID =
    huskyBestPillarID;

  pillarDetectionXAtStart =
    huskyBestPillarX;

  // FINAL BEHAVIOR: RED = pass on RIGHT.
  if (
    isRedPillarID(
      activePillarID
    )
  ) {
    pillarAvoidDirection = -1;
  }

  // FINAL BEHAVIOR: GREEN = pass on LEFT.
  else if (
    isGreenPillarID(
      activePillarID
    )
  ) {
    pillarAvoidDirection = +1;
  }
  else {
    return;
  }

  // Capture the local side-wall/background distances automatically.
  // These become the per-pillar ultrasonic reference.
  resetPillarSideTracking();

  // --------------------------------------------------------
  // CALCULATE REQUIRED TURN FROM PILLAR X LOCATION
  // --------------------------------------------------------
  pillarDynamicTurnDeg =
    calculatePillarTurnDeg(
      activePillarID,
      pillarDetectionXAtStart
    );

  // Visual closeness strengthens an already-dangerous trajectory.
  bool forceCloseAvoid =
    pillarIsDangerouslyClose();

  if (
    forceCloseAvoid &&
    pillarAvoidSeverity >=
      PILLAR_CLOSE_BOOST_START_SEVERITY
  ) {
    pillarAvoidSeverity =
      pillarAvoidSeverity +
      PILLAR_CLOSE_SEVERITY_BOOST *
      (
        1.0 -
        pillarAvoidSeverity
      );

    pillarAvoidSeverity =
      constrain(
        pillarAvoidSeverity,
        0.0,
        1.0
      );

    pillarDynamicTurnDeg =
      PILLAR_MIN_TURN_OUT_DEG +
      pillarAvoidSeverity *
      (
        PILLAR_MAX_TURN_OUT_DEG -
        PILLAR_MIN_TURN_OUT_DEG
      );
  }

  // --------------------------------------------------------
  // NO AVOIDANCE REQUIRED
  // This is allowed ONLY when the pillar is not dangerously close.
  // --------------------------------------------------------
  if (
    pillarDynamicTurnDeg <= 0.0
  ) {
    Serial.print(F("# PILLAR SAFE - NO AVOID id="));
    Serial.print(activePillarID);

    Serial.print(F(" X="));
    Serial.print(
      pillarDetectionXAtStart
    );

    Serial.print(F(" area="));
    Serial.print(huskyBestPillarArea);

    Serial.print(F(" h="));
    Serial.print(huskyBestPillarH);

    Serial.print(F(" bottom="));
    Serial.print(
      huskyBestPillarY +
      huskyBestPillarH / 2
    );

    Serial.println(
      F(" continue corridor / recheck")
    );

    // IMPORTANT:
    // Do NOT set pillarIgnoreUntilClear here.
    //
    // The same pillar may be safe now but become large/close a moment
    // later. Re-evaluate it after PILLAR_SAFE_RECHECK_MS.
    lastPillarSafeCheckMs =
      millis();

    activePillarID = 0;
    pillarAvoidDirection = 0;

    return;
  }

  // Save the ORIGINAL main corridor yaw.
  pillarSavedCorridorYaw =
    targetYaw;

  // Dynamic outward yaw.
  pillarOutYaw =
    pillarSavedCorridorYaw +
    pillarAvoidDirection *
    pillarDynamicTurnDeg;

  pillarStateStartMs =
    millis();

  pillarPreviousYawError = 0.0;
  pillarLastControlMicros =
    micros();

  pillarSmoothSteeringDeg = 0.0;
  pillarSmoothSteeringMicros = micros();

  // --------------------------------------------------------
  // STOP BEFORE AVOIDANCE
  // --------------------------------------------------------
  if (PILLAR_STOP_BEFORE_AVOID) {
    stopMotor();
    centerSteering();

    // Keep targetYaw at the saved corridor heading while stopped.
    targetYaw =
      pillarSavedCorridorYaw;

    Serial.print(F("# PILLAR AVOID START STOP id="));
  }
  else {
    // Continuous alternative.
    targetYaw =
      pillarOutYaw;

    setMotor(
      pillarAdaptivePWM()
    );

    setSteering(
      pillarSmoothTurnSteering(
        pillarOutYaw,
        pillarAvoidDirection
      )
    );

    Serial.print(F("# PILLAR AVOID START CONTINUOUS id="));
  }
  Serial.print(activePillarID);

  Serial.print(F(" X="));
  Serial.print(
    pillarDetectionXAtStart
  );

  Serial.print(F(" dir="));
  Serial.print(
    pillarAvoidDirection < 0
      ? F("RIGHT")
      : F("LEFT")
  );

  Serial.print(F(" adaptiveDeg="));
  Serial.print(
    pillarDynamicTurnDeg,
    1
  );

  Serial.print(F(" forceClose="));
  Serial.print(
    forceCloseAvoid ? 1 : 0
  );

  Serial.print(F(" shapedSeverity="));
  Serial.print(
    pillarAvoidSeverity,
    2
  );

  Serial.print(F(" frontSideStart="));
  Serial.print(
    pillarFrontSideStartCM,
    1
  );

  Serial.print(F(" rearSideStart="));
  Serial.print(
    pillarRearSideStartCM,
    1
  );

  Serial.print(F(" area="));
  Serial.print(
    huskyBestPillarArea
  );

  Serial.print(F(" Y="));
  Serial.print(
    huskyBestPillarY
  );

  Serial.print(F(" savedYaw="));
  Serial.print(
    pillarSavedCorridorYaw,
    2
  );

  Serial.print(F(" outYaw="));
  Serial.println(
    pillarOutYaw,
    2
  );

  if (PILLAR_STOP_BEFORE_AVOID) {
    state =
      STATE_PILLAR_STOP;
  }
  else {
    state =
      STATE_PILLAR_TURN_OUT;
  }
}

void runPillarStop() {
  stopMotor();
  centerSteering();

  if (
    millis() -
    pillarStateStartMs >=
    PILLAR_STOP_MS
  ) {
    targetYaw =
      pillarOutYaw;

    pillarStateStartMs =
      millis();

    pillarSmoothSteeringDeg =
      0.0;

    pillarSmoothSteeringMicros =
      micros();

    setMotor(
      pillarAdaptivePWM()
    );

    state =
      STATE_PILLAR_TURN_OUT;
  }
}

void runPillarTurnOut() {
  updateYaw();

  // Camera keeps refining the S-curve while approaching the pillar.
  updateAdaptivePillarYaw();

  if (
    pillarYawReached(
      pillarOutYaw,
      pillarAvoidDirection
    )
  ) {
    targetYaw =
      pillarOutYaw;

    pillarPassStartTicks =
      getEncoderTicks();

    pillarPassStartMs =
      millis();

    pillarPreviousYawError =
      0.0;

    pillarLastControlMicros =
      micros();

    // Do NOT return yet.
    // Hold the offset until FRONT and then REAR side sensors confirm
    // the whole robot has passed the pillar.
    setMotor(
      pillarAdaptivePWM()
    );

    Serial.print(
      F("# PILLAR TURN OUT COMPLETE yaw=")
    );
    Serial.print(
      currentYaw,
      2
    );

    Serial.print(
      F(" frontSide=")
    );
    Serial.print(
      getPillarFrontSideCM(),
      1
    );

    Serial.print(
      F(" rearSide=")
    );
    Serial.println(
      getPillarRearSideCM(),
      1
    );

    state =
      STATE_PILLAR_FORWARD;

    return;
  }

  if (
    millis() -
    pillarStateStartMs >=
    PILLAR_TURN_TIMEOUT_MS
  ) {
    // Recovery: keep the current achieved offset and continue pass tracking.
    pillarOutYaw =
      currentYaw;

    targetYaw =
      currentYaw;

    pillarPassStartTicks =
      getEncoderTicks();

    pillarPassStartMs =
      millis();

    Serial.println(
      F("# PILLAR TURN TIMEOUT -> SMART PASS")
    );

    state =
      STATE_PILLAR_FORWARD;

    return;
  }

  setMotor(
    pillarAdaptivePWM()
  );

  setSteering(
    pillarSmoothTurnSteering(
      pillarOutYaw,
      pillarAvoidDirection
    )
  );
}

void runPillarForward() {
  updateYaw();

  // Continue gently adapting the outward yaw while camera still sees
  // the same pillar. Once it leaves the camera, hold the last safe yaw.
  if (
    pillarPassPhase <= 1
  ) {
    updateAdaptivePillarYaw();
  }

  setMotor(
    pillarAdaptivePWM()
  );

  setSteering(
    pillarSlewSteering(
      pillarYawOnlySteering()
    )
  );

  float traveled =
    encoderDistanceCMFrom(
      pillarPassStartTicks
    );

  // --------------------------------------------------------
  // PHASE 0: FRONT-SIDE SENSOR PASSES THE PILLAR
  // --------------------------------------------------------
  if (pillarPassPhase == 0) {
    bool frontPassed =
      updateFrontSidePass();

    // Track rear sensor from the beginning too, so its minimum history
    // is available by the time the front passes.
    updateRearSidePass();

    if (frontPassed) {
      pillarPassPhase = 1;

      pillarRearRiseCount = 0;

      Serial.print(
        F("# PILLAR FRONT PASSED min=")
      );
      Serial.print(
        pillarFrontSideMinCM,
        1
      );

      Serial.print(
        F(" now=")
      );
      Serial.println(
        pillarTrackedFrontSideCM,
        1
      );
    }
  }

  // --------------------------------------------------------
  // PHASE 1: REAR-SIDE SENSOR PASSES THE PILLAR
  // --------------------------------------------------------
  else if (pillarPassPhase == 1) {
    bool rearPassed =
      updateRearSidePass();

    if (rearPassed) {
      pillarPassPhase = 2;

      pillarRearSafetyStartTicks =
        getEncoderTicks();

      // Hold the current safe offset/yaw while the wide rear wheel
      // gains a final small encoder safety margin.
      targetYaw =
        pillarOutYaw;

      Serial.print(
        F("# PILLAR REAR PASSED min=")
      );
      Serial.print(
        pillarRearSideMinCM,
        1
      );

      Serial.print(
        F(" now=")
      );
      Serial.println(
        pillarTrackedRearSideCM,
        1
      );
    }
  }

  // --------------------------------------------------------
  // PHASE 2: SMALL REAR-WHEEL SAFETY DISTANCE
  // --------------------------------------------------------
  else if (pillarPassPhase == 2) {
    float rearSafetyTravel =
      encoderDistanceCMFrom(
        pillarRearSafetyStartTicks
      );

    if (
      rearSafetyTravel >=
      PILLAR_REAR_SAFETY_CM
    ) {
      targetYaw =
        pillarSavedCorridorYaw;

      pillarStateStartMs =
        millis();

      pillarPreviousYawError =
        0.0;

      pillarLastControlMicros =
        micros();

      Serial.print(
        F("# PILLAR REAR CLEAR safetyCM=")
      );
      Serial.print(
        rearSafetyTravel,
        1
      );

      Serial.print(
        F(" returnYaw=")
      );
      Serial.println(
        pillarSavedCorridorYaw,
        2
      );

      state =
        STATE_PILLAR_RETURN_YAW;

      return;
    }
  }

  // --------------------------------------------------------
  // SENSOR FALLBACK
  // --------------------------------------------------------
  // If one of the new side sensors is disconnected or noisy, do not
  // freeze the robot beside the pillar forever.
  bool passTimeout =
    millis() -
      pillarPassStartMs >=
      PILLAR_PASS_TIMEOUT_MS;

  bool distanceFallback =
    traveled >=
      PILLAR_PASS_FALLBACK_CM;

  if (
    passTimeout ||
    distanceFallback
  ) {
    targetYaw =
      pillarSavedCorridorYaw;

    pillarStateStartMs =
      millis();

    pillarPreviousYawError =
      0.0;

    pillarLastControlMicros =
      micros();

    Serial.print(
      F("# PILLAR PASS FALLBACK time=")
    );
    Serial.print(
      passTimeout ? 1 : 0
    );

    Serial.print(
      F(" cm=")
    );
    Serial.println(
      traveled,
      1
    );

    state =
      STATE_PILLAR_RETURN_YAW;
  }
}

void runPillarReturnYaw() {
  updateYaw();

  int returnDirection =
    -pillarAvoidDirection;

  if (
    pillarYawReached(
      pillarSavedCorridorYaw,
      returnDirection
    )
  ) {
    targetYaw =
      pillarSavedCorridorYaw;

    setMotor(
      DRIVE_PWM
    );

    setSteering(
      corridorSteering()
    );

    lastPillarAvoidCompleteMs =
      millis();

    pillarIgnoreUntilClear =
      true;

    Serial.print(
      F("# SMART PILLAR COMPLETE yaw=")
    );
    Serial.print(
      currentYaw,
      2
    );

    Serial.print(
      F(" mainYaw=")
    );
    Serial.println(
      targetYaw,
      2
    );

    activePillarID = 0;
    pillarAvoidDirection = 0;
    pillarPassPhase = 0;

    state =
      STATE_CORRIDOR;

    return;
  }

  if (
    millis() -
    pillarStateStartMs >=
    PILLAR_TURN_TIMEOUT_MS
  ) {
    // Recovery: accept the actual heading and continue safely.
    targetYaw =
      currentYaw;

    lastPillarAvoidCompleteMs =
      millis();

    pillarIgnoreUntilClear =
      true;

    Serial.println(
      F("# PILLAR RETURN TIMEOUT -> CORRIDOR")
    );

    setMotor(
      DRIVE_PWM
    );

    setSteering(
      corridorSteering()
    );

    activePillarID = 0;
    pillarAvoidDirection = 0;
    pillarPassPhase = 0;

    state =
      STATE_CORRIDOR;

    return;
  }

  // Return is also slew-limited, creating the second half of the S-curve.
  setMotor(
    pillarAdaptivePWM()
  );

  setSteering(
    pillarSmoothTurnSteering(
      pillarSavedCorridorYaw,
      returnDirection
    )
  );
}

// ===================== STUCK / ESCAPE =====================
long lastStuckTicks = 0;
unsigned long lastStuckCheckMs = 0;
long escapeStartTicks = 0;
unsigned long escapeStartMs = 0;

void beginEscape() {
  if (
      state == STATE_WAIT_START ||
      state == STATE_STOPPED ||
      state == STATE_ESCAPE_BACK ||
      state == STATE_ESCAPE_FORWARD ||
      state == STATE_PRE_TURN_BACK ||


      state == STATE_PILLAR_STOP ||
      state == STATE_PILLAR_TURN_OUT ||
      state == STATE_PILLAR_FORWARD ||
      state == STATE_PILLAR_RETURN_YAW ||

      state == STATE_FINAL_TO_WALL ||
      state == STATE_FINAL_BACK ||
      state == STATE_FINAL_CORRIDOR_TURN ||
      state == STATE_FINAL_CENTER_FORWARD ||
      state == STATE_FINAL_SHIFT_TURN ||
      state == STATE_FINAL_SHIFT_FORWARD ||
      state == STATE_FINAL_RETURN_MAIN_YAW ||
      state == STATE_FINAL_BEFORE_PARK ||
      state == STATE_FINAL_PARK_ENTRY_TURN ||
      state == STATE_FINAL_PARK_FORWARD
  ) {
    return;
  }
  stateBeforeEscape = state;
  stopMotor();
  centerSteering();
  buzzerStuck();
  escapeStartTicks = getEncoderTicks();
  escapeStartMs = millis();
  state = STATE_ESCAPE_BACK;
}

void monitorStuck() {
  if (
      state == STATE_WAIT_START ||
      state == STATE_STOPPED ||
      state == STATE_ESCAPE_BACK ||
      state == STATE_ESCAPE_FORWARD ||
      state == STATE_PRE_TURN_BACK ||


      state == STATE_PILLAR_STOP ||
      state == STATE_PILLAR_TURN_OUT ||
      state == STATE_PILLAR_FORWARD ||
      state == STATE_PILLAR_RETURN_YAW ||

      state == STATE_FINAL_TO_WALL ||
      state == STATE_FINAL_BACK ||
      state == STATE_FINAL_CORRIDOR_TURN ||
      state == STATE_FINAL_CENTER_FORWARD ||
      state == STATE_FINAL_SHIFT_TURN ||
      state == STATE_FINAL_SHIFT_FORWARD ||
      state == STATE_FINAL_RETURN_MAIN_YAW ||
      state == STATE_FINAL_BEFORE_PARK ||
      state == STATE_FINAL_PARK_ENTRY_TURN ||
      state == STATE_FINAL_PARK_FORWARD
  ) {
    lastStuckTicks =
      getEncoderTicks();

    lastStuckCheckMs =
      millis();

    return;
  }

  if (abs(lastMotorCommand) < STUCK_MIN_MOTOR_PWM) {
    lastStuckTicks = getEncoderTicks();
    lastStuckCheckMs = millis();
    return;
  }

  if (millis() - lastStuckCheckMs < STUCK_CHECK_WINDOW_MS) return;

  long nowTicks = getEncoderTicks();
  long movement = labs(nowTicks - lastStuckTicks);
  lastStuckTicks = nowTicks;
  lastStuckCheckMs = millis();

  if (movement < STUCK_MIN_ENCODER_COUNTS) beginEscape();
}

void runEscapeBack() {
  float s = 0.0;
  if (validSideDistance(leftCM) && validSideDistance(rightCM))
    s = (leftCM < rightCM) ? -ESCAPE_STEER_DEG : +ESCAPE_STEER_DEG;
  setSteering(s);
  setMotor(-ESCAPE_BACK_PWM);

  bool done = encoderDistanceCMFrom(escapeStartTicks) >= ESCAPE_BACK_CM;
  bool timeout = millis() - escapeStartMs >= ESCAPE_BACK_TIMEOUT_MS;
  if (done || timeout) {
    stopMotor();
    centerSteering();
    escapeStartTicks = getEncoderTicks();
    escapeStartMs = millis();
    state = STATE_ESCAPE_FORWARD;
  }
}

void runEscapeForward() {
  float s = 0.0;
  if (validSideDistance(leftCM) && validSideDistance(rightCM))
    s = (leftCM < rightCM) ? +ESCAPE_STEER_DEG : -ESCAPE_STEER_DEG;
  setSteering(s);
  setMotor(ESCAPE_FORWARD_PWM);

  bool done = encoderDistanceCMFrom(escapeStartTicks) >= ESCAPE_FORWARD_CM;
  bool timeout = millis() - escapeStartMs >= ESCAPE_FORWARD_TIMEOUT_MS;
  if (done || timeout) {
    stopMotor();
    centerSteering();
    lastStuckTicks = getEncoderTicks();
    lastStuckCheckMs = millis();
    state = stateBeforeEscape;
  }
}

// ===================== START =====================
bool startButtonPressed() {
  if (digitalRead(PIN_START_BUTTON) != LOW) return false;
  delay(15);
  return digitalRead(PIN_START_BUTTON) == LOW;
}
void startRound() {
  stopMotor();
  centerSteering();

  resetYaw();
  resetEncoderTicks();

  turnCounter = 0;
  activeCornerIndex = 0;

  // First direction is UNKNOWN until camera sees BLUE/ORANGE.
  firstTurnDirection = 0;
  latchedDirection = 0;
  turnDirection = 0;

  // Final parking direction will be copied from the first camera corner.
  parkingDirection = 0;

  activePillarID = 0;
  pillarAvoidDirection = 0;
  pillarSavedCorridorYaw = 0.0;
  pillarOutYaw = 0.0;
  pillarDynamicTurnDeg = 0.0;
  pillarAvoidSeverity = 0.0;
  pillarDetectionXAtStart = 0;
  pillarIgnoreUntilClear = false;

  pillarConfidenceScore = 0;
  pillarConfidenceID = 0;

  pillarPassPhase = 0;
  pillarFrontSideEngaged = false;
  pillarRearSideEngaged = false;
  pillarFrontRiseCount = 0;
  pillarRearRiseCount = 0;

  pillarFrontSideStartCM = 999.0;
  pillarFrontSideMinCM = 999.0;
  pillarFrontSidePrevCM = 999.0;

  pillarRearSideStartCM = 999.0;
  pillarRearSideMinCM = 999.0;
  pillarRearSidePrevCM = 999.0;

  pillarTrackedFrontSideCM = 999.0;
  pillarTrackedRearSideCM = 999.0;

  lastPillarAvoidCompleteMs = 0;
  lastPillarSafeCheckMs = 0;

  pillarSmoothSteeringDeg = 0.0;
  pillarSmoothSteeringMicros = micros();

  previousYawError = 0.0;
  lastAcceptedLineMs = 0;

  cornerTriggerLocked = false;
  cornerRearmStartTicks = getEncoderTicks();
  cornerFinishedMs = 0;
  lastCompletedTurnDirection = 0;

  preTurnBackStartTicks = 0;
  preTurnBackStartMs = 0;
  preTurnBackSavedYaw = 0.0;

  lastStuckTicks = 0;
  lastStuckCheckMs = millis();

  // Straight starting reference.
  targetYaw = 0.0;

  Serial.println(
    F("# START: straight yaw=0, waiting BLUE/ORANGE")
  );

  Serial.println(
    F("# BLUE=LEFT, ORANGE=RIGHT")
  );

  buzzerStart();

  // Immediately enter normal corridor driving.
  state =
    STATE_CORRIDOR;

  while (
    digitalRead(
      PIN_START_BUTTON
    ) == LOW
  ) {
    delay(1);
  }
}

// ===================== SERIAL COMMANDS =====================
char commandBuffer[32];
uint8_t commandLength = 0;

bool commandAllowed() {
  return state == STATE_WAIT_START || state == STATE_STOPPED;
}
void printHelp() {
  Serial.println(F("# commands: help i2c us imu husky pillar enc button motorf motorb servol servoc servor buzz recal all"));
}
void scanI2C() {
  Serial.println(F("# I2C_SCAN_BEGIN"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("# I2C 0x"));
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
    }
  }
  Serial.println(F("# I2C_SCAN_END"));
}
void printSonarsNow() {
  readAllUltrasonicsNow();

  Serial.print(F("# SONAR F="));
  Serial.print(frontCM,1);

  Serial.print(F(" FL="));
  Serial.print(leftCM,1);

  Serial.print(F(" FR="));
  Serial.print(rightCM,1);

  Serial.print(F(" RL="));
  Serial.print(rearLeftCM,1);

  Serial.print(F(" RR="));
  Serial.print(rearRightCM,1);

  Serial.print(F(" B="));
  Serial.println(rearCM,1);
}
void printIMUNow() {
  if (!imuOK) {
    Serial.println(F("# MPU6050 NOT_CONNECTED"));
    return;
  }

  readIMUData();

  Serial.print(F("# MPU ACC="));
  Serial.print(latestAccelX, 3);
  Serial.print(',');
  Serial.print(latestAccelY, 3);
  Serial.print(',');
  Serial.print(latestAccelZ, 3);

  Serial.print(F(" GYRO_DPS="));
  Serial.print(latestGyroX, 2);
  Serial.print(',');
  Serial.print(latestGyroY, 2);
  Serial.print(',');
  Serial.print(latestGyroZ, 2);

  Serial.print(F(" YAW="));
  Serial.print(currentYaw, 2);

  Serial.print(F(" TEMP="));
  Serial.println(latestMPUTempC, 1);
}

void printHuskyNow() {
  Serial.print(F("# HUSKY OK=")); Serial.print(huskyOK ? 1 : 0);
  Serial.print(F(" COUNT=")); Serial.print(huskyResultCount);
  Serial.print(F(" LEARNED=")); Serial.print(huskyLearnedCount);
  Serial.print(F(" ID=")); Serial.print(huskyBestLearnedID);
  Serial.print(F(" X=")); Serial.print(huskyBestX);
  Serial.print(F(" Y=")); Serial.print(huskyBestY);
  Serial.print(F(" W=")); Serial.print(huskyBestW);
  Serial.print(F(" H=")); Serial.println(huskyBestH);
}

void printPillarNow() {
  Serial.print(F("# PILLAR visible="));
  Serial.print(pillarVisibleNow() ? 1 : 0);

  Serial.print(F(" color="));
  Serial.print(pillarName());

  Serial.print(F(" ID="));
  Serial.print(huskyBestPillarID);

  Serial.print(F(" X="));
  Serial.print(huskyBestPillarX);

  Serial.print(F(" Y="));
  Serial.print(huskyBestPillarY);

  Serial.print(F(" W="));
  Serial.print(huskyBestPillarW);

  Serial.print(F(" H="));
  Serial.print(huskyBestPillarH);

  Serial.print(F(" AREA="));
  Serial.println(huskyBestPillarArea);
}
void testMotorForward() {
  long b = getEncoderTicks(); setMotor(200); delay(500); stopMotor();
  Serial.print(F("# MOTOR_FORWARD ENC_DELTA=")); Serial.println(getEncoderTicks()-b);
}
void testMotorBackward() {
  long b = getEncoderTicks(); setMotor(-200); delay(500); stopMotor();
  Serial.print(F("# MOTOR_BACKWARD ENC_DELTA=")); Serial.println(getEncoderTicks()-b);
}
void fullHardwareTest() {
  scanI2C(); printSonarsNow(); printIMUNow(); printHuskyNow(); printPillarNow();
  Serial.print(F("# ENCODER=")); Serial.println(getEncoderTicks());
  Serial.print(F("# BUTTON=")); Serial.println(digitalRead(PIN_START_BUTTON)==LOW ? F("PRESSED") : F("RELEASED"));
  tone(PIN_BUZZER, 2200, 120); delay(180);
  setSteering(-TEST_STEER_DEG); delay(400); centerSteering(); delay(250);
  setSteering(+TEST_STEER_DEG); delay(400); centerSteering(); delay(250);
  testMotorForward(); delay(300); testMotorBackward();
}

void executeCommand() {
  commandBuffer[commandLength] = '\0';
  for (uint8_t i=0; i<commandLength; i++)
    if (commandBuffer[i]>='A' && commandBuffer[i]<='Z') commandBuffer[i] += ('a'-'A');

  if (strcmp(commandBuffer,"help")==0) printHelp();
  else if (!commandAllowed()) Serial.println(F("# COMMAND_BLOCKED robot is running"));
  else if (strcmp(commandBuffer,"i2c")==0) scanI2C();
  else if (strcmp(commandBuffer,"us")==0) printSonarsNow();
  else if (strcmp(commandBuffer,"imu")==0) printIMUNow();
  else if (strcmp(commandBuffer,"husky")==0) printHuskyNow();
  else if (strcmp(commandBuffer,"pillar")==0) printPillarNow();
  else if (strcmp(commandBuffer,"enc")==0) { Serial.print(F("# ENCODER=")); Serial.println(getEncoderTicks()); }
  else if (strcmp(commandBuffer,"button")==0) { Serial.print(F("# BUTTON=")); Serial.println(digitalRead(PIN_START_BUTTON)==LOW?F("PRESSED"):F("RELEASED")); }
  else if (strcmp(commandBuffer,"motorf")==0) testMotorForward();
  else if (strcmp(commandBuffer,"motorb")==0) testMotorBackward();
  else if (strcmp(commandBuffer,"servol")==0) setSteering(-TEST_STEER_DEG);
  else if (strcmp(commandBuffer,"servoc")==0) centerSteering();
  else if (strcmp(commandBuffer,"servor")==0) setSteering(+TEST_STEER_DEG);
  else if (strcmp(commandBuffer,"buzz")==0) tone(PIN_BUZZER,2200,250);
  else if (strcmp(commandBuffer,"recal")==0) { calibrateGyroQuick(); resetYaw(); Serial.print(F("# GYRO_BIAS=")); Serial.println(GYRO_Z_BIAS_DPS,5); }
  else if (strcmp(commandBuffer,"all")==0) fullHardwareTest();
  else { Serial.print(F("# UNKNOWN_COMMAND ")); Serial.println(commandBuffer); }
  commandLength = 0;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c=='\n' || c=='\r') {
      if (commandLength > 0) executeCommand();
    } else if (commandLength < sizeof(commandBuffer)-1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

// ===================== TELEMETRY =====================
// Use int instead of RobotState in this helper signature.
 // This avoids an Arduino IDE auto-prototype error where the generated
 // prototype can appear before the RobotState enum declaration.
// ===================== PARKING HELPERS =====================





bool parkingYawReached(
  float wantedYaw,
  int direction,
  float toleranceDeg
) {
  if (direction > 0) {
    return
      currentYaw >=
      (
        wantedYaw -
        toleranceDeg
      );
  }

  return
    currentYaw <=
    (
      wantedYaw +
      toleranceDeg
    );
}

float parkingTurnSteering(
  float wantedYaw,
  int direction,
  float minSteer,
  float maxSteer,
  float slowZoneDeg
) {
  float remaining =
    fabs(
      wantedYaw -
      currentYaw
    );

  float ratio =
    constrain(
      remaining /
      slowZoneDeg,
      0.0,
      1.0
    );

  float magnitude =
    minSteer +
    ratio *
    (
      maxSteer -
      minSteer
    );

  return
    -direction *
    magnitude;
}

float parkingYawOnlySteering(
  float wantedYaw,
  bool reverseMotion,
  float kp,
  float maxSteer
) {
  float error =
    wantedYaw -
    currentYaw;

  float command =
    -kp *
    error;

  // Steering/yaw relationship reverses while backing up.
  if (reverseMotion)
    command = -command;

  return constrain(
    command,
    -maxSteer,
    +maxSteer
  );
}

void startFinalParkingSequence() {
  // Final sequence uses the race direction learned from camera.
  if (parkingDirection == 0) {
    if (latchedDirection != 0)
      parkingDirection = latchedDirection;
    else if (firstTurnDirection != 0)
      parkingDirection = firstTurnDirection;
    else
      parkingDirection = (turnDirection != 0) ? turnDirection : +1;
  }
  // Final special sequence replaces the normal last corner turn.
  clearPillarCandidateForCorner();

  cornerTriggerLocked = true;

  finalPreCornerYaw =
    targetYaw;

  finalMainCorridorYaw =
    finalPreCornerYaw +
    parkingDirection *
    FINAL_CORRIDOR_TURN_DEG;

  finalShiftDirection =
    parkingDirection *
    FINAL_SHIFT_DIRECTION_MULTIPLIER;

  finalParkEntryDirection =
    parkingDirection *
    FINAL_PARK_ENTRY_DIRECTION_MULTIPLIER;

  // Keep old corridor yaw while approaching the 5-cm wall point.
  targetYaw =
    finalPreCornerYaw;

  Serial.println(
    F("# FINAL PARKING SEQUENCE START")
  );

  Serial.print(
    F("# raceDir=")
  );
  Serial.println(
    parkingDirection > 0
      ? F("LEFT")
      : F("RIGHT")
  );

  Serial.print(
    F("# preYaw=")
  );
  Serial.print(
    finalPreCornerYaw,
    2
  );

  Serial.print(
    F(" finalCorridorYaw=")
  );
  Serial.println(
    finalMainCorridorYaw,
    2
  );

  state =
    STATE_FINAL_TO_WALL;
}

void clearPillarCandidateForCorner() {
  huskyBestPillarID = 0;
  huskyBestPillarX = 0;
  huskyBestPillarY = 0;
  huskyBestPillarW = 0;
  huskyBestPillarH = 0;
  huskyBestPillarArea = 0;
  lastGoodPillarFrameMs = 0;

  pillarConfidenceScore = 0;
  pillarConfidenceID = 0;

  activePillarID = 0;
  pillarDetectionXAtStart = 0;
  pillarDynamicTurnDeg = 0.0;
  pillarAvoidSeverity = 0.0;

  pillarPassPhase = 0;
  pillarFrontSideEngaged = false;
  pillarRearSideEngaged = false;
  pillarFrontRiseCount = 0;
  pillarRearRiseCount = 0;
}

const char* stateName(int s) {
  switch (s) {
    case STATE_WAIT_START: return "WAIT";


    case STATE_CORRIDOR: return "CORRIDOR";
    case STATE_APPROACH_CORNER: return "APPROACH";
    case STATE_PRE_TURN_BACK: return "PRE_BACK";
    case STATE_TURNING: return "TURN";

    case STATE_PILLAR_STOP: return "P_STOP";
    case STATE_PILLAR_TURN_OUT: return "P_TURN";
    case STATE_PILLAR_FORWARD: return "P_FWD";
    case STATE_PILLAR_RETURN_YAW: return "P_RETURN";

    case STATE_ESCAPE_BACK: return "ESC_BACK";
    case STATE_ESCAPE_FORWARD: return "ESC_FWD";

    case STATE_FINAL_TO_WALL: return "F_TO_5";
    case STATE_FINAL_BACK: return "F_BACK";
    case STATE_FINAL_CORRIDOR_TURN: return "F_TURN90";
    case STATE_FINAL_CENTER_FORWARD: return "F_TO_20";
    case STATE_FINAL_SHIFT_TURN: return "F_SHIFT";
    case STATE_FINAL_SHIFT_FORWARD: return "F_SHIFT_FWD";
    case STATE_FINAL_RETURN_MAIN_YAW: return "F_RETURN";
    case STATE_FINAL_BEFORE_PARK: return "F_BEFORE_PK";
    case STATE_FINAL_PARK_ENTRY_TURN: return "F_PK_TURN";
    case STATE_FINAL_PARK_FORWARD: return "F_PK_FWD";

    case STATE_FINAL_FIND_INTERSECTION: return "FINAL_OLD";
    case STATE_STOPPED: return "STOP";

    default: return "?";
  }
}

void streamTelemetry() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs < TELEMETRY_INTERVAL_MS) return;
  lastMs = millis();

  Serial.print(F("DATA ms=")); Serial.print(millis());
  Serial.print(F(" state=")); Serial.print(stateName(state));
  Serial.print(F(" imu=")); Serial.print(imuOK?1:0);
  Serial.print(F(" husky=")); Serial.print(huskyOK?1:0);
  Serial.print(F(" yaw=")); Serial.print(currentYaw,2);
  Serial.print(F(" target=")); Serial.print(targetYaw,2);
  Serial.print(F(" gyroX=")); Serial.print(latestGyroX,2);
  Serial.print(F(" gyroY=")); Serial.print(latestGyroY,2);
  Serial.print(F(" gyroZ=")); Serial.print(latestGyroZ,2);
  Serial.print(F(" yawScale=")); Serial.print(YAW_SCALE,4);
  Serial.print(F(" mpuT=")); Serial.print(latestMPUTempC,1);
  Serial.print(F(" F=")); Serial.print(frontCM,1);
  Serial.print(F(" L=")); Serial.print(leftCM,1);
  Serial.print(F(" R=")); Serial.print(rightCM,1);
  Serial.print(F(" RL=")); Serial.print(rearLeftCM,1);
  Serial.print(F(" RR=")); Serial.print(rearRightCM,1);
  Serial.print(F(" LOK=")); Serial.print(validSideDistance(leftCM) ? 1 : 0);
  Serial.print(F(" ROK=")); Serial.print(validSideDistance(rightCM) ? 1 : 0);
  Serial.print(F(" wall=")); Serial.print(lastWallCorrectionDeg,1);
  Serial.print(F(" avoid=")); Serial.print(wallAvoidActive ? 1 : 0);

  bool turnSideOpen = true;
  if (USE_SIDE_OPEN_TURN_GATE) {
    if (turnDirection > 0) {
      turnSideOpen =
        validTurnOpeningDistance(leftCM) &&
        leftCM >= TURN_SIDE_OPEN_MIN_CM;
    }
    else if (turnDirection < 0) {
      turnSideOpen =
        validTurnOpeningDistance(rightCM) &&
        rightCM >= TURN_SIDE_OPEN_MIN_CM;
    }
  }

  float turnSideCM = 0.0;
  if (turnDirection > 0)
    turnSideCM = leftCM;
  else if (turnDirection < 0)
    turnSideCM = rightCM;

  Serial.print(F(" turnSideCM=")); Serial.print(turnSideCM,1);
  Serial.print(F(" sideOpen=")); Serial.print(turnSideOpen ? 1 : 0);
  Serial.print(F(" sideOpenMin=")); Serial.print(TURN_SIDE_OPEN_MIN_CM,1);

  int fallbackPreviewDir =
    getUltrasonicFallbackTurnDirection();

  Serial.print(F(" usFallback="));
  Serial.print(fallbackPreviewDir);

  Serial.print(F(" usSideMin="));
  Serial.print(
    ULTRASONIC_FALLBACK_SIDE_MIN_CM,
    1
  );

  Serial.print(F(" PcornerBlock="));
  Serial.print(
    suppressPillarBecauseTurnSideIsOpen()
      ? 1
      : 0
  );

  Serial.print(F(" CornerLock="));
  Serial.print(
    cornerTriggerLocked ? 1 : 0
  );

  if (cornerTriggerLocked) {
    Serial.print(F(" RearmCM="));
    Serial.print(
      encoderDistanceCMFrom(
        cornerRearmStartTicks
      ),
      1
    );
  }

  Serial.print(F(" finalStop=")); Serial.print(FINAL_STOP_FRONT_CM,1);
  Serial.print(F(" B=")); Serial.print(rearCM,1);
  Serial.print(F(" enc=")); Serial.print(getEncoderTicks());
  Serial.print(F(" motor=")); Serial.print(lastMotorCommand);
  Serial.print(F(" steer=")); Serial.print(lastSteeringCommand,1);
  Serial.print(F(" Hcnt=")); Serial.print(huskyResultCount);
  Serial.print(F(" Hlearn=")); Serial.print(huskyLearnedCount);
  Serial.print(F(" Hid=")); Serial.print(huskyBestLearnedID);
  Serial.print(F(" Hx=")); Serial.print(huskyBestX);
  Serial.print(F(" Hy=")); Serial.print(huskyBestY);

  // Pillar detection telemetry. Open round naturally stays NONE/0.
  Serial.print(F(" pillar=")); Serial.print(pillarName());
  Serial.print(F(" Pvis=")); Serial.print(pillarVisibleNow() ? 1 : 0);
  Serial.print(F(" Pid=")); Serial.print(huskyBestPillarID);
  Serial.print(F(" Px=")); Serial.print(huskyBestPillarX);
  Serial.print(F(" Py=")); Serial.print(huskyBestPillarY);
  Serial.print(F(" Pw=")); Serial.print(huskyBestPillarW);
  Serial.print(F(" Ph=")); Serial.print(huskyBestPillarH);
  Serial.print(F(" Parea=")); Serial.print(huskyBestPillarArea);
  Serial.print(F(" Pconf=")); Serial.print(pillarConfidenceScore);

  Serial.print(F(" PavoidID=")); Serial.print(activePillarID);
  Serial.print(F(" Pdir=")); Serial.print(pillarAvoidDirection);
  Serial.print(F(" PstartX=")); Serial.print(pillarDetectionXAtStart);
  Serial.print(F(" Pdeg=")); Serial.print(pillarDynamicTurnDeg,1);
  Serial.print(F(" Psev=")); Serial.print(pillarAvoidSeverity,2);
  Serial.print(F(" Pignore=")); Serial.print(pillarIgnoreUntilClear ? 1 : 0);
  Serial.print(F(" PsavedYaw=")); Serial.print(pillarSavedCorridorYaw,1);
  Serial.print(F(" PoutYaw=")); Serial.print(pillarOutYaw,1);
  Serial.print(F(" Pphase=")); Serial.print(pillarPassPhase);
  Serial.print(F(" PfrontSide=")); Serial.print(pillarTrackedFrontSideCM,1);
  Serial.print(F(" PfrontMin=")); Serial.print(pillarFrontSideMinCM,1);
  Serial.print(F(" PrearSide=")); Serial.print(pillarTrackedRearSideCM,1);
  Serial.print(F(" PrearMin=")); Serial.print(pillarRearSideMinCM,1);
  Serial.print(F(" PfrontEng=")); Serial.print(pillarFrontSideEngaged ? 1 : 0);
  Serial.print(F(" PrearEng=")); Serial.print(pillarRearSideEngaged ? 1 : 0);
  Serial.print(F(" Parea=")); Serial.print(huskyBestPillarArea);
  Serial.print(F(" Py=")); Serial.print(huskyBestPillarY);

  Serial.print(F(" PreBackOn=")); Serial.print(ENABLE_BACK_BEFORE_TURN ? 1 : 0);
  Serial.print(F(" PreBackSet=")); Serial.print(BACK_BEFORE_TURN_CM[activeCornerIndex],1);

  Serial.print(F(" turns=")); Serial.print(turnCounter);
  Serial.print(F(" corner=")); Serial.print(activeCornerIndex+1);
  Serial.print(F(" latch=")); Serial.print(latchedDirection);
  Serial.print(F(" btn=")); Serial.println(digitalRead(PIN_START_BUTTON)==LOW ? 1 : 0);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println();
  Serial.println(F("======================================"));
  Serial.println(F("# MEGA BOOT / SETUP START"));
  Serial.println(F("======================================"));
  pinMode(PIN_BTS_RPWM, OUTPUT);
  pinMode(PIN_BTS_LPWM, OUTPUT);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  stopMotor();
  steeringServo.attach(PIN_STEERING);
  centerSteering();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, RISING);

  // ----------------------------------------------------------
  // MPU6050 ONLY on I2C
  // ----------------------------------------------------------
  Wire.begin();

  initializeIMU();

  if (imuOK)
    calibrateGyroQuick();

  resetYaw();

  // ----------------------------------------------------------
  // HUSKYLENS ONLY on UART Serial1
  // ----------------------------------------------------------
  initializeHusky();

  state = STATE_WAIT_START;
  printHelp();
  Serial.print(F("# BOOT MPU6050_I2C=")); Serial.print(imuOK?1:0);
  Serial.print(F(" HUSKYLENS_UART=")); Serial.print(huskyOK?1:0);
  Serial.print(F(" GYRO_BIAS=")); Serial.println(GYRO_Z_BIAS_DPS,5);
  Serial.println(F("# READY - continuous telemetry active - press START button"));
  Serial.println(F("# SETUP COMPLETE - MPU WILL NOT RECALIBRATE AGAIN"));

  buzzerReady();
  Serial.println(F("# READY BUZZER DONE - PRESS START BUTTON"));

  Serial.print(F("# PRE-TURN BACK ENABLED="));
  Serial.println(ENABLE_BACK_BEFORE_TURN ? 1 : 0);
  Serial.print(F("# PRE-TURN BACK CM C1/C2/C3/C4="));
  Serial.print(BACK_BEFORE_TURN_CM[0], 1);
  Serial.print(',');
  Serial.print(BACK_BEFORE_TURN_CM[1], 1);
  Serial.print(',');
  Serial.print(BACK_BEFORE_TURN_CM[2], 1);
  Serial.print(',');
  Serial.println(BACK_BEFORE_TURN_CM[3], 1);

  Serial.println(F("# ================= FINAL TUNES ================="));
  Serial.println(F("# SERVO_CENTER=98 MAX_STEER=60 CORRIDOR_MAX=55 TURN_STEER=55"));
  Serial.println(F("# DRIVE/APPROACH/TURN/ESCAPE PWM = 255"));
  Serial.println(F("# YAW_KP=4.20 YAW_KD=0.16 WALL_KP=4.60"));
  Serial.println(F("# CORNER FORWARD CM = 0 / 40 / 40 / 0"));
  Serial.println(F("# CORNER FRONT CM = 140 / 45 / 45 / 140"));
  Serial.println(F("# NORMAL TURN SIDE OPEN MIN = 65cm"));
  Serial.println(F("# US FALLBACK SIDE MIN = 100cm"));
  Serial.println(F("# CORNER ONE-SHOT: rearm after 35cm + side closed, force at 60cm"));
  Serial.println(F("# FINAL STOP FRONT = 80cm"));
  Serial.println(F("# PILLAR DIRECTION: RED->RIGHT GREEN->LEFT"));
  Serial.println(F("# RED pillar IDs = 5,3 | GREEN pillar IDs = 4,6"));
  Serial.println(F("# PILLAR NEAR: area>=500 AND (Y>=105 OR bottom>=155)"));
  Serial.println(F("# RED safe X<=180 full avoid X>=430"));
  Serial.println(F("# GREEN safe X>=300 full avoid X<=150"));
  Serial.println(F("# PILLAR TURN = 25..60deg"));
  Serial.println(F("# PILLAR FORWARD = 45cm + up to 15cm extra (25% shorter)"));
  Serial.println(F("# PILLAR STOP MODE: STOP BEFORE AVOID + STOP BETWEEN PHASES"));
  Serial.println(F("# PILLAR DETECT EARLIER: area>=500 AND (Y>=105 OR bottom>=155)"));
  Serial.println(F("# START DIR FROM CAMERA: BLUE LEFT, ORANGE RIGHT"));  Serial.println(F("# FINAL: front5 -> back -> 90 -> front20 -> 45 -> shiftFWD -> mainYaw -> distance -> park90 -> front5"));
  Serial.println(F("# FINAL DIRECTION: race 90 SAME, shift45 SAME, park90 OPPOSITE"));
  Serial.println(F("# PILLAR TUNE ONLY 5: AREA, MAXTURN, PASSpwm, SMOOTHdps, REARsafeCM"));
  Serial.println(F("# NEW US: D27 rear-left, D28 rear-right"));
  Serial.println(F("# SMART PILLAR: camera confidence + adaptive speed + front/rear side tracking + S-curve"));
  Serial.println(F("# PILLAR STEERING: smooth yaw-scaled + slew limited"));
  Serial.println(F("# CLOSE PILLAR FORCE AVOID: area>=1200 OR h>=55 OR bottom>=190"));
  Serial.println(F("# CLOSE PILLAR MIN AVOID: severity>=0.70 turn>=50deg"));
  Serial.println(F("# ONE CODE: OPEN + OBSTACLE"));
  Serial.println(F("# ==============================================="));
}

// ===================== LOOP =====================
void loop() {
  // ----------------------------------------------------------
  // YAW FIRST
  // ----------------------------------------------------------
  // Read before slower sensors so rotation is never waiting on
  // ultrasonic or HUSKYLENS communication.
  updateYaw();

  // One ultrasonic sensor is read per slot.
  updateUltrasonics();

  // Sample again immediately after the potentially blocking pulseIn().
  updateYaw();

  // HUSKYLENS is UART and can occasionally take longer than the
  // normal loop. Sample yaw immediately before and after it.
  updateYaw();
  updateHusky();
  updateYaw();

  // Sensors are initialized ONCE in setup().
  // Do not automatically begin/recalibrate them during the run.
  readSerialCommands();

  updateYaw();

  streamTelemetry();

  updateYaw();

  switch (state) {
    case STATE_WAIT_START:
      stopMotor();

      if (startButtonPressed()) {
        centerSteering();
        startRound();
      }
      break;

    case STATE_CORRIDOR: {
      setMotor(DRIVE_PWM);

      updateYaw();

      setSteering(
        corridorSteering()
      );

      // ======================================================
      // POST-CORNER LOCK
      // ======================================================
      updateCornerTriggerRearm();

      // While still leaving the previous physical corner:
      // - no camera intersection
      // - no ultrasonic fallback
      // - no pillar avoidance
      //
      // Just drive out of the corner on the completed target yaw.
      if (cornerTriggerLocked) {
        break;
      }

      // ======================================================
      // PRIORITY 1: CAMERA INTERSECTION / CORNER
      // ======================================================
      int event =
        readIntersectionEvent();

      if (event != 0) {
        // Once the robot enters APPROACH_CORNER, pillar avoidance
        // is completely ignored until the corner is finished.
        acceptIntersection(event);
        break;
      }

      // ======================================================
      // PRIORITY 2: ULTRASONIC FALLBACK CORNER
      // ======================================================
      int fallbackDirection =
        getUltrasonicFallbackTurnDirection();

      if (fallbackDirection != 0) {
        startUltrasonicFallbackCorner(
          fallbackDirection
        );
        break;
      }

      // ======================================================
      // PRIORITY 3: PILLAR
      // ======================================================
      // If the side of the known turning direction is already open
      // by the previously tuned 100cm value, we are entering a corner,
      // so do NOT react to pillars.
      bool ignorePillarForCorner =
        suppressPillarBecauseTurnSideIsOpen();

      if (
        !ignorePillarForCorner &&
        currentPillarIsVisible()
      ) {
        startPillarAvoidance();
        break;
      }

      break;
    }

    case STATE_APPROACH_CORNER: {
      setMotor(APPROACH_PWM);

      updateYaw();

      setSteering(
        corridorSteering()
      );

      float traveled =
        encoderDistanceCMFrom(
          cornerApproachStartTicks
        );

      float needed =
        CORNER_FORWARD_CM[
          activeCornerIndex
        ];

      float safety =
        CORNER_FRONT_TURN_CM[
          activeCornerIndex
        ];

      bool encoderTurnReady =
        USE_ENCODER_CORNER_TRIGGER &&
        traveled >= needed;

      bool frontTurnReady =
        USE_FRONT_CORNER_TRIGGER &&
        safety > 0.0 &&
        frontCM <= safety;

      bool normalTurnTrigger =
        encoderTurnReady ||
        frontTurnReady;

      // ------------------------------------------------------
      // SIDE OPENING GATE
      // ------------------------------------------------------
      // LEFT  (+1) -> check LEFT ultrasonic
      // RIGHT (-1) -> check RIGHT ultrasonic
      bool sideOpenReady = true;

      if (USE_SIDE_OPEN_TURN_GATE) {
        if (turnDirection > 0) {
          sideOpenReady =
            validTurnOpeningDistance(leftCM) &&
            leftCM >= TURN_SIDE_OPEN_MIN_CM;
        }
        else {
          sideOpenReady =
            validTurnOpeningDistance(rightCM) &&
            rightCM >= TURN_SIDE_OPEN_MIN_CM;
        }
      }

      if (
        normalTurnTrigger &&
        sideOpenReady
      ) {
        // Normal corners optionally reverse by encoder first.
        // The special 12th/final corner bypasses this extra reverse
        // and keeps its existing FINAL_BACK_CM behavior.
        beginPreTurnBackOrTurn();
      }

      break;
    }

    case STATE_PRE_TURN_BACK:
      runPreTurnBack();
      break;

    case STATE_TURNING:
      // Make sure turn control always uses the newest yaw.
      updateYaw();
      runCornerTurn();
      updateYaw();
      break;

    case STATE_PILLAR_STOP:
      runPillarStop();
      break;

    case STATE_PILLAR_TURN_OUT:
      runPillarTurnOut();
      break;

    case STATE_PILLAR_FORWARD:
      runPillarForward();
      break;

    case STATE_PILLAR_RETURN_YAW:
      runPillarReturnYaw();
      break;

    case STATE_ESCAPE_BACK:
      runEscapeBack();
      break;

    case STATE_ESCAPE_FORWARD:
      runEscapeForward();
      break;

    case STATE_FINAL_TO_WALL: {
      // Final corner: first continue straight on the OLD yaw.
      // NO wall centering / wall avoid here.
      setMotor(
        FINAL_APPROACH_PWM
      );

      updateYaw();

      setSteering(
        parkingYawOnlySteering(
          finalPreCornerYaw,
          false,
          FINAL_YAW_KP,
          FINAL_YAW_MAX_STEER_DEG
        )
      );

      bool frontValid =
        frontCM > 0.0 &&
        frontCM < 500.0;

      if (
        frontValid &&
        frontCM <=
          FINAL_WALL_FRONT_CM
      ) {
        stopMotor();
        centerSteering();

        finalStageStartTicks =
          getEncoderTicks();

        Serial.print(
          F("# FINAL FRONT 5 reached=")
        );
        Serial.println(
          frontCM,
          1
        );

        state =
          STATE_FINAL_BACK;
      }

      break;
    }

    case STATE_FINAL_BACK: {
      // Reverse by encoder, holding the OLD corridor yaw.
      setMotor(
        -FINAL_BACK_PWM
      );

      updateYaw();

      setSteering(
        parkingYawOnlySteering(
          finalPreCornerYaw,
          true,
          FINAL_YAW_KP,
          FINAL_YAW_MAX_STEER_DEG
        )
      );

      float traveled =
        encoderDistanceCMFrom(
          finalStageStartTicks
        );

      if (
        traveled >=
        FINAL_BACK_CM
      ) {
        stopMotor();
        centerSteering();

        targetYaw =
          finalMainCorridorYaw;

        Serial.print(
          F("# FINAL BACK DONE cm=")
        );
        Serial.println(
          traveled,
          1
        );

        state =
          STATE_FINAL_CORRIDOR_TURN;
      }

      break;
    }

    case STATE_FINAL_CORRIDOR_TURN: {
      updateYaw();

      if (
        parkingYawReached(
          finalMainCorridorYaw,
          parkingDirection,
          FINAL_TURN_TOLERANCE_DEG
        )
      ) {
        targetYaw =
          finalMainCorridorYaw;

        // This special 90-degree turn IS the last normal corner.
        turnCounter =
          TOTAL_TURNS;

        setMotor(
          FINAL_FORWARD_PWM
        );

        Serial.print(
          F("# FINAL 90 COMPLETE yaw=")
        );
        Serial.println(
          currentYaw,
          2
        );

        state =
          STATE_FINAL_CENTER_FORWARD;

        break;
      }

      setMotor(
        FINAL_TURN_PWM
      );

      setSteering(
        parkingTurnSteering(
          finalMainCorridorYaw,
          parkingDirection,
          FINAL_TURN_MIN_STEER_DEG,
          FINAL_TURN_MAX_STEER_DEG,
          FINAL_TURN_SLOW_ZONE_DEG
        )
      );

      break;
    }

    case STATE_FINAL_CENTER_FORWARD: {
      // YAW CENTERING ONLY.
      // Deliberately NO wallCorrection()/corridorSteering().
      setMotor(
        FINAL_FORWARD_PWM
      );

      updateYaw();

      setSteering(
        parkingYawOnlySteering(
          finalMainCorridorYaw,
          false,
          FINAL_YAW_KP,
          FINAL_YAW_MAX_STEER_DEG
        )
      );

      bool frontValid =
        frontCM > 0.0 &&
        frontCM < 500.0;

      if (
        frontValid &&
        frontCM <=
          FINAL_CENTER_FRONT_CM
      ) {
        finalShiftYaw =
          finalMainCorridorYaw +
          finalShiftDirection *
          FINAL_SHIFT_ANGLE_DEG;

        targetYaw =
          finalShiftYaw;

        Serial.print(
          F("# FINAL FRONT 20 reached=")
        );
        Serial.print(
          frontCM,
          1
        );

        Serial.print(
          F(" shiftYaw=")
        );
        Serial.println(
          finalShiftYaw,
          2
        );

        state =
          STATE_FINAL_SHIFT_TURN;
      }

      break;
    }

    case STATE_FINAL_SHIFT_TURN: {
      updateYaw();

      if (
        parkingYawReached(
          finalShiftYaw,
          finalShiftDirection,
          FINAL_TURN_TOLERANCE_DEG
        )
      ) {
        // Hold the 45-degree yaw and move forward by encoder
        // BEFORE returning to the main corridor yaw.
        targetYaw =
          finalShiftYaw;

        finalStageStartTicks =
          getEncoderTicks();

        setMotor(
          FINAL_FORWARD_PWM
        );

        Serial.print(
          F("# FINAL SHIFT 45 COMPLETE yaw=")
        );
        Serial.print(
          currentYaw,
          2
        );

        Serial.print(
          F(" moveCM=")
        );
        Serial.println(
          FINAL_SHIFT_FORWARD_CM,
          1
        );

        state =
          STATE_FINAL_SHIFT_FORWARD;

        break;
      }

      setMotor(
        FINAL_TURN_PWM
      );

      setSteering(
        parkingTurnSteering(
          finalShiftYaw,
          finalShiftDirection,
          FINAL_TURN_MIN_STEER_DEG,
          FINAL_TURN_MAX_STEER_DEG,
          FINAL_TURN_SLOW_ZONE_DEG
        )
      );

      break;
    }

    case STATE_FINAL_SHIFT_FORWARD: {
      // Move along the 45-degree heading using encoder distance.
      // YAW ONLY: no wall centering / wall avoidance here.
      setMotor(
        FINAL_FORWARD_PWM
      );

      updateYaw();

      setSteering(
        parkingYawOnlySteering(
          finalShiftYaw,
          false,
          FINAL_YAW_KP,
          FINAL_YAW_MAX_STEER_DEG
        )
      );

      float traveled =
        encoderDistanceCMFrom(
          finalStageStartTicks
        );

      if (
        traveled >=
        FINAL_SHIFT_FORWARD_CM
      ) {
        // Now return to the exact main corridor yaw.
        targetYaw =
          finalMainCorridorYaw;

        Serial.print(
          F("# FINAL SHIFT FORWARD DONE cm=")
        );
        Serial.print(
          traveled,
          1
        );

        Serial.print(
          F(" returnMainYaw=")
        );
        Serial.println(
          finalMainCorridorYaw,
          2
        );

        state =
          STATE_FINAL_RETURN_MAIN_YAW;
      }

      break;
    }

    case STATE_FINAL_RETURN_MAIN_YAW: {
      updateYaw();

      int returnDirection =
        -finalShiftDirection;

      if (
        parkingYawReached(
          finalMainCorridorYaw,
          returnDirection,
          FINAL_TURN_TOLERANCE_DEG
        )
      ) {
        targetYaw =
          finalMainCorridorYaw;

        finalStageStartTicks =
          getEncoderTicks();

        Serial.print(
          F("# FINAL RETURN MAIN YAW=")
        );
        Serial.println(
          currentYaw,
          2
        );

        state =
          STATE_FINAL_BEFORE_PARK;

        break;
      }

      setMotor(
        FINAL_TURN_PWM
      );

      setSteering(
        parkingTurnSteering(
          finalMainCorridorYaw,
          returnDirection,
          FINAL_TURN_MIN_STEER_DEG,
          FINAL_TURN_MAX_STEER_DEG,
          FINAL_TURN_SLOW_ZONE_DEG
        )
      );

      break;
    }

    case STATE_FINAL_BEFORE_PARK: {
      // Tunable straight positioning distance before entering parking.
      setMotor(
        FINAL_FORWARD_PWM
      );

      updateYaw();

      setSteering(
        parkingYawOnlySteering(
          finalMainCorridorYaw,
          false,
          FINAL_YAW_KP,
          FINAL_YAW_MAX_STEER_DEG
        )
      );

      float traveled =
        encoderDistanceCMFrom(
          finalStageStartTicks
        );

      if (
        traveled >=
        FINAL_BEFORE_PARK_CM
      ) {
        finalParkEntryYaw =
          finalMainCorridorYaw +
          finalParkEntryDirection *
          FINAL_PARK_ENTRY_DEG;

        targetYaw =
          finalParkEntryYaw;

        Serial.print(
          F("# FINAL BEFORE PARK DONE cm=")
        );
        Serial.print(
          traveled,
          1
        );

        Serial.print(
          F(" parkYaw=")
        );
        Serial.println(
          finalParkEntryYaw,
          2
        );

        state =
          STATE_FINAL_PARK_ENTRY_TURN;
      }

      break;
    }

    case STATE_FINAL_PARK_ENTRY_TURN: {
      updateYaw();

      if (
        parkingYawReached(
          finalParkEntryYaw,
          finalParkEntryDirection,
          FINAL_TURN_TOLERANCE_DEG
        )
      ) {
        targetYaw =
          finalParkEntryYaw;

        setMotor(
          FINAL_FORWARD_PWM
        );

        state =
          STATE_FINAL_PARK_FORWARD;

        break;
      }

      setMotor(
        FINAL_TURN_PWM
      );

      setSteering(
        parkingTurnSteering(
          finalParkEntryYaw,
          finalParkEntryDirection,
          FINAL_TURN_MIN_STEER_DEG,
          FINAL_TURN_MAX_STEER_DEG,
          FINAL_TURN_SLOW_ZONE_DEG
        )
      );

      break;
    }

    case STATE_FINAL_PARK_FORWARD: {
      setMotor(
        FINAL_FORWARD_PWM
      );

      updateYaw();

      setSteering(
        parkingYawOnlySteering(
          finalParkEntryYaw,
          false,
          FINAL_YAW_KP,
          FINAL_YAW_MAX_STEER_DEG
        )
      );

      bool frontValid =
        frontCM > 0.0 &&
        frontCM < 500.0;

      if (
        frontValid &&
        frontCM <=
          FINAL_PARK_STOP_FRONT_CM
      ) {
        stopMotor();
        centerSteering();

        buzzerFinish();

        Serial.print(
          F("# PARKING COMPLETE front=")
        );
        Serial.println(
          frontCM,
          1
        );

        state =
          STATE_STOPPED;
      }

      break;
    }

    case STATE_FINAL_FIND_INTERSECTION:
      // Old final behavior is intentionally replaced.
      startFinalParkingSequence();
      break;

    case STATE_STOPPED:
      stopMotor();
      break;
  }

  updateYaw();

  monitorStuck();
}

