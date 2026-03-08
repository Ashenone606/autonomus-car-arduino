// ==============================
// SMART ULTRASONIC AVOIDANCE ROBOT
// WITH INTELLIGENT ESCAPE SEQUENCE
// ==============================

// ---------- L298N Pins ----------
#define ENA 3
#define IN1 4
#define IN2 5
#define IN3 7
#define IN4 8
#define ENB 6

// ---------- Ultrasonic Sensors ----------
#define ULTRA_LEFT_TRIG 9
#define ULTRA_LEFT_ECHO 10
#define ULTRA_RIGHT_TRIG 11
#define ULTRA_RIGHT_ECHO 12

// ---------- IR Sensor (REAR) ----------
#define IR_REAR_SENSOR A0

// ---------- Button ----------
#define START_BUTTON 2

// ---------- Parameters ----------
#define OBSTACLE_DISTANCE 30
#define CLOSE_OBSTACLE_DISTANCE 15
#define NAVIGATION_MAX_DISTANCE 50
#define STALL_MAX_DISTANCE 350
#define MOTOR_SPEED 90
#define TURN_TIME 525
#define LOOP_INTERVAL 20
#define RUN_TIME 30000UL
#define BACKUP_DURATION 1300
#define PAUSE_TIME 200

// ---------- Smart Backup ----------
#define BACKUP_SENSOR_CHECK_TIME 600
#define CLEAR_THRESHOLD 25

// ---------- STUCK DETECTION ----------
#define MAX_CONSECUTIVE_BACKUPS 3
#define BACKUP_RESET_TIME 5000
int consecutiveBackups = 0;
unsigned long lastBackupTime = 0;

// ---------- FORWARD STALL DETECTION ----------
#define FORWARD_STALL_TIMEOUT 3000
#define MIN_MOVEMENT_THRESHOLD 5
unsigned long lastMovementTime = 0;
long lastLeftDistance = 0;
long lastRightDistance = 0;

// ---------- TURN PATTERN DETECTION ----------
#define MAX_CONSECUTIVE_SAME_TURNS 3
#define PATTERN_RESET_TIME 8000
int consecutiveLeftTurns = 0;
int consecutiveRightTurns = 0;
unsigned long lastTurnTime = 0;

// ---------- ESCAPE CONTROL ----------
bool inEscapeSequence = false;

// ---------- Globals ----------
bool missionActive = false;
unsigned long missionStartTime = 0;
bool isBackingUp = false;
unsigned long backupStartTime = 0;
bool isPaused = false;
unsigned long pauseStartTime = 0;
bool turnAfterPause = true;

// ==============================
// SETUP
// ==============================
void setup() {
  Serial.begin(9600);
  Serial.println("=== SMART AVOIDANCE ROBOT ===");
  Serial.println("Escape: Backup → IR check → Clearest side or 180° spin");
  Serial.println("Press button to start");
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);
  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);
  
  pinMode(IR_REAR_SENSOR, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  
  stopMotors();
}

// ==============================
// MAIN LOOP
// ==============================
void loop() {
  // --- Wait for button ---
  if (!missionActive) {
    stopMotors();
    if (digitalRead(START_BUTTON) == LOW) {
      delay(50);
      missionStartTime = millis();
      missionActive = true;
      isBackingUp = false;
      isPaused = false;
      consecutiveBackups = 0;
      resetTurnPattern();
      resetForwardStallDetection();
      inEscapeSequence = false;
      Serial.println("\n>>> MISSION STARTED <<<");
    }
    return;
  }

  // --- Check 30 seconds ---
  if (missionActive && millis() - missionStartTime >= RUN_TIME) {
    stopMotors();
    missionActive = false;
    Serial.println("\n>>> MISSION COMPLETE <<<");
    return;
  }

  // --- Check if in escape sequence ---
  if (inEscapeSequence) {
    return; // Don't do anything else while escaping
  }

  // --- Handle pause state ---
  if (isPaused) {
    if (millis() - pauseStartTime >= PAUSE_TIME) {
      Serial.println("PAUSE complete - EXECUTING TURN");
      isPaused = false;
      if (turnAfterPause) {
        turnLeftWithPattern();
      } else {
        turnRightWithPattern();
      }
      resetForwardStallDetection();
      return;
    } else {
      return;
    }
  }

  // --- Handle backup state ---
  if (isBackingUp) {
    handleSmartBackupSequence();
    return;
  }

  // --- Read sensors ---
  long leftRaw = readUltrasonic(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  delay(10);
  long rightRaw = readUltrasonic(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);

  // === SEPARATE FILTERS ===
  // 1. NAVIGATION VALUES (ignore >50cm)
  long leftNav = leftRaw;
  long rightNav = rightRaw;
  if (leftNav > NAVIGATION_MAX_DISTANCE || leftNav == 999) leftNav = 999;
  if (rightNav > NAVIGATION_MAX_DISTANCE || rightNav == 999) rightNav = 999;
  if (leftNav <= 0) leftNav = 999;
  if (rightNav <= 0) rightNav = 999;

  // 2. STALL DETECTION VALUES (use up to 350cm)
  long leftStall = leftRaw;
  long rightStall = rightRaw;
  if (leftStall > STALL_MAX_DISTANCE && leftStall != 999) leftStall = STALL_MAX_DISTANCE;
  if (rightStall > STALL_MAX_DISTANCE && rightStall != 999) rightStall = STALL_MAX_DISTANCE;
  if (leftStall <= 0) leftStall = STALL_MAX_DISTANCE;
  if (rightStall <= 0) rightStall = STALL_MAX_DISTANCE;

  // === FORWARD STALL DETECTION ===
  if (!isBackingUp && !isPaused && missionActive && !inEscapeSequence) {
    checkForwardStall(leftStall, rightStall);
  }

  // === NAVIGATION DECISION TREE ===
  bool leftClose = (leftNav != 999 && leftNav <= CLOSE_OBSTACLE_DISTANCE);
  bool rightClose = (rightNav != 999 && rightNav <= CLOSE_OBSTACLE_DISTANCE);
  bool leftBlocked = (leftNav != 999 && leftNav < OBSTACLE_DISTANCE);
  bool rightBlocked = (rightNav != 999 && rightNav < OBSTACLE_DISTANCE);

  // Debug
  Serial.print("Nav[L:");
  if (leftNav == 999) Serial.print("CLEAR");
  else Serial.print(leftNav);
  Serial.print("cm R:");
  if (rightNav == 999) Serial.print("CLEAR");
  else Serial.print(rightNav);
  Serial.print("cm] -> ");

  // Decision tree
  if (leftClose && rightClose) {
    Serial.println("BOTH <=15cm - SMART BACKUP!");
    resetTurnPattern();
    isBackingUp = true;
    backupStartTime = millis();
    moveBackward();
    resetForwardStallDetection();
  }
  else if (leftClose) {
    Serial.println("LEFT <=15cm - START PAUSE (then turn right)");
    stopMotors();
    isPaused = true;
    pauseStartTime = millis();
    turnAfterPause = false;
    resetForwardStallDetection();
  }
  else if (rightClose) {
    Serial.println("RIGHT <=15cm - START PAUSE (then turn left)");
    stopMotors();
    isPaused = true;
    pauseStartTime = millis();
    turnAfterPause = true;
    resetForwardStallDetection();
  }
  else if (leftBlocked) {
    Serial.println("LEFT <25cm - TURN RIGHT");
    turnRightWithPattern();
    resetForwardStallDetection();
  }
  else if (rightBlocked) {
    Serial.println("RIGHT <25cm - TURN LEFT");
    turnLeftWithPattern();
    resetForwardStallDetection();
  }
  else {
    Serial.println("CLEAR - FORWARD");
    if (millis() - lastTurnTime > 2000) {
      resetTurnPattern();
    }
    moveForward();
  }

  delay(LOOP_INTERVAL);
}

// ==============================
// INTELLIGENT ESCAPE SEQUENCE
// ==============================
void executeStuckEscape() {
  // Prevent re-entrancy
  if (inEscapeSequence) return;
  inEscapeSequence = true;
  
  Serial.println("\n🔥 EXECUTING INTELLIGENT ESCAPE");
  
  // Force exit any other states
  isBackingUp = false;
  isPaused = false;
  
  // Step 1: Always backup first (2 seconds)
  Serial.println("Step 1: Backup (2 seconds)");
  moveBackward();
  delay(2000);
  stopMotors();
  delay(300);
  
  // Step 2: Check IR sensor AFTER backup
  bool irBlocked = (digitalRead(IR_REAR_SENSOR) == LOW);
  Serial.print("IR check after backup: ");
  Serial.println(irBlocked ? "BLOCKED" : "CLEAR");
  
  // Step 3: Take sensor readings
  long leftDist = readUltrasonic(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  long rightDist = readUltrasonic(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
  
  // Filter for decision making
  if (leftDist > 200 || leftDist == 999) leftDist = 999;
  if (rightDist > 200 || rightDist == 999) rightDist = 999;
  if (leftDist <= 0) leftDist = 999;
  if (rightDist <= 0) rightDist = 999;
  
  Serial.print("Sensor check: L=");
  if (leftDist == 999) Serial.print("CLEAR");
  else Serial.print(leftDist);
  Serial.print("cm R=");
  if (rightDist == 999) Serial.print("CLEAR");
  else Serial.print(rightDist);
  Serial.println("cm");
  
  // Step 4: Decision based on IR
  if (irBlocked) {
    // IR BLOCKED → NO 180° SPIN, just turn toward clearest side
    Serial.println("IR BLOCKED → Turning toward clearest side (NO SPIN)");
    
    // Determine which side is clearer
    bool leftClearer = (leftDist > rightDist);
    
    if (leftClearer) {
      // Left is clearer → Turn left (normal turn, not 180°)
      Serial.println("Left clearer → TURN LEFT");
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, 0);
      analogWrite(ENB, MOTOR_SPEED);
      delay(TURN_TIME);
      stopMotors();
    } else {
      // Right is clearer → Turn right (normal turn, not 180°)
      Serial.println("Right clearer → TURN RIGHT");
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      analogWrite(ENA, MOTOR_SPEED);
      analogWrite(ENB, 0);
      delay(TURN_TIME);
      stopMotors();
    }
    delay(300);
  } else {
    // IR CLEAR → Do 180° spin (random direction)
    Serial.println("IR CLEAR → Doing 180° RANDOM SPIN");
    
    randomSeed(analogRead(A5) + micros());
    bool spinRight = random(0, 2) == 0;
    
    if (spinRight) {
      Serial.println("180° RIGHT spin");
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      analogWrite(ENA, MOTOR_SPEED);
      analogWrite(ENB, MOTOR_SPEED);
      delay(1000);
      stopMotors();
    } else {
      Serial.println("180° LEFT spin");
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, MOTOR_SPEED);
      analogWrite(ENB, MOTOR_SPEED);
      delay(1000);
      stopMotors();
    }
    delay(300);
  }
  
  // Step 5: Forward burst (always)
  Serial.println("Step 3: Forward burst");
  moveForward();
  delay(1500);
  stopMotors();
  delay(500);
  
  // Reset all counters
  consecutiveBackups = 0;
  lastBackupTime = millis();
  resetForwardStallDetection();
  resetTurnPattern();
  
  // Clear escape flag
  inEscapeSequence = false;
  
  Serial.println("Escape complete!");
}

// ==============================
// TURN PATTERN DETECTION
// ==============================
void trackTurnPattern(bool isLeftTurn) {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTurnTime > PATTERN_RESET_TIME) {
    consecutiveLeftTurns = 0;
    consecutiveRightTurns = 0;
  }
  
  if (isLeftTurn) {
    consecutiveLeftTurns++;
    consecutiveRightTurns = 0;
    Serial.print("Left turn #");
    Serial.print(consecutiveLeftTurns);
    Serial.print(" of ");
    Serial.println(MAX_CONSECUTIVE_SAME_TURNS);
  } else {
    consecutiveRightTurns++;
    consecutiveLeftTurns = 0;
    Serial.print("Right turn #");
    Serial.print(consecutiveRightTurns);
    Serial.print(" of ");
    Serial.println(MAX_CONSECUTIVE_SAME_TURNS);
  }
  
  lastTurnTime = currentTime;
  
  if (consecutiveLeftTurns >= MAX_CONSECUTIVE_SAME_TURNS) {
    Serial.println("\n🔄 LEFT TURN PATTERN DETECTED! Correcting...");
    executeTurnPatternCorrection(true);
  } else if (consecutiveRightTurns >= MAX_CONSECUTIVE_SAME_TURNS) {
    Serial.println("\n🔄 RIGHT TURN PATTERN DETECTED! Correcting...");
    executeTurnPatternCorrection(false);
  }
}

void executeTurnPatternCorrection(bool wasLeftPattern) {
  Serial.println("Executing CURVED BACKUP correction...");
  
  stopMotors();
  isBackingUp = false;
  isPaused = false;
  delay(300);
  
  if (wasLeftPattern) {
    // Curve backup RIGHT
    Serial.println("CURVED BACKUP RIGHT (undoing left turns)");
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 60);
    analogWrite(ENB, 100);
  } else {
    // Curve backup LEFT
    Serial.println("CURVED BACKUP LEFT (undoing right turns)");
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 100);
    analogWrite(ENB, 60);
  }
  
  delay(1500);
  stopMotors();
  delay(300);
  
  Serial.println("Forward burst to complete correction");
  moveForward();
  delay(800);
  stopMotors();
  delay(300);
  
  consecutiveLeftTurns = 0;
  consecutiveRightTurns = 0;
  lastTurnTime = millis();
  
  Serial.println("Turn pattern correction complete!");
}

void resetTurnPattern() {
  consecutiveLeftTurns = 0;
  consecutiveRightTurns = 0;
  lastTurnTime = millis();
}

void turnLeftWithPattern() {
  trackTurnPattern(true);
  
  if (consecutiveLeftTurns < MAX_CONSECUTIVE_SAME_TURNS) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, MOTOR_SPEED);
    delay(TURN_TIME);
    stopMotors();
  }
}

void turnRightWithPattern() {
  trackTurnPattern(false);
  
  if (consecutiveRightTurns < MAX_CONSECUTIVE_SAME_TURNS) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, 0);
    delay(TURN_TIME);
    stopMotors();
  }
}

// ==============================
// FORWARD STALL DETECTION
// ==============================
void checkForwardStall(long currentLeft, long currentRight) {
  static unsigned long lastCheckTime = 0;
  static bool firstReading = true;
  
  if (millis() - lastCheckTime < 500) return;
  lastCheckTime = millis();
  
  if (firstReading) {
    lastLeftDistance = currentLeft;
    lastRightDistance = currentRight;
    lastMovementTime = millis();
    firstReading = false;
    return;
  }
  
  long leftChange = abs(currentLeft - lastLeftDistance);
  long rightChange = abs(currentRight - lastRightDistance);
  
  int effectiveThreshold = MIN_MOVEMENT_THRESHOLD;
  
  if (currentLeft == STALL_MAX_DISTANCE || currentRight == STALL_MAX_DISTANCE) {
    effectiveThreshold = 2;
  } else if (currentLeft > 100 || currentRight > 100) {
    effectiveThreshold = 3;
  }
  
  if (leftChange >= effectiveThreshold || rightChange >= effectiveThreshold) {
    lastMovementTime = millis();
    lastLeftDistance = currentLeft;
    lastRightDistance = currentRight;
    
    static int debugCounter = 0;
    if (debugCounter++ % 10 == 0) {
      Serial.print("[MOVING: LΔ=");
      Serial.print(leftChange);
      Serial.print("cm RΔ=");
      Serial.print(rightChange);
      Serial.print("cm Thresh=");
      Serial.print(effectiveThreshold);
      Serial.println("cm]");
    }
  }
  else if (millis() - lastMovementTime >= FORWARD_STALL_TIMEOUT) {
    Serial.println("\n🚨 FORWARD STALL DETECTED! No movement for 3s.");
    Serial.print("Last L=");
    if (lastLeftDistance == STALL_MAX_DISTANCE) Serial.print("350+cm");
    else Serial.print(lastLeftDistance);
    Serial.print(" R=");
    if (lastRightDistance == STALL_MAX_DISTANCE) Serial.print("350+cm");
    else Serial.print(lastRightDistance);
    Serial.print("cm | Current L=");
    if (currentLeft == STALL_MAX_DISTANCE) Serial.print("350+cm");
    else Serial.print(currentLeft);
    Serial.print("cm R=");
    if (currentRight == STALL_MAX_DISTANCE) Serial.print("350+cm");
    else Serial.print(currentRight);
    Serial.println("cm");
    
    executeStuckEscape();
    resetForwardStallDetection();
  }
}

void resetForwardStallDetection() {
  lastMovementTime = millis();
}

// ==============================
// SMART BACKUP HANDLING
// ==============================
void handleSmartBackupSequence() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastBackupTime > BACKUP_RESET_TIME) {
    consecutiveBackups = 0;
  }
  
  consecutiveBackups++;
  lastBackupTime = currentTime;
  
  Serial.print("Backup #");
  Serial.print(consecutiveBackups);
  Serial.print(" of ");
  Serial.print(MAX_CONSECUTIVE_BACKUPS);
  Serial.println();
  
  if (consecutiveBackups >= MAX_CONSECUTIVE_BACKUPS) {
    Serial.println("\n🚨 STUCK! Executing escape...");
    executeStuckEscape();
    return;
  }
  
  if (millis() - backupStartTime > BACKUP_SENSOR_CHECK_TIME) {
    long leftCheck = readUltrasonic(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
    long rightCheck = readUltrasonic(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
    
    if (leftCheck > NAVIGATION_MAX_DISTANCE || leftCheck == 999) leftCheck = 999;
    if (rightCheck > NAVIGATION_MAX_DISTANCE || rightCheck == 999) rightCheck = 999;
    
    Serial.print("Smart check: L=");
    if (leftCheck == 999) Serial.print("CLEAR");
    else Serial.print(leftCheck);
    Serial.print("cm R=");
    if (rightCheck == 999) Serial.print("CLEAR");
    else Serial.print(rightCheck);
    Serial.println("cm");
    
    if (leftCheck == 999 && rightCheck != 999) {
      Serial.println("SMART: Left side opened! Turning LEFT");
      isBackingUp = false;
      turnLeftWithPattern();
      resetForwardStallDetection();
      return;
    }
    else if (rightCheck == 999 && leftCheck != 999) {
      Serial.println("SMART: Right side opened! Turning RIGHT");
      isBackingUp = false;
      turnRightWithPattern();
      resetForwardStallDetection();
      return;
    }
    else if (leftCheck > CLEAR_THRESHOLD && rightCheck <= CLEAR_THRESHOLD) {
      Serial.println("SMART: Left is clearer, turning LEFT");
      isBackingUp = false;
      turnLeftWithPattern();
      resetForwardStallDetection();
      return;
    }
    else if (rightCheck > CLEAR_THRESHOLD && leftCheck <= CLEAR_THRESHOLD) {
      Serial.println("SMART: Right is clearer, turning RIGHT");
      isBackingUp = false;
      turnRightWithPattern();
      resetForwardStallDetection();
      return;
    }
  }
  
  if (millis() - backupStartTime >= BACKUP_DURATION) {
    Serial.println("BACKUP complete - Choosing SMART direction");
    
    stopMotors();
    delay(100);
    
    long leftFinal = readUltrasonic(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
    long rightFinal = readUltrasonic(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
    
    if (leftFinal > NAVIGATION_MAX_DISTANCE || leftFinal == 999) leftFinal = 999;
    if (rightFinal > NAVIGATION_MAX_DISTANCE || rightFinal == 999) rightFinal = 999;
    
    Serial.print("Final readings: L=");
    if (leftFinal == 999) Serial.print("CLEAR");
    else Serial.print(leftFinal);
    Serial.print("cm R=");
    if (rightFinal == 999) Serial.print("CLEAR");
    else Serial.print(rightFinal);
    Serial.println("cm");
    
    if (leftFinal == 999 && rightFinal == 999) {
      Serial.println("Both clear - default LEFT");
      turnLeftWithPattern();
    }
    else if (leftFinal == 999) {
      Serial.println("Only left clear - TURN LEFT");
      turnLeftWithPattern();
    }
    else if (rightFinal == 999) {
      Serial.println("Only right clear - TURN RIGHT");
      turnRightWithPattern();
    }
    else if (leftFinal >= rightFinal) {
      Serial.println("Left clearer - TURN LEFT");
      turnLeftWithPattern();
    }
    else {
      Serial.println("Right clearer - TURN RIGHT");
      turnRightWithPattern();
    }
    
    isBackingUp = false;
    resetForwardStallDetection();
    return;
  }
  
  if (digitalRead(IR_REAR_SENSOR) == LOW) {
    Serial.println("BACKUP: IR detected! Emergency turn");
    isBackingUp = false;
    consecutiveBackups++;
    moveForward();
    delay(300);
    stopMotors();
    turnLeftWithPattern();
    resetForwardStallDetection();
    return;
  }
  
  moveBackward();
}

// ==============================
// ULTRASONIC FUNCTION
// ==============================
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 45000);
  if (duration == 0) return 999;
  return duration / 58;
}

// ==============================
// MOTOR FUNCTIONS
// ==============================
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 85);
  analogWrite(ENB, 95);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 85);
  analogWrite(ENB, 95);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}