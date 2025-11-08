#include <Arduino.h>
#line 1 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
#include <Servo.h>
#define trigPin 8
#define echoPin 11
#define ENA 9   // PWM pin for motor A
#define ENB 10  // PWM pin for motor B
#define IN1 7
#define IN2 6
#define IN3 5
#define IN4 4
#define LED_PIN 12       // LED for obstacle detected
#define BUZZER_PIN 13    // Buzzer for alerts
#define BATTERY_PIN A5   // Analog pin for battery monitoring (using voltage divider)

Servo ultrasonicServo;

long duration;
int distance;
int servoAngle = 90;
int motorSpeed = 180;        // Max motor speed (0–255)
int currentMotorSpeed = 0;   // Current speed for ramping
int targetMotorSpeed = 0;    // Target speed for ramping
int robotState = 0;          // 0 = moving forward, 1 = avoiding obstacle
bool isMoving = false;
unsigned long lastMovementTime = 0;
unsigned long stuckCheckTime = 0;  // Stuck detection timer
int consecutiveStuckCount = 0;     // Counter for stuck events

// --- Battery Monitoring Variables ---
int batteryLevel = 0;
int lastBatteryWarning = 0;        // Time of last battery warning
const unsigned char BATTERY_CHECK_INTERVAL = 50; // Check battery every 50 loops
unsigned char batteryCheckCounter = 0;
const int LOW_BATTERY_THRESHOLD = 350;  // ADC value (~3.0V for 5V with divider)
bool batteryLow = false;
unsigned long totalEnergy = 0;     // Track total energy used

// --- Movement Tracking Variables ---
unsigned long totalMovementTime = 0;  // Total time moving forward
unsigned long totalAvoidanceTime = 0; // Total time avoiding obstacles
int totalTurns = 0;                    // Count of turns made
int successfulMoves = 0;               // Successful forward movements
int failedAvoidances = 0;              // Times stuck detection triggered
unsigned long sessionStartTime = 0;

// --- Adjustable timing constants ---
const int SAFE_DISTANCE = 30;     // cm
const int BACKWARD_TIME = 600;    // ms (RESTORED)
const int STOP_DELAY = 300;       // ms (RESTORED)
const int TURN_DELAY = 700;       // ms (RESTORED)
const int SERVO_LOOK_DELAY = 300; // ms (RESTORED - servo needs time to settle!)
const int SENSOR_SAMPLE_DELAY = 5; // ms (RESTORED - better accuracy)
const int SENSOR_SAMPLES = 3;     // Keep at 3 samples

// --- Debug mode ---
const bool DEBUG_MODE = true;

#line 57 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void setup();
#line 89 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void loop();
#line 143 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
int measureDistance();
#line 161 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
int lookRight();
#line 169 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
int lookLeft();
#line 178 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void moveForward();
#line 187 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void moveForwardSmooth();
#line 196 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void moveBackward();
#line 205 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void turnLeft();
#line 214 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void turnRight();
#line 223 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void stopCar();
#line 233 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void beep(int duration);
#line 240 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void checkBatteryLevel();
#line 282 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void printRobotStats();
#line 57 "C:\\Users\\SRHS\\Downloads\\obstable_code\\obstable_code.ino"
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);  // Battery monitoring pin

  Serial.begin(9600);
  ultrasonicServo.attach(3);
  ultrasonicServo.write(servoAngle); // face forward
  
  // Initialize session tracking
  sessionStartTime = millis();
  stuckCheckTime = millis();
  
  if (DEBUG_MODE) {
    Serial.println("=== OBSTACLE AVOIDING ROBOT INITIALIZED ===");
    Serial.println("System ready!");
    Serial.println("Battery monitoring: ACTIVE");
    Serial.println("Movement tracking: ACTIVE");
  }
  
  // Startup beep
  beep(100);
}

void loop() {
  distance = measureDistance();

  // --- Battery Monitoring (check every 50 loops to save power) ---
  batteryCheckCounter++;
  if (batteryCheckCounter >= BATTERY_CHECK_INTERVAL) {
    checkBatteryLevel();
    batteryCheckCounter = 0;
  }

  if (DEBUG_MODE) {
    Serial.print("Distance: ");
    Serial.println(distance);
  }

  if (distance > 0 && distance < SAFE_DISTANCE) {
    stopCar();
    delay(STOP_DELAY);

    // Move backward briefly
    moveBackward();
    delay(BACKWARD_TIME);
    stopCar();
    delay(STOP_DELAY);

    // Look left and right quickly
    int distanceLeft = lookLeft();
    int distanceRight = lookRight();
    ultrasonicServo.write(90); // reset forward

    // Choose best direction
    if (distanceLeft > distanceRight) {
      turnLeft();
      totalTurns++;
    } else {
      turnRight();
      totalTurns++;
    }
    delay(TURN_DELAY);
    failedAvoidances++;
  } else {
    moveForward();
    totalMovementTime += 50;  // Approximate 50ms per loop cycle
  }

  // Periodic stats report (less frequently to avoid Serial bottleneck)
  if (millis() % 10000 < 50) {  // Every ~10 seconds
    if (DEBUG_MODE) {
      printRobotStats();
    }
  }
}

// --- Ultrasonic measurement ---
int measureDistance() {
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 20000); // timeout in 20ms
    int measuredDistance = duration * 0.034 / 2;
    sum += measuredDistance;
    delay(SENSOR_SAMPLE_DELAY);
  }
  return sum / SENSOR_SAMPLES;
}

// --- Servo scanning ---
int lookRight() {
  ultrasonicServo.write(0);
  delay(SERVO_LOOK_DELAY);
  int distance = measureDistance();
  delay(100);
  return distance;
}

int lookLeft() {
  ultrasonicServo.write(180);
  delay(SERVO_LOOK_DELAY);
  int distance = measureDistance();
  delay(100);
  return distance;
}

// --- Motion control ---
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveForwardSmooth() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// --- Beep function for audio feedback ---
void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

// --- Battery Monitoring Function ---
void checkBatteryLevel() {
  batteryLevel = analogRead(BATTERY_PIN);
  
  if (batteryLevel < LOW_BATTERY_THRESHOLD) {
    if (!batteryLow) {
      batteryLow = true;
      if (DEBUG_MODE) Serial.println("⚠️  LOW BATTERY WARNING! ⚠️");
      
      // Alert with beeps
      beep(200);
      delay(100);
      beep(200);
      delay(100);
      beep(200);
      
      // Turn on LED for continuous warning
      digitalWrite(LED_PIN, HIGH);
    }
    lastBatteryWarning = millis();
  } else if (batteryLow && batteryLevel > (LOW_BATTERY_THRESHOLD + 50)) {
    // Battery recovered above threshold + 50 ADC buffer
    batteryLow = false;
    digitalWrite(LED_PIN, LOW);
    if (DEBUG_MODE) Serial.println("✓ Battery level recovered!");
  }
  
  // Calculate estimated battery percentage (adjust for your setup)
  // For 5V with divider: 1024 = 10V, 512 = 5V, 350 = 3.4V (low)
  int batteryPercent = map(batteryLevel, 350, 1024, 0, 100);
  if (batteryPercent < 0) batteryPercent = 0;
  if (batteryPercent > 100) batteryPercent = 100;
  
  if (DEBUG_MODE && millis() % 3000 < 50) {  // Every 3 seconds
    Serial.print("🔋 Battery ADC: ");
    Serial.print(batteryLevel);
    Serial.print(" | Estimated: ");
    Serial.print(batteryPercent);
    Serial.println("%");
  }
}

// --- Print Robot Statistics ---
void printRobotStats() {
  unsigned long sessionDuration = (millis() - sessionStartTime) / 1000;  // In seconds
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      ROBOT SESSION STATISTICS          ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  Serial.print("║ Session Duration: ");
  Serial.print(sessionDuration);
  Serial.println("s                ║");
  
  Serial.print("║ Total Movement Time: ");
  Serial.print(totalMovementTime / 1000);
  Serial.println("s             ║");
  
  Serial.print("║ Total Avoidance Time: ");
  Serial.print(totalAvoidanceTime / 1000);
  Serial.println("s            ║");
  
  Serial.print("║ Successful Moves: ");
  Serial.print(successfulMoves);
  Serial.println("                 ║");
  
  Serial.print("║ Failed Avoidances: ");

  Serial.print(failedAvoidances);
  Serial.println("                ║");
  
  Serial.print("║ Battery Level: ");
  Serial.print(batteryLevel);
  Serial.println(" ADC                   ║");
  
  Serial.print("║ Current State: ");
  if (robotState == 0) {
    Serial.println("MOVING FORWARD       ║");
  } else {
    Serial.println("AVOIDING OBSTACLE    ║");
  }
  
  Serial.print("║ Stuck Attempts: ");
  Serial.print(consecutiveStuckCount);
  Serial.println("                   ║");
  
  Serial.println("╚════════════════════════════════════════╝\n");
}
