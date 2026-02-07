/*
 * Mr. Beep-A-Lot - Your Friendly Arduino Companion
 * 
 * A multi-mode autonomous robot with personality that can:
 * - Navigate autonomously while avoiding obstacles
 * - Scan environment with radar visualization
 * - Follow humans (affectionately called "hoomans")
 * 
 * Hardware:
 * - Arduino UNO
 * - 2x HC-SR04 Ultrasonic Sensors (dual sensor setup)
 * - 2x Servo Motors (head movement + radar sweep)
 * - L298N Motor Driver (dual DC motor control)
 * - 16x2 I2C LCD Display (for personality expressions)
 * - IR Receiver (wireless remote control)
 * 
 * Software Integration:
 * - Communicates with Processing (Java) for real-time radar visualization
 * - Serial output for debugging and data transmission
 * 
 * @file beep_alot.ino
 * @author Shayan Mazahir
 * @date of last edit: February 2025
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include <IRremote.h>           // IR remote control functionality
#include <NewPing.h>            // Ultrasonic sensor library (better than default)
#include <Servo.h>              // Servo motor control
#include <Wire.h>               // I2C communication for LCD
#include <LiquidCrystal_I2C.h>  // I2C LCD display

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// IR Remote Control
const byte IR_RECEIVE_PIN = 10;  // IR receiver data pin

// Primary Ultrasonic Sensor (for obstacle avoidance & human following)
#define TRIG_PIN 8               // Trigger pin for main sensor
#define ECHO_PIN 9               // Echo pin for main sensor

// Radar Ultrasonic Sensor (for environment scanning)
#define RADAR_TRIG_PIN 12        // Trigger pin for radar sensor
#define RADAR_ECHO_PIN 13        // Echo pin for radar sensor

// Servo Motors
#define RADAR_SERVO_PIN 6        // Servo for radar sweep (0-90°)
#define SERVO_PIN 11             // Servo for obstacle avoidance head movement

// Motor Driver Pins (L298N)
#define IN1 2                    // Motor A direction control 1
#define IN2 3                    // Motor A direction control 2
#define IN3 4                    // Motor B direction control 1
#define IN4 5                    // Motor B direction control 2
#define ENA 6                    // Motor A speed control (PWM) - optional
#define ENB 7                    // Motor B speed control (PWM) - optional

// Constants
#define MAX_DISTANCE 200         // Maximum distance to detect (cm)
#define TURN_AMOUNT 500          // Duration for turns (ms) - not currently used

// ============================================================================
// HARDWARE OBJECT CREATION
// ============================================================================
Servo myservo;                   // Servo for looking left/right in obstacle avoidance
Servo radarServo;                // Servo for radar sweep mode
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);                      // Main ultrasonic sensor
NewPing radarSonar(RADAR_TRIG_PIN, RADAR_ECHO_PIN, MAX_DISTANCE);    // Radar ultrasonic sensor
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD with I2C address 0x27, 16 cols, 2 rows

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================
int distance = 0;                        // Current distance reading (cm)
boolean obstacleAvoidanceActive = false; // Flag for obstacle avoidance mode
boolean radarModeActive = false;         // Flag for radar scanning mode
boolean humanFollowingActive = false;    // Flag for human following mode

// ============================================================================
// SETUP - RUNS ONCE AT STARTUP
// ============================================================================
void setup() {
  // Initialize serial communication for debugging and Processing integration
  Serial.begin(9600);
  Serial.println("IR Controlled Robot");
  
  // Initialize IR receiver with LED feedback
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Initialize servo motors
  myservo.attach(SERVO_PIN);
  myservo.write(90);              // Center the obstacle avoidance servo
  radarServo.attach(RADAR_SERVO_PIN);
  radarServo.write(0);            // Set radar servo to starting position

  // Initialize LCD display
  lcd.init();                     // Initialize the LCD
  lcd.backlight();                // Turn on backlight
  lcd.clear();                    // Clear any previous content
  updateLCD("Please wait for", "10 seconds");
  delay(10000);                   // Startup delay to prevent abuse (prevents rapid mode switching)

  // Configure motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Display startup message with personality
  lcd.clear();
  updateLCD("MA LOOAARD!", "MR. BEEP-A-LOT");  // Startup greeting (because why not!)
  delay(2000);
}

// ============================================================================
// MAIN LOOP - RUNS CONTINUOUSLY
// ============================================================================
void loop() {
  // -------------------------------------------------------------------------
  // IR REMOTE CONTROL HANDLER
  // -------------------------------------------------------------------------
  if (IrReceiver.decode()) {
    int buttonCode = IrReceiver.decodedIRData.command;  // Get button code
    Serial.print("Button Code: ");
    Serial.println(buttonCode);

    // Ignore invalid button code (0 = no valid signal)
    if (buttonCode == 0) {
      IrReceiver.resume();        // Get ready for next signal
      return;                     // Skip rest of processing
    }

    // -----------------------------------------------------------------------
    // MODE SELECTION VIA IR REMOTE
    // -----------------------------------------------------------------------
    
    if (buttonCode == 69) {       // Button 1: Obstacle Avoidance Mode
      Serial.println("Obstacle Avoidance Mode Activated");
      obstacleAvoidanceActive = true;
      radarModeActive = false;
      humanFollowingActive = false;
      updateLCD("Obstacle", "Avoidance");
      delay(5000);
      resetRadarServo();          // Return radar servo to default
      radarServo.detach();        // Disable radar servo to prevent jitter
      
    } else if (buttonCode == 70) { // Button 2: Radar Scanning Mode
      Serial.println("Radar Mode Activated");
      moveStop();                 // Stop all movement
      radarServo.attach(RADAR_SERVO_PIN);  // Re-enable radar servo
      obstacleAvoidanceActive = false;
      radarModeActive = true;
      humanFollowingActive = false;
      updateLCD("Radar", "Mode");
      resetRadarServo();          // Reset servo to start position
      
    } else if (buttonCode == 71) { // Button 3: Human Following Mode
      Serial.println("Human Following Mode Activated");
      radarServo.write(0);        // Set radar servo to 0°
      obstacleAvoidanceActive = false;
      radarModeActive = false;
      humanFollowingActive = true;
      updateLCD("Human", "Following");
      delay(5000);
      resetRadarServo();          // Reset servo to default
      
    } else {                      // Any other button: Idle Mode
      Serial.println("Idle Mode Activated");
      moveStop();                 // Stop all motors
      radarServo.detach();        // Disable radar servo
      obstacleAvoidanceActive = false;
      radarModeActive = false;
      humanFollowingActive = false;
      updateLCD("I'M A MINOR!", "(Idle Mode)");  // Easter egg for undefined buttons :)
      delay(5000);
      resetRadarServo();
      radarServo.detach();
    }

    IrReceiver.resume();          // Prepare for next IR command
  }

  // -------------------------------------------------------------------------
  // MODE EXECUTION
  // -------------------------------------------------------------------------
  if (obstacleAvoidanceActive) {
    runObstacleAvoidance();       // Execute obstacle avoidance behavior
  } else if (radarModeActive) {
    runRadarMode();               // Execute radar scanning
    moveStop();                   // Keep robot stationary during radar scan
  } else if (humanFollowingActive) {
    runHumanFollowing();          // Execute human following behavior
  }
}

// ============================================================================
// MODE 1: OBSTACLE AVOIDANCE
// ============================================================================
/**
 * Autonomous navigation that avoids obstacles
 * 
 * Algorithm:
 * 1. Check distance ahead
 * 2. If obstacle detected (<15cm-ish):
 *    a. Stop and back up
 *    b. Look left and right with servo
 *    c. Turn toward more open space
 * 3. If path clear: move forward
 */
void runObstacleAvoidance() {
  int distanceR = 0;              // Distance to right side
  int distanceL = 0;              // Distance to left side
  delay(40);                      // Sensor stabilization delay
  distance = readPing();          // Measure distance ahead
  Serial.println(distance);

  if (distance <= 15) {           // Obstacle detected within 15cm
    Serial.println("Object Detected");
    
    // Emergency stop and backup sequence
    moveStop();
    delay(100);
    moveBackward();               // Back away from obstacle
    delay(300);
    moveStop();
    delay(200);

    // Scan left and right to find open path
    distanceR = lookRight();      // Measure distance to right
    Serial.print("Distance Right = ");
    Serial.println(distanceR);
    delay(200);
    
    distanceL = lookLeft();       // Measure distance to left
    Serial.print("Distance Left = ");
    Serial.println(distanceL);
    delay(200);

    // Turn toward the more open direction
    if (distanceR >= distanceL) {
      turnRight();                // Right has more space
      moveStop();
    } else {
      turnLeft();                 // Left has more space
      moveStop();
    }
  } else {
    delay(100);
    moveForward();                // Path is clear, continue forward
  }
}

// ============================================================================
// MODE 2: RADAR SCANNING
// ============================================================================
/**
 * Environment scanning mode with Processing visualization
 * 
 * Sweeps radar servo from 0° to 90° and back, measuring distance
 * at each angle. Sends data via Serial to Processing for real-time
 * radar display visualization.
 * 
 * Serial Output Format: "angle,distance\n"
 * Example: "45,120\n" means 45° angle, 120cm distance
 */
void runRadarMode() {
  moveStop();                     // Robot must be stationary for accurate scanning
  static int angle = 0;           // Current servo angle (persists between calls)
  static int step = 5;            // Angle increment (5° steps)

  // Move servo to current angle
  radarServo.write(angle);
  delay(50);                      // Allow servo time to reach position

  // Measure distance at current angle
  int radarDistance = radarSonar.ping_cm();
  if (radarDistance == 0) {       // 0 means no echo received
    radarDistance = 200;          // Assume max range if no object detected
  }
  
  // Debug output to Serial Monitor
  Serial.print("Radar Angle: ");
  Serial.print(angle);
  Serial.print(" Distance: ");
  Serial.println(radarDistance);

  // Send formatted data to Processing for visualization
  Serial.print(angle);
  Serial.print(",");              // CSV format: angle,distance
  Serial.println(radarDistance);

  // Update angle for next sweep
  angle += step;
  if (angle >= 90 || angle <= 0) {
    step = -step;                 // Reverse direction at boundaries (ping-pong sweep)
  }
}

// ============================================================================
// MODE 3: HUMAN FOLLOWING
// ============================================================================
/**
 * Follows detected person with personality-driven LCD messages
 * 
 * Distance Zones:
 * - <15cm: Too close! "Hooman too.. close" - STOP
 * - 15-100cm: Perfect! "POOKIE HOOMAN!!" - FOLLOW
 * - >100cm: Too far! "Hooman? Where?" - STOP
 */
void runHumanFollowing() {
  delay(100);                     // Stability delay
  distance = readPing();          // Measure distance to person
  Serial.print("Distance: ");
  Serial.println(distance);

  // Person is too close - maintain personal space
  if (distance < 15) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hooman too..");
    lcd.setCursor(0, 1);
    lcd.print("close");
    moveStop();                   // Stop - respect the personal space bubble!
  }
  // Person is at ideal following distance
  else if (distance >= 15 && distance <= 100) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("POOKIE HOOMAN!!");  // Affectionate following mode engaged
    moveForward();                 // Follow the hooman!
  }
  // Person is too far away or not detected
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hooman? Where?");   // Lost sight of person
    moveStop();                    // Stop and wait
  }
}

// ============================================================================
// SERVO HELPER FUNCTIONS
// ============================================================================

/**
 * Reset radar servo to default position (0°)
 */
void resetRadarServo() {
  radarServo.write(0);            // Return to starting position
  delay(500);                     // Wait for servo to reach position
}

/**
 * Look right by rotating obstacle avoidance servo
 * @return Distance measured on right side (cm)
 */
int lookRight() {
  myservo.write(0);               // Rotate servo to right (0°)
  delay(500);                     // Wait for servo to reach position
  int distance = sonar.ping_cm(); // Measure distance
  delay(100);
  myservo.write(90);              // Return servo to center
  return distance;
}

/**
 * Look left by rotating obstacle avoidance servo
 * @return Distance measured on left side (cm)
 */
int lookLeft() {
  myservo.write(180);             // Rotate servo to left (180°)
  delay(500);                     // Wait for servo to reach position
  int distance = sonar.ping_cm(); // Measure distance
  delay(100);
  myservo.write(90);              // Return servo to center
  return distance;
}

/**
 * Read distance from main ultrasonic sensor
 * @return Distance in cm (250 if no object detected)
 */
int readPing() {
  delay(70);                      // Sensor stabilization delay
  int cm = sonar.ping_cm();       // Measure distance
  if (cm == 0) {                  // 0 = no echo received
    cm = 250;                     // Assume max range
  }
  return cm;
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * Stop all motor movement
 */
void moveStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Stopping");
}

/**
 * Move backward
 * Both motors reversed
 */
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Backward");
}

/**
 * Turn right
 * Left motor forward, right motor backward
 */
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Right");
}

/**
 * Turn left
 * Right motor forward, left motor backward
 */
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning Left");
}

/**
 * Move forward
 * Both motors forward
 */
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving Forward");
}

// ============================================================================
// LCD DISPLAY HELPER
// ============================================================================

/**
 * Update LCD display with two lines of text
 * @param line1 Text for first row (16 chars max)
 * @param line2 Text for second row (16 chars max)
 */
void updateLCD(String line1, String line2) {
  lcd.clear();                    // Clear previous content
  lcd.setCursor(0, 0);            // Move cursor to first row
  lcd.print(line1);               // Display first line
  lcd.setCursor(0, 1);            // Move cursor to second row
  lcd.print(line2);               // Display second line
}