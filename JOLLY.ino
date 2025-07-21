// Pin definitions for the Ultrasonic Sensor (HC-SR04)
const int trigPin = 9;  // Trigger pin connected to Arduino Digital Pin 9
const int echoPin = 10; // Echo pin connected to Arduino Digital Pin 10

// Pin definition for the original IR Obstacle Sensor
const int irObstaclePin = A0; // IR Obstacle Sensor connected to Arduino Analog Pin A0

// Pin definitions for the Motor Driver (L298N)
// Motor 1 (Left Motor)
const int motor1Pin1 = 2; // IN1 of L298N connected to Arduino Digital Pin 2
const int motor1Pin2 = 3; // IN2 of L298N connected to Arduino Digital Pin 3
const int enableA = 6;    // ENA of L298N connected to Arduino Digital Pin 6 (for PWM speed control)

// Motor 2 (Right Motor)
const int motor2Pin1 = 4; // IN3 of L298N connected to Arduino Digital Pin 4
const int motor2Pin2 = 5; // IN4 of L298N connected to Arduino Digital Pin 5
const int enableB = 7;    // ENB of L298N connected to Arduino Digital Pin 7 (for PWM speed control)

// Pin definitions for the new IR Line Follower Sensors
// Assuming analog line sensors (e.g., TCRT5000 modules)
const int irLineLeftPin = A1;   // Left Line Sensor connected to Arduino Analog Pin A1
const int irLineCenterPin = A2; // Center Line Sensor connected to Arduino Analog Pin A2
const int irLineRightPin = A3;  // Right Line Sensor connected to Arduino Analog Pin A3

// Motor speed setting (0-255, 255 is max speed)
const int motorSpeed = 200;

// Threshold for line detection (adjust based on your sensors and line/surface color)
// A lower value typically means detecting a darker surface (e.g., black line on white floor)
// You might need to calibrate this value by reading sensor output on and off the line.
const int lineDetectionThreshold = 500; // Example: if analogRead < 500, it's on the line

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);

  // Set motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enableA, OUTPUT); // Enable pin for Motor 1 speed control
  pinMode(enableB, OUTPUT); // Enable pin for Motor 2 speed control

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT); // Trigger pin sends out a pulse
  pinMode(echoPin, INPUT);  // Echo pin receives the reflected pulse

  // Set IR obstacle sensor pin as input
  pinMode(irObstaclePin, INPUT);

  // Line follower sensor pins are analog inputs, no pinMode needed for analogRead

  // Set initial motor speeds (PWM)
  analogWrite(enableA, motorSpeed); // Set speed for Motor 1
  analogWrite(enableB, motorSpeed); // Set speed for Motor 2
}

void loop() {
  // --- Obstacle Avoidance Logic (Higher Priority) ---

  // Measure distance using the ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  // Calculate distance in cm (speed of sound is approx 0.034 cm/microsecond)
  distance = duration * 0.034 / 2;

  // Read value from the IR obstacle sensor
  int irObstacleValue = analogRead(irObstaclePin);

  // Check for obstacles: if distance is less than 20cm OR IR sensor detects something close
  // (Assuming IR sensor value drops below 500 when an obstacle is near)
  if (distance < 20 || irObstacleValue < 500) {
    Serial.println("Obstacle detected! Avoiding...");
    stopRobot(); // Stop the robot immediately
    delay(200);  // Brief pause

    // Turn right to avoid the obstacle
    turnRight();
    delay(500); // Allow time for the turn to complete
    stopRobot(); // Stop after turning
    delay(200); // Brief pause before continuing
  }
  // --- Line Following Logic (Lower Priority, only if no obstacles) ---
  else {
    // Read values from the line follower sensors
    int irLineLeftValue = analogRead(irLineLeftPin);
    int irLineCenterValue = analogRead(irLineCenterPin);
    int irLineRightValue = analogRead(irLineRightPin);

    // Determine if each sensor is on the line based on the threshold
    bool onLineLeft = irLineLeftValue < lineDetectionThreshold;
    bool onLineCenter = irLineCenterValue < lineDetectionThreshold;
    bool onLineRight = irLineRightValue < lineDetectionThreshold;

    // Debugging output for line sensors (uncomment to see values in Serial Monitor)
    // Serial.print("L: "); Serial.print(irLineLeftValue);
    // Serial.print(" C: "); Serial.print(irLineCenterValue);
    // Serial.print(" R: "); Serial.print(irLineRightValue);
    // Serial.print(" | OL: "); Serial.print(onLineLeft);
    // Serial.print(" OC: "); Serial.print(onLineCenter);
    // Serial.print(" OR: "); Serial.println(onLineRight);

    // Line following logic
    if (onLineCenter && !onLineLeft && !onLineRight) {
      // Robot is perfectly on the line, move straight
      Serial.println("On line: Straight");
      moveForward();
    } else if (onLineLeft && !onLineCenter) {
      // Robot drifted right, line is under left sensor, turn left
      Serial.println("Drifted right: Turn Left");
      turnLeft();
    } else if (onLineRight && !onLineCenter) {
      // Robot drifted left, line is under right sensor, turn right
      Serial.println("Drifted left: Turn Right");
      turnRight();
    } else if (onLineCenter && onLineLeft && !onLineRight) {
      // Line is slightly to the left, but still under center. Slight adjustment needed.
      // For simplicity, we'll still turn left slightly, or just maintain forward.
      // A more advanced robot might use proportional control here.
      Serial.println("Slightly left: Turn Left");
      turnLeft();
    } else if (onLineCenter && onLineRight && !onLineLeft) {
      // Line is slightly to the right, but still under center. Slight adjustment needed.
      Serial.println("Slightly right: Turn Right");
      turnRight();
    } else if (!onLineLeft && !onLineCenter && !onLineRight) {
      // Robot has lost the line (all sensors off the line)
      Serial.println("Lost line: Stop and search");
      stopRobot();
      // Optional: Add logic to search for the line, e.g., slowly turn in one direction
      // turnLeft();
      // delay(100);
    } else {
      // Any other combination (e.g., all on line, or complex edge cases)
      // Default to moving forward or stopping.
      Serial.println("Complex state: Move Forward");
      moveForward();
    }
  }

  // Small delay to stabilize sensor readings and motor actions
  delay(100);
}

// Function to make the robot move forward
void moveForward() {
  digitalWrite(motor1Pin1, HIGH); // Left motor forward
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH); // Right motor forward
  digitalWrite(motor2Pin2, LOW);
}

// Function to stop the robot
void stopRobot() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Function to make the robot pivot right
// Left motor moves backward, Right motor moves forward
void turnRight() {
  digitalWrite(motor1Pin1, LOW);  // Left motor backward
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH); // Right motor forward
  digitalWrite(motor2Pin2, LOW);
}

// Function to make the robot pivot left (newly added)
// Left motor moves forward, Right motor moves backward
void turnLeft() {
  digitalWrite(motor1Pin1, HIGH); // Left motor forward
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);  // Right motor backward
  digitalWrite(motor2Pin2, HIGH);
}
