const int trigPin = 9;
const int echoPin = 10;
const int irPin = A0;

//pin in
const int motor1Pin1 = 2;
const int motor1Pin2 = 3;
const int motor2Pin1 = 4;
const int motor2Pin2 = 5;
const int enableA = 6;
const int enableB = 7;

//motor speed
const int motorSpeed = 200;

void setup() {
  Serial.begin(9600);

  //pins out
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);

  //sonar pin
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //IR pin
  pinMode(irPin, INPUT);
  
  analogWrite(enableA, motorSpeed);
  analogWrite(enableB, motorSpeed);
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;


  int irValue = analogRead(irPin);


  if (distance < 20 || irValue < 500) {
    stopRobot();

    turnRight();
    delay(500);
  } else {

    moveForward();
  }

  delay(100);
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void stopRobot() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void turnRight() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}
