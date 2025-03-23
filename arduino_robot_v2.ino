#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/wdt.h>

#define BT_RX 2  
#define BT_TX 3  
SoftwareSerial BTSerial(BT_RX, BT_TX);

// Motor A
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;

// Motor B
const int ENB = 11;
const int IN3 = 8;
const int IN4 = 9;

#define BASE_SPEED 50  // Starting speed (slow start)
#define MAX_SPEED 120  // Max speed for heavy load
#define TURN_SPEED 80  // Speed when turning
int currentSpeed = BASE_SPEED;

// IR Sensors
const int IRSensorLeft = 12;
const int IRSensorRight = 10;

// Ultrasonic Sensor & Buzzer
#define TRIG_PIN A0
#define ECHO_PIN A1
#define BUZZER_PIN A2

// Servo Motor & IR Bin Sensor
Servo myServo;
#define SERVO_PIN 4
#define IR_BIN_SENSOR 13
unsigned long lastMotionTime = 0;

char mode = 'A';  // Default to Automatic Mode

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IRSensorLeft, INPUT);
  pinMode(IRSensorRight, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  myServo.attach(SERVO_PIN);
  pinMode(IR_BIN_SENSOR, INPUT);

  Serial.begin(9600);
  BTSerial.begin(9600);

  wdt_enable(WDTO_8S);

  // Close bin lid at startup
  myServo.write(0);
  delay(1000);
}

void loop() {
  wdt_reset();
  checkBluetooth();
  if (mode == 'A') {
    runAutomaticMode();
  }
}

// Check Bluetooth Commands
void checkBluetooth() {
  if (BTSerial.available()) {
    char command = BTSerial.read();
    while (BTSerial.available()) BTSerial.read();  

    if (command == 'A') {
      mode = 'A';
      Serial.println("🔄 Automatic Mode Activated");
    } else if (command == 'M') {
      mode = 'M';
      Serial.println("🎮 Manual Mode Activated");
      stopMotors();
    } else if (mode == 'M') {
      processManualCommand(command);
    }
  }
}

// Process Manual Commands
void processManualCommand(char command) {
  switch (command) {
    case 'F': accelerate(); break;
    case 'B': moveBackward(); break;
    case 'R': turnRight(); break;
    case 'L': turnLeft(); break;
    case 'S': stopMotors(); break;
    default: stopMotors(); break;
  }
}

// Automatic Mode (Line Following)
void runAutomaticMode() {
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 20000);
  distance = (duration == 0) ? 999 : (duration * 0.034) / 2;

  digitalWrite(BUZZER_PIN, (distance <= 20) ? HIGH : LOW);

  if (digitalRead(IR_BIN_SENSOR) == LOW) {
    myServo.write(120);
    lastMotionTime = millis();
  }
  if (millis() - lastMotionTime >= 5000) {
    myServo.write(0);
  }

  bool leftSensor = digitalRead(IRSensorLeft);
  bool rightSensor = digitalRead(IRSensorRight);

  if (leftSensor == HIGH && rightSensor == HIGH) {
    stopMotors();
  } else if (leftSensor == LOW && rightSensor == LOW) {
    accelerate();
  } else if (leftSensor == LOW) {
    turnRight();
  } else {
    turnLeft();
  }
}

// Smooth Acceleration Function
void accelerate() {
  while (currentSpeed < MAX_SPEED) {
    currentSpeed += 5;  // Increase speed gradually
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    delay(50);  // Small delay for smooth acceleration
  }
  moveForward();
}

// Move Forward at Current Speed
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, currentSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, currentSpeed);
}

// Move Backward
void moveBackward() {
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Turn Right
void turnRight() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Turn Left
void turnLeft() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Stop Motors Gradually
void stopMotors() {
  while (currentSpeed > BASE_SPEED) {
    currentSpeed -= 5;
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    delay(50);  // Small delay for smooth deceleration
  }
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
