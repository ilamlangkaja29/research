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
#define s 180
#define t 170

// Motor B
const int ENB = 11;
const int IN3 = 8;
const int IN4 = 9;

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

// Mode Selection: Automatic (A) or Manual (M)
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
}

void loop() {
  wdt_reset();  

  // Check for Bluetooth command
  if (BTSerial.available()) {
    char command = BTSerial.read();
    while (BTSerial.available()) BTSerial.read();  // Flush buffer

    if (command == 'A') {
      mode = 'A';  // Enable Automatic Mode
      Serial.println("🔄 Automatic Mode Activated (Line Following)");
    } else if (command == 'M') {
      mode = 'M';  // Enable Manual Mode
      Serial.println("🎮 Manual Mode Activated (Bluetooth Control)");
      stopMotors();  // Stop motors when switching to manual
    } else if (mode == 'M') {  
      processManualCommand(command);
    }
  }

  if (mode == 'A') {
    runAutomaticMode();
  }

  delay(100);
}

void processManualCommand(char command) {
  switch (command) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'R': turnRight(); break;
    case 'L': turnLeft(); break;
    case 'S': stopMotors(); break;
    default: stopMotors(); break;
  }
}

void runAutomaticMode() {
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 20000); 
  if (duration == 0) distance = 999;  
  else distance = (duration * 0.034) / 2;

  if (distance > 0 && distance <= 20) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  if (digitalRead(IR_BIN_SENSOR) == LOW) {
    myServo.write(120);
    lastMotionTime = millis();
  }
  if (millis() - lastMotionTime >= 5000) {
    myServo.write(0);
  }

  bool leftSensor = digitalRead(IRSensorLeft);
  bool rightSensor = digitalRead(IRSensorRight);

  Serial.print("Left Sensor: ");
  Serial.print(leftSensor);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightSensor);

  if (leftSensor == HIGH && rightSensor == HIGH) {
    Serial.println("⚠ Warning: Both IR sensors HIGH! Possible failure.");
    stopMotors();
  } else {
    if (leftSensor == LOW && rightSensor == LOW) {
      moveForward();
    } else if (leftSensor == LOW && rightSensor == HIGH) {
      turnRight();
    } else if (leftSensor == HIGH && rightSensor == LOW) {
      turnLeft();
    } else {
      stopMotors();
    }
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, s);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, s);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, s);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, s);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, t);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, t);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, t);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, t);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
