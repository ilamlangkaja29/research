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

// **Speed Configuration**
#define AUTO_FORWARD_SPEED 140  // Stable forward speed for line-following
#define AUTO_TURN_SPEED 120     // Reduced speed for smooth turns
int manualSpeed = 180;          // Default speed for manual mode

// IR Sensors
const int IRSensorLeft = 12;
const int IRSensorRight = 10;

// **Ultrasonic Sensor & Buzzer**
#define TRIG_FRONT A0   // Front Ultrasonic - Trigger
#define ECHO_FRONT A1   // Front Ultrasonic - Echo
#define BUZZER_PIN A2   // Buzzer

// **Bin Ultrasonic Sensor**
#define TRIG_BIN A3     // Bin Ultrasonic - Trigger
#define ECHO_BIN A4     // Bin Ultrasonic - Echo

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

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(TRIG_BIN, OUTPUT);
  pinMode(ECHO_BIN, INPUT);

  myServo.attach(SERVO_PIN);
  pinMode(IR_BIN_SENSOR, INPUT);

  Serial.begin(9600);
  BTSerial.begin(9600);

  wdt_enable(WDTO_8S);  

  // Ensure bin lid is closed on startup
  myServo.write(0);  
}

void loop() {
  wdt_reset();  

  // Check for Bluetooth command
  if (BTSerial.available()) {
    char command = BTSerial.read();
    while (BTSerial.available()) BTSerial.read();  // Flush buffer

    if (command == 'A') {
      mode = 'A';  
      Serial.println("🔄 Automatic Mode (Line-Following)");
    } else if (command == 'M') {
      mode = 'M';  
      Serial.println("🎮 Manual Mode (Bluetooth Control)");
      stopMotors();  
    } else if (mode == 'M') {  
      processManualCommand(command);
    }
  }

  if (mode == 'A') {
    runAutomaticMode();
  }

  checkObstacle();
  checkGarbage();

  delay(100);
}

void processManualCommand(char command) {
  switch (command) {
    case 'F': moveForward(manualSpeed); break;
    case 'B': moveBackward(manualSpeed); break;
    case 'R': turnRight(manualSpeed); break;
    case 'L': turnLeft(manualSpeed); break;
    case 'S': stopMotors(); break;

    // **New: Adjust Speed in Manual Mode**
    case '+': 
      manualSpeed = constrain(manualSpeed + 10, 100, 255);  
      Serial.print("Manual Speed Increased: ");
      Serial.println(manualSpeed);
      break;
    case '-': 
      manualSpeed = constrain(manualSpeed - 10, 100, 255);  
      Serial.print("Manual Speed Decreased: ");
      Serial.println(manualSpeed);
      break;

    default: stopMotors(); break;
  }
}

void runAutomaticMode() {
  bool leftSensor = digitalRead(IRSensorLeft);
  bool rightSensor = digitalRead(IRSensorRight);

  Serial.print("Left Sensor: ");
  Serial.print(leftSensor);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightSensor);

  // **Stable Line-Following Logic**
  if (leftSensor == HIGH && rightSensor == HIGH) {
    Serial.println("⚠ Both Sensors HIGH - Possible Error");
    stopMotors();
  } else {
    if (leftSensor == LOW && rightSensor == LOW) {
      moveForward(AUTO_FORWARD_SPEED);  // Smooth forward movement
    } else if (leftSensor == LOW && rightSensor == HIGH) {
      turnRight(AUTO_TURN_SPEED);  // Slightly slower right turn
    } else if (leftSensor == HIGH && rightSensor == LOW) {
      turnLeft(AUTO_TURN_SPEED);  // Slightly slower left turn
    } else {
      stopMotors();
    }
  }
}

// **Ultrasonic Obstacle Detection**
void checkObstacle() {
  long distance = getUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  
  if (distance > 0 && distance <= 20) {
    Serial.println("🚧 Obstacle detected! Stopping...");
    digitalWrite(BUZZER_PIN, HIGH);
    stopMotors();
    delay(500);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// **Ultrasonic Garbage Detection**
void checkGarbage() {
  long binDistance = getUltrasonicDistance(TRIG_BIN, ECHO_BIN);

  if (binDistance > 0 && binDistance <= 5) {
    Serial.println("🗑 Garbage Detected! Activating Auto Mode.");
    mode = 'A';  
  }
}

// **Ultrasonic Sensor Reading**
long getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);  
  if (duration == 0) return 999;  

  return (duration * 0.034) / 2;  
}

// **Updated Motor Control Functions with Speed Parameter**
void moveForward(int speedVal) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedVal);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedVal);
}

void moveBackward(int speedVal) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedVal);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedVal);
}

void turnRight(int speedVal) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedVal);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedVal);
}

void turnLeft(int speedVal) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedVal);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedVal);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
