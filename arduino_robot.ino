#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"

// Motor Driver Pins
#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 10
#define IN3 8
#define IN4 9

// Bluetooth Module
#define BT_RX 2  // SoftwareSerial (if needed)
#define BT_TX 3

// IR Sensors
#define IR_MOTOR 4
#define IR_BIN 11

// Servo Motor
Servo binServo;
#define SERVO_PIN 12

// Ultrasonic Sensor
#define TRIG 13
#define ECHO A0

// Buzzer
#define BUZZER A1

// Load Cell (HX711)
#define LOADCELL_DOUT_PIN A2
#define LOADCELL_SCK_PIN A3
HX711 scale;

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Serial.begin(9600);
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(IR_MOTOR, INPUT); pinMode(IR_BIN, INPUT);
    pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
    pinMode(BUZZER, OUTPUT);
    
    binServo.attach(SERVO_PIN);
    binServo.write(0);
    
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Weight: 0g");
}

void loop() {
    bluetoothControl();
    IR_SensorControl();
    ultrasonicAvoidance();
    readWeight();
}

// Bluetooth Control for Motors
void bluetoothControl() {
    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'F') moveForward();
        else if (command == 'B') moveBackward();
        else if (command == 'L') turnLeft();
        else if (command == 'R') turnRight();
        else if (command == 'S') stopMotors();
    }
}

// IR Sensor Control
void IR_SensorControl() {
    if (digitalRead(IR_MOTOR) == LOW) stopMotors();
    if (digitalRead(IR_BIN) == LOW) openBinCover();
}

// Open Bin Cover
void openBinCover() {
    binServo.write(90);
    delay(3000);
    binServo.write(0);
}

// Ultrasonic Sensor Avoidance
void ultrasonicAvoidance() {
    long duration;
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    duration = pulseIn(ECHO, HIGH);
    int distance = duration * 0.034 / 2;
    if (distance < 15) {
        stopMotors();
        digitalWrite(BUZZER, HIGH);
        delay(1000);
        digitalWrite(BUZZER, LOW);
    }
}

// Read Weight from Load Cell
void readWeight() {
    float weight = scale.get_units();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Weight: ");
    lcd.print(weight, 1);
    lcd.print("g");
}

// Motor Functions
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
