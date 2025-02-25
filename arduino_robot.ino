#include <Servo.h>
#include <NewPing.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin Configuration
#define IR_SENSOR 4            // Line Following IR sensor
#define TRIG_PIN A0            // Ultrasonic sensor trigger
#define ECHO_PIN A0            // Ultrasonic sensor echo
#define BIN_SENSOR 11          // E18-D80NK IR sensor for bin detection
#define SERVO_PIN 12           // Servo motor for bin lid
#define BUZZER A1              // Buzzer for alerts

// Motor Driver (L298N)
#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10

// Bluetooth module (HC-05)
#define BT_RX 0
#define BT_TX 1

// Load Cell (HX711)
#define LOADCELL_DOUT 2
#define LOADCELL_SCK 3

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Create LCD object with I2C address 0x27

// Sensors and Components
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);  // Ultrasonic sensor
Servo binServo;
HX711 scale;

// Variables
char command;   // Bluetooth command
long distance;  // Ultrasonic distance

void setup() {
    Serial.begin(9600); // Bluetooth communication
    
    // Motor Pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Sensor Pins
    pinMode(IR_SENSOR, INPUT);
    pinMode(BIN_SENSOR, INPUT);
    pinMode(BUZZER, OUTPUT);

    // Load Cell
    scale.begin(LOADCELL_DOUT, LOADCELL_SCK);

    // Servo
    binServo.attach(SERVO_PIN);
    binServo.write(0); // Initial position

    // LCD Display
    lcd.begin(16, 2);  // Initialize LCD with 16 columns and 2 rows
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Robot Initialized");
}

void loop() {
    // Check Ultrasonic Distance
    distance = sonar.ping_cm();
    if (distance > 0 && distance < 10) {
        stopMotors();
        digitalWrite(BUZZER, HIGH);
        delay(1000);
        digitalWrite(BUZZER, LOW);
    }

    // Line Following (Automatic Mode)
    if (digitalRead(IR_SENSOR) == LOW) {
        moveForward();
    } else {
        stopMotors();
    }

    // Bin Detection & Servo Control
    if (digitalRead(BIN_SENSOR) == LOW) {
        binServo.write(90); // Open lid
        delay(2000);
        binServo.write(0);  // Close lid
    }

    // Bluetooth Control
    if (Serial.available()) {
        command = Serial.read();
        executeBluetoothCommand(command);
    }

    // Load Cell Measurement
    float weight = scale.get_units(10);
    lcd.setCursor(0, 1);
    lcd.print("Weight: ");
    lcd.print(weight);
    lcd.print("g");

    delay(100);
}

// Motor Control Functions
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
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Bluetooth Command Execution
void executeBluetoothCommand(char cmd) {
    switch (cmd) {
        case 'F': moveForward(); break;
        case 'B': moveBackward(); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        case 'S': stopMotors(); break;
        default: break;
    }
}
