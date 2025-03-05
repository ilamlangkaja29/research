#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Bluetooth module
SoftwareSerial BTSerial(10, 11); // RX, TX
char receivedChar;

// Motor Driver (L298N)
#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 12

// LCD Display (16x2, I2C address may vary, common: 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("BT & Motor Test");

    // Initialize motor driver pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    Serial.println("System Initialized");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
}

void loop() {
    // Bluetooth data reception
    if (BTSerial.available()) {
        receivedChar = BTSerial.read();
        Serial.print("Received: ");
        Serial.println(receivedChar);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BT: ");
        lcd.print(receivedChar);
        handleMotor(receivedChar);
    }

    // Send data from Serial Monitor to Bluetooth
    if (Serial.available()) {
        receivedChar = Serial.read();
        BTSerial.print(receivedChar);
        Serial.print("Sent: ");
        Serial.println(receivedChar);
        handleMotor(receivedChar);
    }
}

void handleMotor(char command) {
    switch (command) {
        case 'F': // Forward
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.setCursor(0, 1);
            lcd.print("Moving Forward");
            break;
        case 'B': // Backward
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.setCursor(0, 1);
            lcd.print("Moving Backward");
            break;
        case 'L': // Left
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.setCursor(0, 1);
            lcd.print("Turning Left");
            break;
        case 'R': // Right
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.setCursor(0, 1);
            lcd.print("Turning Right");
            break;
        case 'S': // Stop
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 0);
            analogWrite(ENB, 0);
            lcd.setCursor(0, 1);
            lcd.print("Stopped   ");
            break;
        default:
            lcd.setCursor(0, 1);
            lcd.print("Invalid Cmd");
            break;
    }
}
