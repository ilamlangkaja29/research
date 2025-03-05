#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Bluetooth module
SoftwareSerial BTSerial(10, 11); // RX, TX
char receivedChar;
bool btConnected = false;

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
    delay(100); // Short delay for LCD stability
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
    lcd.print("Waiting for BT...");
}

void loop() {
    // Check for Bluetooth connection
    if (BTSerial.available()) {
        if (!btConnected) {
            btConnected = true;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("BT Connected");
            lcd.setCursor(0, 1);
            lcd.print("Waiting for Cmd");
        }
        
        receivedChar = BTSerial.read();
        if (isValidCommand(receivedChar)) {
            Serial.print("Received: ");
            Serial.println(receivedChar);
            lcd.setCursor(0, 0);
            lcd.print("BT: ");
            lcd.print(receivedChar);
            lcd.setCursor(0, 1);
            handleMotor(receivedChar);
        }
    } else {
        if (btConnected) {
            btConnected = false;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("No BT Signal");
            lcd.setCursor(0, 1);
            lcd.print("Waiting...");
        }
    }

    // Send data from Serial Monitor to Bluetooth
    if (Serial.available()) {
        receivedChar = Serial.read();
        if (isValidCommand(receivedChar)) {
            BTSerial.print(receivedChar);
            Serial.print("Sent: ");
            Serial.println(receivedChar);
            handleMotor(receivedChar);
        }
    }
}

void handleMotor(char command) {
    lcd.clear();
    lcd.setCursor(0, 0);
    switch (command) {
        case 'F': // Forward
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.print("Moving Forward");
            break;
        case 'B': // Backward
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.print("Moving Backward");
            break;
        case 'L': // Left
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.print("Turning Left");
            break;
        case 'R': // Right
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            lcd.print("Turning Right");
            break;
        case 'S': // Stop
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 0);
            analogWrite(ENB, 0);
            lcd.print("Stopped");
            break;
        default:
            lcd.print("Invalid Cmd");
            break;
    }
}

bool isValidCommand(char cmd) {
    return (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S');
}
