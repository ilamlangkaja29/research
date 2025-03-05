#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX, TX 
const int BTStatePin = 9; // 
char receivedChar;

void setup() {
    pinMode(BTStatePin, INPUT);
    Serial.begin(9600);
    BTSerial.begin(9600);

    Serial.println("Bluetooth Test Initialized");
}

void loop() {
    // Check Bluetooth connection status
    if (digitalRead(BTStatePin) == HIGH) {
        Serial.println("Bluetooth Connected!");
    } else {
        Serial.println("Waiting for Bluetooth connection...");
    }

    // Check if data is received from Bluetooth
    if (BTSerial.available()) {
        receivedChar = BTSerial.read();
        Serial.print("Received from Bluetooth: ");
        Serial.println(receivedChar);
    }

    // Send data to Bluetooth if entered in Serial Monitor
    if (Serial.available()) {
        receivedChar = Serial.read();
        BTSerial.print(receivedChar);
        Serial.print("Sent to Bluetooth: ");
        Serial.println(receivedChar);
    }

    delay(1000); // Reduce serial spam
}
