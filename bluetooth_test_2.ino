#include <SoftwareSerial.h>

SoftwareSerial BTSerial(0, 1); // RX, TX (Use different pins if needed)

char receivedChar;

void setup() {
    Serial.begin(9600);   // Serial Monitor communication
    BTSerial.begin(9600); // Bluetooth HC-05 communication

    Serial.println("Bluetooth Test Initialized");
    Serial.println("Waiting for Bluetooth connection...");
}

void loop() {
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
}
