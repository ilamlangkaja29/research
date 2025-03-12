#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);  // RX | TX (HC-05 connected to pins 2 and 3)

void setup() {
    Serial.begin(9600);    // Serial Monitor Communication
    BTSerial.begin(9600);  // Bluetooth Module Communication (HC-05 default baud rate)

    Serial.println("Bluetooth Module Ready...");
    Serial.println("Waiting for command from phone...");
}

void loop() {
    // Check if data is received from Bluetooth
    if (BTSerial.available()) {
        String receivedCommand = BTSerial.readString();
        receivedCommand.trim();  // Remove extra spaces and newline characters

        // Print received command on Serial Monitor
        Serial.print("Received from phone: ");
        Serial.println(receivedCommand);

        // Send acknowledgment back to the phone
        BTSerial.print("Command Received: ");
        BTSerial.println(receivedCommand);
    }
}
