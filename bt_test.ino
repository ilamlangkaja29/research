#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX, TX

void setup() {
    Serial.begin(9600);
    BTSerial.begin(38400); // Try 38400 baud first
    Serial.println("Send AT commands:");
}

void loop() {
    if (BTSerial.available()) {
        Serial.write(BTSerial.read());
    }
    if (Serial.available()) {
        BTSerial.write(Serial.read());
    }
}
