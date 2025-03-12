#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX, TX (Arduino -> HC-05)

#define IN1 4     
#define IN2 5     
#define IN3 6     
#define IN4 7     
#define ENA 8     
#define ENB 9     

void setup() {
    Serial.begin(9600);   // For Serial Monitor
    BTSerial.begin(9600); // For Bluetooth HC-05

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    Serial.println("Waiting for Bluetooth commands...");
}

void loop() {
    if (BTSerial.available()) {  // Read from Bluetooth
        String received = BTSerial.readString();
        received.trim();

        Serial.print("Received command: ");
        Serial.println(received);

        if (received == "F") {
            Serial.println("Executing: FORWARD");
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 200);
            analogWrite(ENB, 200);
        } else if (received == "B") {
            Serial.println("Executing: BACKWARD");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, 200);
            analogWrite(ENB, 200);
        } else if (received == "L") {
            Serial.println("Executing: LEFT TURN");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 150);
            analogWrite(ENB, 200);
        } else if (received == "R") {
            Serial.println("Executing: RIGHT TURN");
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, 200);
            analogWrite(ENB, 150);
        } else if (received == "S") {
            Serial.println("Executing: STOP");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
        } else if (received.startsWith("SPEED")) {
            int speedValue = received.substring(6).toInt();
            speedValue = constrain(speedValue, 0, 255);
            analogWrite(ENA, speedValue);
            analogWrite(ENB, speedValue);
            Serial.print("Executing: SPEED SET TO ");
            Serial.println(speedValue);
        } else {
            Serial.println("Unknown command received.");
        }
    }
}
