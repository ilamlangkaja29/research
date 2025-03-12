// HC-05 Bluetooth Controlled Motor Driver Code with L298N (4-pin control)
// Uses RX (Pin 0) and TX (Pin 1) for Bluetooth communication

#define IN1 4     // L298N IN1 Pin
#define IN2 5     // L298N IN2 Pin
#define IN3 6     // L298N IN3 Pin
#define IN4 7     // L298N IN4 Pin
#define ENA 8     // L298N ENA (Speed Control)
#define ENB 9     // L298N ENB (Speed Control)

void setup() {
    Serial.begin(9600);        // Serial Monitor communication

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);      // Motor A speed control
    pinMode(ENB, OUTPUT);      // Motor B speed control

    digitalWrite(IN1, LOW);    // Motors OFF initially
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    Serial.println("Waiting for Bluetooth commands...");
}

void loop() {
    if (Serial.available()) {  // Read from Bluetooth (RX/TX Pins)
        String received = Serial.readString();
        received.trim();  // Remove whitespace and newline characters

        // Reflect received command to the serial monitor
        Serial.print("Received command: ");
        Serial.println(received);

        // Motor control commands
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
