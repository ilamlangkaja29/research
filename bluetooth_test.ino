char receivedChar; // Variable to store received data

void setup() {
    Serial.begin(9600);  // Start serial communication with PC
    Serial.println("Bluetooth Test Initialized");
    
    Serial1.begin(9600); // Start serial communication with HC-05 (TX:1, RX:0)
    Serial.println("Waiting for Bluetooth connection...");
}

void loop() {
    // Check if data is received from Bluetooth
    if (Serial1.available()) {
        receivedChar = Serial1.read();
        Serial.print("Received from Bluetooth: ");
        Serial.println(receivedChar);
    }

    // Send data to Bluetooth if entered in Serial Monitor
    if (Serial.available()) {
        receivedChar = Serial.read();
        Serial1.print(receivedChar);
        Serial.print("Sent to Bluetooth: ");
        Serial.println(receivedChar);
    }
}
