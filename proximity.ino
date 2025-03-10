#define IR_SENSOR_1 2  // First IR sensor output pin
#define IR_SENSOR_2 3  // Second IR sensor output pin

void setup() {
    pinMode(IR_SENSOR_1, INPUT);
    pinMode(IR_SENSOR_2, INPUT);
    Serial.begin(9600); // Start Serial Monitor
}

void loop() {
    int sensor1State = digitalRead(IR_SENSOR_1);
    int sensor2State = digitalRead(IR_SENSOR_2);

    Serial.print("Sensor 1: ");
    Serial.print(sensor1State ? "No Object" : "Object Detected");
    Serial.print(" | Sensor 2: ");
    Serial.println(sensor2State ? "No Object" : "Object Detected");

    delay(500); // Wait for 500ms
}
