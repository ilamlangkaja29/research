// IR Sensor Pins
#define LEFT_SENSOR 2  
#define RIGHT_SENSOR 3  

// Motor Driver Pins
#define MOTOR_LEFT_FORWARD 6  
#define MOTOR_LEFT_BACKWARD 7  
#define MOTOR_RIGHT_FORWARD 8  
#define MOTOR_RIGHT_BACKWARD 9  

void setup() {
    pinMode(LEFT_SENSOR, INPUT);
    pinMode(RIGHT_SENSOR, INPUT);

    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
    
    Serial.begin(9600);
}

void loop() {
    int leftState = digitalRead(LEFT_SENSOR);  // Read left sensor
    int rightState = digitalRead(RIGHT_SENSOR);  // Read right sensor

    Serial.print("Left Sensor: ");
    Serial.print(leftState ? "White " : "Black ");
    Serial.print(" | Right Sensor: ");
    Serial.println(rightState ? "White" : "Black");

    // Both sensors detect white (off the line) → Stop
    if (leftState == HIGH && rightState == HIGH) {
        stopMotors();
    }
    // Left sensor on black, right on white → Turn Left
    else if (leftState == LOW && rightState == HIGH) {
        turnLeft();
    }
    // Left sensor on white, right on black → Turn Right
    else if (leftState == HIGH && rightState == LOW) {
        turnRight();
    }
    // Both sensors on black (on the line) → Move Forward
    else {
        moveForward();
    }

    delay(100); // Short delay for stability
}

// Function to move forward
void moveForward() {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// Function to turn left
void turnLeft() {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// Function to turn right
void turnRight() {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// Function to stop motors
void stopMotors() {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}
