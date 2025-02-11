// Motor control pins
const int motor1Pin1 = 4; // Connect to D4
const int motor1Pin2 = 5; // Connect to D5
const int motor2Pin1 = 6; // Connect to D6
const int motor2Pin2 = 7; // Connect to D7

void setup() {
  // Define motor control pins as OUTPUT
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if data is available on the Serial port
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();

    // Perform actions based on the received command
    switch (command) {
      case 'F': // Forward
        moveForward();
        Serial.println("Moving Forward");
        break;
      case 'B': // Backward
        moveBackward();
        Serial.println("Moving Backward");
        break;
      case 'L': // Left (anticlockwise)
        turnLeft();
        Serial.println("Turning Left");
        break;
      case 'R': // Right (clockwise)
        turnRight();
        Serial.println("Turning Right");
        break;
      default:
        // Stop the motors if an unknown command is received
        stopMotors();
        Serial.println("Unknown Command. Motors Stopped");
        break;
    }
  }
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void moveBackward() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void turnLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void turnRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

