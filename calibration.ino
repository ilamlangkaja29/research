#include "HX711.h"

HX711 myScale;

// Corrected pin assignment
const uint8_t dataPin = 2;  // HX711 DOUT
const uint8_t clockPin = 3; // HX711 SCK

// Use your known offset value
const int32_t knownOffset = YOUR_OFFSET_VALUE;  // Replace with your actual offset

void setup()
{
    Serial.begin(115200);
    Serial.println("HX711 Quick Calibration Tool");

    // Initialize HX711 with known offset
    myScale.begin(dataPin, clockPin);
    myScale.set_offset(knownOffset);
    
    Serial.print("Using known OFFSET: ");
    Serial.println(knownOffset);
    
    Serial.println("\nPlace a 100g weight on the load cell.");
    Serial.println("Then press ENTER in the Serial Monitor.");
    
    // Wait for user confirmation
    while (Serial.available()) Serial.read();
    while (Serial.available() == 0);
    Serial.read();  // Consume ENTER key press

    Serial.println("Calibrating with 100g weight...");
    
    myScale.calibrate_scale(100, 20);  // Use 100g as reference weight
    float scale = myScale.get_scale();

    Serial.print("New SCALE FACTOR: ");
    Serial.println(scale, 6);

    Serial.println("\nUse these values in your project:");
    Serial.print("scale.set_offset(");
    Serial.print(knownOffset);
    Serial.println(");");

    Serial.print("scale.set_scale(");
    Serial.print(scale, 6);
    Serial.println(");");

    Serial.println("Calibration complete!");
}

void loop() {
    // No need for repeated calibration, loop remains empty
}
