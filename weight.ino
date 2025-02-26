#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>

// LCD Setup (I2C Address: 0x27, 16x2 Display)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// HX711 Load Cell Setup
#define LOADCELL_DOUT 2  // Data pin
#define LOADCELL_SCK 3   // Clock pin
HX711 scale;

void setup() {
    Serial.begin(9600);
    
    // Initialize LCD
    lcd.begin(16, 2);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Weight Sensor Test");

    // Initialize Load Cell
    scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
    scale.set_scale();    // Default scale (Needs calibration)
    scale.tare();         // Reset to zero
}

void loop() {
    // Read weight from Load Cell
    float weight = scale.get_units(10);  

    // Display weight on LCD
    lcd.setCursor(0, 1);
    lcd.print("Weight: ");
    lcd.print(weight, 2);  // Display 2 decimal places
    lcd.print(" g  ");     // "g" for grams

    // Print weight in Serial Monitor (for debugging)
    Serial.print("Weight: ");
    Serial.print(weight, 2);
    Serial.println(" g");

    delay(500);  // Small delay for stable reading
}
