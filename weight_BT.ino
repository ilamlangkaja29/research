#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#include <SoftwareSerial.h>

// Load Cell (HX711) Pins
#define LOADCELL_DOUT 2
#define LOADCELL_SCK 3

// Bluetooth HC-05 (Software Serial)
#define BT_TX 10
#define BT_RX 11
SoftwareSerial BTSerial(BT_TX, BT_RX);

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);
HX711 scale;

// Calibration Values (Use your own)
const int32_t knownOffset = -17770;
const float scaleFactor = 0.973008;

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600); // Bluetooth Communication
    
    // Initialize LCD
    lcd.begin(16, 2);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    
    // Initialize Load Cell with Calibration
    scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
    scale.set_offset(knownOffset);
    scale.set_scale(scaleFactor);

    delay(2000);
    lcd.clear();
    Serial.println("Scale Ready!");
}

void loop() {
    float weight = scale.get_units(10); // Get average of 10 readings

    // Print to Serial Monitor
    Serial.print("Weight: ");
    Serial.print(weight, 2); // Show 2 decimal places
    Serial.println(" g");

    // Print to LCD
    lcd.setCursor(0, 0);
    lcd.print("Weight:       "); // Clear previous digits
    lcd.setCursor(8, 0);
    lcd.print(weight, 2);
    lcd.print(" g");

    // Send weight data via Bluetooth
    BTSerial.print("Weight: ");
    BTSerial.print(weight, 2);
    BTSerial.println(" g");

    delay(500);
}
