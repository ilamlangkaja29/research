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

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600); // Bluetooth Communication
    
    // Initialize LCD
    lcd.begin(16, 2);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Weight Sensor Init");
    
    // Initialize Load Cell
    scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
    scale.set_scale(); // Adjust based on your calibration
    scale.tare(); // Reset scale to zero
    
    delay(2000);
    lcd.clear();
}

void loop() {
    float weight = scale.get_units(10); // Get average of 10 readings
    Serial.print("Weight: ");
    Serial.print(weight);
    Serial.println(" g");
    
    lcd.setCursor(0, 0);
    lcd.print("Weight: ");
    lcd.print(weight);
    lcd.print(" g ");
    
    // Send weight data via Bluetooth
    BTSerial.print("Weight: ");
    BTSerial.print(weight);
    BTSerial.println(" g");
    
    delay(500);
}
