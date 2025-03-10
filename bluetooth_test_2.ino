#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// Initialize SoftwareSerial for Bluetooth communication
SoftwareSerial BTSerial(10, 11); // RX, TX
const int BTStatePin = 9;
char receivedChar;
bool btConnected = false;

// Initialize LCD (Assuming I2C address 0x27, adjust if necessary)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    pinMode(BTStatePin, INPUT);
    Serial.begin(9600);
    BTSerial.begin(9600);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(1000);
    lcd.clear();
    
    Serial.println("Bluetooth Test Initialized");
}

void loop() {
    bool currentBTState = digitalRead(BTStatePin);
    
    // Check Bluetooth connection status
    if (currentBTState && !btConnected) {
        btConnected = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BT Connected");
        lcd.setCursor(0, 1);
        lcd.print("Ready for command");
        Serial.println("Bluetooth Connected! Ready for command.");
    } else if (!currentBTState && btConnected) {
        btConnected = false;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Waiting for BT");
        Serial.println("Waiting for Bluetooth connection...");
    }
    
    // Check if a single uppercase letter is received from Bluetooth
    if (btConnected && BTSerial.available()) {
        receivedChar = BTSerial.read();
        if (receivedChar >= 'A' && receivedChar <= 'Z') {
            Serial.print("Received: ");
            Serial.println(receivedChar);
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Received: ");
            lcd.setCursor(10, 0);
            lcd.print(receivedChar);
            delay(2000); // Display command before resetting
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("BT Connected");
            lcd.setCursor(0, 1);
            lcd.print("Ready for command");
        }
    }
}
