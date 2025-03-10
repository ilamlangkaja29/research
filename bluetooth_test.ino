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

    // Debug: Check if any data is available
    if (BTSerial.available()) {
        Serial.print("Bytes Available: ");
        Serial.println(BTSerial.available());
    }

    // Check if a single uppercase letter is received from Bluetooth
    while (btConnected && BTSerial.available()) {
        receivedChar = BTSerial.read();
        
        Serial.print("Raw Received: ");
        Serial.println(receivedChar);  // Debugging

        // Convert to uppercase if lowercase
        if (receivedChar >= 'a' && receivedChar <= 'z') {
            receivedChar -= 32;
        }
        
        // Process only uppercase letters (A-Z)
        if (receivedChar >= 'A' && receivedChar <= 'Z') {
            Serial.print("Processed Command: ");
            Serial.println(receivedChar);
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Received: ");
            lcd.setCursor(10, 0);
            lcd.print(receivedChar);
            
            delay(2000); // Display command before resetting
            
            // Flush extra data
            while (BTSerial.available()) {
                char flushChar = BTSerial.read();
                Serial.print("Flushing: ");
                Serial.println(flushChar);
            }
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("BT Connected");
            lcd.setCursor(0, 1);
            lcd.print("Ready for command");
            break; // Exit after processing one valid command
        }
    }
}
