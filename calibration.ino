#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 2  
#define LOADCELL_SCK_PIN 3   

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float calibration_factor = 1.0;  // Adjust this to correct the reading

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing HX711...");

  scale.begin();
  delay(500);

  Serial.println("Remove all weight and wait...");
  delay(3000);

  Serial.println("Taring...");
  scale.tare();
  delay(2000);

  Serial.println("Place a known weight (500g) on the sensor.");
  delay(5000);  // Give time to place weight
}

void loop() {
  scale.update();
  float raw_weight = scale.getData();

  float adjusted_weight = raw_weight * calibration_factor;  // Apply correction

  Serial.print("Raw Weight: ");
  Serial.print(raw_weight);
  Serial.print(" | Adjusted Weight: ");
  Serial.print(adjusted_weight);
  Serial.println(" g");

  delay(1000);
}
