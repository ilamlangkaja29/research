#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 2  // Changed to pin 2
#define LOADCELL_SCK_PIN 3   // Changed to pin 3

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float fixed_weight = 0;  

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

  Serial.println("Calibration complete.");
}

void loop() {
  scale.update();  
  float current_weight = scale.getData();

  if (abs(current_weight - fixed_weight) > 1.0) {  
    fixed_weight = current_weight;  
  }

  Serial.print("Stable Weight: ");
  Serial.print(fixed_weight);
  Serial.println(" g");

  delay(500);
}
