#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 2  
#define LOADCELL_SCK_PIN 3   

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float calibration_factor = 71.4;  // Adjust this after calibration
const int NUM_READINGS = 5;  // Number of samples to average

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing HX711...");

  scale.begin();
  delay(500);

  Serial.println("Taring...");
  scale.tare();
  delay(2000);

  Serial.println("Calibration complete.");
}

void loop() {
  float total_weight = 0;

  // Take 5 readings and calculate average
  for (int i = 0; i < NUM_READINGS; i++) {
    scale.update();
    total_weight += scale.getData();
    delay(200);  // Small delay between readings
  }

  float avg_raw_weight = total_weight / NUM_READINGS;
  float adjusted_weight = avg_raw_weight * calibration_factor;

  Serial.print("Stable Weight: ");
  Serial.print(adjusted_weight);
  Serial.println(" g");

  delay(1000);  // Update every second
}
