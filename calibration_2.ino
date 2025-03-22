#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN 7

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float calibration_factor = 2.0;  // Use your calibrated value
float bin_weight = 0;  // Stores the bin's weight after taring

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Weight Sensor...");

  scale.begin();
  scale.setCalFactor(calibration_factor);

  Serial.println("Taring scale (removing bin weight)...");
  scale.tare();  // Tare to remove bin weight
  delay(3000);   // Allow sensor to stabilize

  bin_weight = scale.getData();  // Store bin's weight
  Serial.print("Bin Weight (Tared): ");
  Serial.println(bin_weight);

  Serial.println("Weight Sensor Ready...");
}

void loop() {
  if (scale.update()) {
    float total_weight = scale.getData();
    float garbage_weight = total_weight - bin_weight;  // Subtract bin weight

    Serial.print("Garbage Weight: ");
    Serial.print(garbage_weight);
    Serial.println(" g");
  }
  delay(500);
}
