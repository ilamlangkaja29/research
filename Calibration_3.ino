#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN 7

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float calibration_factor = 10.0;  // Update based on your calculation
float bin_weight = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Weight Sensor...");

  scale.begin();
  scale.setCalFactor(calibration_factor);

  Serial.println("Taring scale...");
  scale.tare();
  delay(3000);

  bin_weight = scale.getData();
  Serial.print("Tared Bin Weight: ");
  Serial.println(bin_weight);
}

void loop() {
  if (scale.update()) {
    float total_weight = scale.getData();
    float garbage_weight = total_weight - bin_weight;

    Serial.print("Garbage Weight: ");
    Serial.print(garbage_weight);
    Serial.println(" g");
  }

  // Adjust calibration factor with '+' and '-'
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '+') calibration_factor += 0.1;
    if (command == '-') calibration_factor -= 0.1;

    scale.setCalFactor(calibration_factor);
    Serial.print("New Calibration Factor: ");
    Serial.println(calibration_factor);
  }

  delay(500);
}
