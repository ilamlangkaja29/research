#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN 7

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Weight Sensor...");

  scale.begin();

  Serial.println("Remove all weight and wait...");
  delay(3000);

  Serial.println("Taring...");
  scale.tare();  // Set zero weight
  delay(2000);

  Serial.println("Starting measurement...");
}

void loop() {
  if (scale.update()) {
    float raw_value = scale.getData();  

    Serial.print("Raw Sensor Value: ");
    Serial.println(raw_value);
  }
  delay(500);
}
