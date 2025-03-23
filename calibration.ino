#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN 7

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting HX711...");

  scale.begin();
  delay(500);  // Allow HX711 to stabilize

  Serial.println("Taring...");
  scale.tare();
  delay(2000);

  Serial.println("Start measuring...");
}

void loop() {
  if (scale.update()) {
    float raw_value = scale.getData();
    Serial.print("Raw Sensor Value: ");
    Serial.println(raw_value);
  }
  delay(500);
}
