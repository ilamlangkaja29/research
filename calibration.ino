#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN 7

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting HX711...");

  scale.begin();
  if (scale.isReady()) {
    Serial.println("HX711 is ready.");
  } else {
    Serial.println("⚠ HX711 NOT detected. Check wiring!");
  }
}

void loop() {
}
