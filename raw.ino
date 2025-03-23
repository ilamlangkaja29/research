#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 2  
#define LOADCELL_SCK_PIN 3   

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing HX711...");

  scale.begin();
  delay(500);

  Serial.println("Taring... Remove all weight.");
  scale.tare();
  delay(2000);

  Serial.println("Now place the 500g weight.");
  delay(5000);  // Wait 5 seconds for you to place weight
}

void loop() {
  scale.update();
  float raw_value = scale.getData();

  Serial.print("Raw Sensor Value: ");
  Serial.println(raw_value);
  
  delay(1000);  // Print every second
}
