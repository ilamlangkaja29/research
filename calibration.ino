#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 2  
#define LOADCELL_SCK_PIN 3   

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float fixed_weight = 0;  
int read_count = 0;
const int max_reads = 5;  // Take only 5 readings

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
  if (read_count < max_reads) {
    scale.update();
    float current_weight = scale.getData();

    if (abs(current_weight - fixed_weight) > 1.0) {  
      fixed_weight = current_weight;
    }

    Serial.print("Stable Weight [");
    Serial.print(read_count + 1);
    Serial.print("/5]: ");
    Serial.print(fixed_weight);
    Serial.println(" g");

    read_count++;  
  } else {
    Serial.print("Final Fixed Weight: ");
    Serial.print(fixed_weight);
    Serial.println(" g");
    while (true);  // Stop updating after 5 readings
  }

  delay(500);
}
