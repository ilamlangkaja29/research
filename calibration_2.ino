#include <HX711_ADC.h>

#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN 7

HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

const int numReadings = 10;  // Number of samples to average
float lastStableValue = 0;   // Store the last stable weight
float stabilityThreshold = 5;  // Minimum change required to update display

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
  float total = 0;
  
  for (int i = 0; i < numReadings; i++) {
    if (scale.update()) {
      total += scale.getData();
      delay(50);  // Small delay to get stable readings
    }
  }
  
  float avgValue = total / numReadings;

  // Update only if change is significant
  if (abs(avgValue - lastStableValue) >= stabilityThreshold) {
    Serial.print("Stable Weight: ");
    Serial.print(avgValue);
    Serial.println(" g");
    lastStableValue = avgValue;  // Update last stable value
  }

  delay(500);
}
