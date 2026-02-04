#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

// Hardware connection settings
#define DHTPIN 13
#define DHTTYPE DHT22
#define POT_PIN 34

// Components setup
DHT dht(DHTPIN, DHTTYPE);
Servo myMotor;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Settings for data filtering
const int FILTER_SIZE = 10;           // Number of data points to average
float history[FILTER_SIZE] = {0, };   // Array to store recent sensor values
int count = 0;                        // Counter for total data samples

// Variables for sensor health check
float lastRawVib = -1.0;              // To check if the sensor value is frozen
int stuckCounter = 0;                 // Counter for "Stuck" error
int outlierCounter = 0;               // Counter for "Outlier" (extreme values)

void setup() {
  Serial.begin(115200);
  dht.begin();
  myMotor.attach(18);
  lcd.init();
  lcd.backlight();
  
  // Message to confirm the system started correctly
  Serial.println(">>> SYSTEM BOOT COMPLETE <<<");
  Serial.println(">>> ARCHITECT: STUDENT & AI ASSISTANT <<<");
}

void loop() {
  // 1. Reading data from sensors
  int potValue = analogRead(POT_PIN);
  float rawVib = (potValue / 4095.0) * 100.0; // Convert to 0-100% scale
  float rawTemp = dht.readTemperature();

  // 2. Self-Diagnosis (Check if sensor is broken)
  // If the sensor value doesn't change at all for 50 cycles, it might be stuck.
  if (rawVib == lastRawVib) {
    stuckCounter++;
  } else {
    stuckCounter = 0; // Reset if the value changes (normal)
  }
  lastRawVib = rawVib; 

  // 3. Handling Outliers (Leaky Bucket Logic)
  // To ignore one-time noise, I used a counter that increases on high values.
  if (rawVib >= 95.0) {
    outlierCounter++;
  } else if (outlierCounter > 0) {
    outlierCounter--; // Slowly decrease if the value returns to normal
  }

  // 4. Moving Average Filter (Ring Buffer)
  // Making the data smooth by averaging the last 10 values.
  int index = count % FILTER_SIZE; 
  history[index] = rawVib; 
  count++;

  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += history[i];
  }

  // Use current count if we don't have 10 data points yet
  int divisor = (count < FILTER_SIZE) ? count : FILTER_SIZE;
  float filteredVib = sum / divisor;

  // 5. Control Logic (Safety First)
  String status = "NORMAL";
  int motorAngle = 180;

  // Check for errors in order of importance
  if (stuckCounter >= 50) {
    status = "SENSOR ERROR";
    motorAngle = 90; // Move to safe position
  }
  else if (outlierCounter >= 5 || rawTemp >= 60.0 || filteredVib >= 80.0) {
    status = "EMERGENCY STOP";
    motorAngle = 0;  // Stop the system
  }
  else if (filteredVib >= 50.0 || rawTemp >= 45.0) {
    status = "WARNING";
    motorAngle = 90;
  }

  // 6. Output (LCD and Servo)
  myMotor.write(motorAngle);

  lcd.setCursor(0, 0);
  lcd.print("V:" + String(filteredVib, 0) + "% T:" + String(rawTemp, 0) + "C");
  lcd.setCursor(0, 1);
  lcd.print(status);

  // Debugging logs for Serial Monitor
  Serial.print("Raw:"); Serial.print(rawVib);
  Serial.print(" Filtered:"); Serial.print(filteredVib);
  Serial.print(" StuckCnt:"); Serial.print(stuckCounter);
  Serial.print(" OutlierCnt:"); Serial.println(outlierCounter);

  delay(100); 
}
