#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include "tfModel.h"
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>


// Blynk authentication and WiFi credentials
#define BLYNK_TEMPLATE_ID "TMPL6hNiz2vv6"
#define BLYNK_TEMPLATE_NAME "Motor Tire Air Leak Detection"
#define BLYNK_AUTH_TOKEN "tO-UCPuM3i76v_mq4yvvl6Vfs2L9suYt"
#define WIFI_SSID "Nut"
#define WIFI_PASS "hero1234"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Virtual pins for Blynk
#define VPIN_DETECTION_SWITCH V0             // Switch to control detection
#define VPIN_PREDICTION_OUTPUT V1            // Output for prediction result
#define VPIN_BAN_BELAKANG_KEMPES_COUNTER V6  // Counter for how many times ban belakang kempes got predicted
#define VPIN_BAN_DEPAN_KEMPES_COUNTER V5     // Counter for how many times ban depan kempes got predicted
#define VPIN_BAN_NORMAL_COUNTER V3           // Counter for how many times ban normal got predicted
#define VPIN_KEDUA_BAN_KEMPES_COUNTER V4     // Counter for how many times kedua ban kempes got predicted

#define ARENA_SIZE 73300
#define SEGMENT_LENGTH 150        // 15 seconds at 10 Hz
#define DETECTION_DURATION 300000  // 5 minutes in milliseconds


Adafruit_MPU6050 mpu;

Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;

static float sliding_window[SEGMENT_LENGTH * 6];  // 6 features (AccX, AccY, AccZ, GyroX, GyroY, GyroZ)
static int window_index = 0;
const int WINDOW_STEP = 10;  // Sliding window step size
static unsigned long lastSampleTime = 0;
static bool isDetectionEnabled = false;
const unsigned long SAMPLE_INTERVAL = 100;  // 100ms = 0.1s sampling rate
int predictionCounts[4] = { 0, 0, 0, 0 };   // Counter for each prediction class
static unsigned long detectionStartTime = 0;

// Convert prediction class to string
String getPredictionString(int predicted_class) {
  switch (predicted_class) {
    case 0:
      return "Ban Belakang Kempes";
    case 1:
      return "Ban Depan Kempes";
    case 2:
      return "Ban Normal";
    case 3:
      return "Kedua Ban Kempes";
    default:
      return "Unknown";
  }
}

void showPredictionCounters() {
  Blynk.virtualWrite(VPIN_BAN_BELAKANG_KEMPES_COUNTER, predictionCounts[0]);
  Blynk.virtualWrite(VPIN_BAN_DEPAN_KEMPES_COUNTER, predictionCounts[1]);
  Blynk.virtualWrite(VPIN_BAN_NORMAL_COUNTER, predictionCounts[2]);
  Blynk.virtualWrite(VPIN_KEDUA_BAN_KEMPES_COUNTER, predictionCounts[3]);
}

// Reset prediction counters
void resetInitials() {
  for (int i = 0; i < 4; i++) {
    predictionCounts[i] = 0;
  }

  // Reset Label Blynk Output
  Blynk.virtualWrite(VPIN_PREDICTION_OUTPUT, 4);
  window_index = 0;
  lastSampleTime = 0;
  detectionStartTime = 0;
}

// Handle detection switch state change
BLYNK_WRITE(VPIN_DETECTION_SWITCH) {
  isDetectionEnabled = param.asInt();
  if (isDetectionEnabled) {
    detectionStartTime = millis();  // Set detection start time when enabled
  } else {
    resetInitials();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Connect to WiFi and Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);


  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setting up MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(2000);

  tf.setNumInputs(TF_NUM_INPUTS);
  tf.setNumOutputs(TF_NUM_OUTPUTS);
  registerNetworkOps(tf);

  while (!tf.begin(tfModel).isOk()) {
    Serial.println(tf.exception.toString());
    delay(1000);
  }

  // Reset the label in blynk
  Blynk.virtualWrite(VPIN_PREDICTION_OUTPUT, 4);
}

void loop() {

  Blynk.run();

  if (!isDetectionEnabled) {
    window_index = 0;
    lastSampleTime = 0;
    detectionStartTime = 0;
    delay(100);
    return;
  }

  if (millis() - detectionStartTime >= DETECTION_DURATION) {

    // 5 minutes have passed, check predictionCounts
    int maxCount = 0;
    int maxIndex = -1;
    for (int i = 0; i < 4; i++) {
      if (predictionCounts[i] > maxCount) {
        maxCount = predictionCounts[i];
        maxIndex = i;
      }
    }

    if (maxIndex == 2) {  // Ban Normal has the most count
      resetInitials();
      lastSampleTime = 0;
      window_index = 0;
      detectionStartTime = millis();  // Restart detection

    } else {  // Other than Ban Normal has the most count
      String label = getPredictionString(maxIndex);
      Serial.print("STOP");

      Blynk.logEvent("tires_condition", label);
      Blynk.virtualWrite(VPIN_DETECTION_SWITCH, 0);
      resetInitials();
      isDetectionEnabled = false;
    }
    return;
  }

  if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    const size_t idx = window_index * 6;
    sliding_window[idx] = a.acceleration.x / 9.81f;
    sliding_window[idx + 1] = a.acceleration.y / 9.81f;
    sliding_window[idx + 2] = a.acceleration.z / 9.81f;
    sliding_window[idx + 3] = g.gyro.x;
    sliding_window[idx + 4] = g.gyro.y;
    sliding_window[idx + 5] = g.gyro.z;

    // Print current sensor values
    Serial.print("Window Index: ");
    Serial.print(window_index);
    Serial.print(" | AccX: ");
    Serial.print(sliding_window[idx]);
    Serial.print(" AccY: ");
    Serial.print(sliding_window[idx + 1]);
    Serial.print(" AccZ: ");
    Serial.print(sliding_window[idx + 2]);
    Serial.print(" | GyroX: ");
    Serial.print(sliding_window[idx + 3]);
    Serial.print(" GyroY: ");
    Serial.print(sliding_window[idx + 4]);
    Serial.print(" GyroZ: ");
    Serial.println(sliding_window[idx + 5]);

    // If we have collected enough samples, make a prediction
    if (++window_index >= SEGMENT_LENGTH) {

      // Print full window before prediction
      Serial.println("\nFull Window Data before prediction:");
      for (int i = 0; i < SEGMENT_LENGTH; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sliding_window[i * 6]);
        Serial.print(", ");
        Serial.print(sliding_window[i * 6 + 1]);
        Serial.print(", ");
        Serial.print(sliding_window[i * 6 + 2]);
        Serial.print(", ");
        Serial.print(sliding_window[i * 6 + 3]);
        Serial.print(", ");
        Serial.print(sliding_window[i * 6 + 4]);
        Serial.print(", ");
        Serial.println(sliding_window[i * 6 + 5]);
      }
      Serial.println();


      // Make a prediction
      if (!tf.predict(sliding_window).isOk()) {
        Serial.println(tf.exception.toString());
        return;
      }

      const int pred_class = tf.classification;

      // Update prediction counters (This will be used for confussion matrix)
      predictionCounts[pred_class]++;
      showPredictionCounters();

      // Send prediction to Blynk
      Blynk.virtualWrite(VPIN_PREDICTION_OUTPUT, pred_class);

      // Slide window
      memmove(sliding_window, &sliding_window[WINDOW_STEP * 6],
              (SEGMENT_LENGTH - WINDOW_STEP) * 6 * sizeof(float));
      window_index = SEGMENT_LENGTH - WINDOW_STEP;
    }
  }
}