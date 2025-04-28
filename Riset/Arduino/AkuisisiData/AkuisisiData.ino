#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include "FS.h"
#include <SD.h>
#include <SPI.h>

// SD Card pins for TTGO LoRa32
#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13

#define BLYNK_TEMPLATE_ID "TMPL69S-QKSJr"
#define BLYNK_TEMPLATE_NAME "Akuisisi Data Ban"
#define BLYNK_AUTH_TOKEN "lpO_qyUQp3xs-e8y1bs4xLTwOcjAuvB_"
#define WIFI_SSID "Nut"
#define WIFI_PASS "hero1234"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

BlynkTimer timer;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Blynk Virtual Pin settings
#define VIRTUAL_PIN_SWITCH V0  // Virtual Pin for switch to start/stop recording
#define VIRTUAL_PIN_TIME V1    // Virtual Pin to display elapsed time
#define VIRTUAL_PIN_FILENAME V2 // Virtual Pin to input the filename for the recorded data

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MPU6050 mpu;

unsigned long lastTime = 0; // menyimpan waktu terakhir data dibaca
unsigned long timerDelay = 100; // interval 100 ms
unsigned long recordingStartTime = 0;
unsigned long sessionStartTime = 0; // For timestamp reset
bool isRecording = false;
String fileName = "data.csv"; // Default file name

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

bool isFileEmpty(fs::FS &fs, const char * path) {
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return true; // Treat non-existent file as empty
  }
  bool isEmpty = file.size() == 0;
  file.close();
  return isEmpty;
}

String getTimeStamp() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime;
  
  // Calculate time since recording session started
  if (isRecording) {
    elapsedTime = currentTime - sessionStartTime;
  } else {
    elapsedTime = 0;
  }

  unsigned long milliseconds = elapsedTime % 1000;
  unsigned long totalSeconds = elapsedTime / 1000;
  unsigned long seconds = totalSeconds % 60;
  unsigned long totalMinutes = totalSeconds / 60;
  unsigned long minutes = totalMinutes % 60;
  unsigned long hours = totalMinutes / 60;

  // Format with leading zeros
  String hoursStr = (hours < 10) ? "0" + String(hours) : String(hours);
  String minutesStr = (minutes < 10) ? "0" + String(minutes) : String(minutes);
  String secondsStr = (seconds < 10) ? "0" + String(seconds) : String(seconds);
  String millisecondsStr = String(milliseconds);
  // Ensure milliseconds has three digits
  while (millisecondsStr.length() < 3) {
    millisecondsStr = "0" + millisecondsStr;
  }

  return hoursStr + ":" + minutesStr + ":" + secondsStr + ":" + millisecondsStr;
}

void writeDataToFile(fs::FS &fs, const char * path, sensors_event_t a, sensors_event_t g) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  
  if (isFileEmpty(fs, path)) {
    String header = String("TimeStamp") + "," + 
                    String("AccX") + "," + String("AccY") + "," + String("AccZ") + "," +
                    String("GyrX") + "," + String("GyrY") + "," + String("GyrZ") + "\n";
    if (!file.print(header)) {
      Serial.println("Failed to write header");
      file.close();
      return;
    }
  }

  // Konversi nilai percepatan dari m/s^2 ke g
  float accel_scale = 1.0f / 9.81f; // 1 g = 9.81 m/s^2
  float accel_x_g = a.acceleration.x * accel_scale;
  float accel_y_g = a.acceleration.y * accel_scale;
  float accel_z_g = a.acceleration.z * accel_scale;

  // Membuat string data dengan nilai percepatan yang telah diubah ke satuan g
  String dataString = String(getTimeStamp()) + "," + 
                      String(accel_x_g) + "," + String(accel_y_g) + "," + String(accel_z_g) + "," +
                      String(g.gyro.x) + "," + String(g.gyro.y) + "," + String(g.gyro.z) + "\n";

  if (file.print(dataString)) {
    Serial.println("Data appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

BLYNK_WRITE(VIRTUAL_PIN_SWITCH) {
  int pinValue = param.asInt();
  if (pinValue == 1) {
    // Start recording
    if (!isRecording) {
      isRecording = true;
      recordingStartTime = millis(); // For 10-minute limit checking
      sessionStartTime = millis();   // For timestamp reset
      Serial.println("Recording started...");
    }
  } else {
    // Stop recording
    if (isRecording) {
      isRecording = false;
      Serial.println("Recording stopped...");
    }
  }
  Serial.println(isRecording);
}

BLYNK_WRITE(VIRTUAL_PIN_FILENAME) {
  String newFileName = param.asString();  // Get file name from Blynk text input
  if (!newFileName.endsWith(".csv")) {
    newFileName = "/" + newFileName + ".csv";  // Ensure file name ends with ".csv"
  }
  
  // If filename changes while recording, stop recording
  if (fileName != newFileName && isRecording) {
    isRecording = false;
    sessionStartTime = 0; // Reset session timer
    Blynk.virtualWrite(VIRTUAL_PIN_SWITCH, 0);
    Serial.println("Recording stopped due to filename change");
  }
  
  fileName = newFileName;
}

void updateElapsedTime() {
  if (isRecording) {
    unsigned long elapsedMillis = millis() - recordingStartTime;
    unsigned long minutes = (elapsedMillis / 60000);
    unsigned long seconds = (elapsedMillis / 1000) % 60;
    String elapsedTime = String(minutes) + ":" + String(seconds);

    Blynk.virtualWrite(VIRTUAL_PIN_TIME, elapsedTime);
    
    // Stop recording after 10 minutes (600000 ms)
    if (elapsedMillis >= 600000) {
      isRecording = false;
      sessionStartTime = 0; // Reset session timer
      Serial.println("Recording time exceeded 10 minutes. Stopping...");
      Blynk.virtualWrite(VIRTUAL_PIN_SWITCH, 0); // Turn off the switch
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup");

  // Blynk setup
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);

  // Try to initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set up the display
  Serial.println("Initializing display");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  Serial.println("Display initialized");
  display.display();
  delay(2000);
  display.clearDisplay();

  // Setting up MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize SPI for SD card
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  // Initialize SD card
  Serial.println("Initializing SD card");
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }

  Serial.println("SD card initialized");
    
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
    
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  listDir(SD, "/", 2); // List directory contents

  timer.setInterval(1000L, updateElapsedTime); // Update elapsed time every second
}

void loop() {
  Blynk.run();
  timer.run();

  unsigned long currentMillis = millis();

  if (isRecording && (currentMillis - lastTime) >= timerDelay) {
    lastTime = currentMillis;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("AccX : "); display.print(a.acceleration.x / 9.81f);
    display.print(" Y : "); display.print(a.acceleration.y / 9.81f);
    display.print(" Z : "); display.print(a.acceleration.z / 9.81f);
    display.println(" g");
    display.print("GyrX : "); display.print(g.gyro.x);
    display.print(" Y: "); display.print(g.gyro.y);
    display.print(" Z: "); display.print(g.gyro.z);
    display.println(" g");
    display.display();

    writeDataToFile(SD, fileName.c_str(), a, g);
  }
}
