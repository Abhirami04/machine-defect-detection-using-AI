#include <esp_now.h>
#include <WiFi.h>

// Pin Definitions
#define RELAY_PIN 4
#define ACS712_PIN 34
#define VOLTAGE_PIN 35
#define FAULT_PIN 2 // D2 for motor fault detection

// Constants
const float ACS712_SENSITIVITY = 0.185; // 5A module: 185mV/A
const float VOLTAGE_DIVIDER_RATIO = 3.2; // For R1=22kΩ, R2=10kΩ
const float MAX_CURRENT = 2.5; // Max safe current in Amps
const float MIN_CURRENT = 0.078; // Normal operating current in Amps
float ZERO_CURRENT_VOLTAGE = 2.20; // Adjusted during calibration
const float MIN_VOLTAGE = 6.0; // Undervoltage threshold for 230V motor

// Data structure
typedef struct struct_message {
  bool motorCommand;
  float voltage;
  float current;
  int status; // 0: normal, 1: overload, 2: dry run, 3: line break, 4: undervoltage, 5: no voltage, 6: motor faulty
} struct_message;

struct_message motorData;

// Global variable to store transmitter's MAC address
uint8_t transmitterMacAddress[] = {0x8C, 0x4F, 0x00, 0xAC, 0x21, 0x9C};

float calibrateACS712() {
  Serial.println("Calibrating ACS712: Ensure no current is flowing.");
  delay(2000);
  const int samples = 100;
  long adcSum = 0;
  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(ACS712_PIN);
    delay(10);
  }
  int adcValue = adcSum / samples;
  float zeroVoltage = (adcValue / 4095.0) * 3.3;
  Serial.print("Calibrated ZERO_CURRENT_VOLTAGE: ");
  Serial.println(zeroVoltage);
  return zeroVoltage;
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for Serial to stabilize
  Serial.println("Starting Receiver Setup (Relay/ACS712/Voltage Divider/Fault Pin)...");

  // Initialize pins
  Serial.println("Initializing pins...");
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(ACS712_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(FAULT_PIN, INPUT); // Initialize D2 as input
  Serial.println("Pins initialized.");

  // Calibrate ACS712
  ZERO_CURRENT_VOLTAGE = calibrateACS712();

  // Test WiFi module
  Serial.println("Attempting to set WiFi mode to STA...");
  WiFi.disconnect(); // Ensure no prior WiFi connection
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(1000); // Wait for WiFi interface to stabilize
  Serial.println("WiFi mode set to STA.");

  Serial.print("Receiver MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  Serial.println("Initializing ESP-NOW...");
  esp_err_t initResult = esp_now_init();
  if (initResult != ESP_OK) {
    Serial.print("Error initializing ESP-NOW: ");
    Serial.println(esp_err_to_name(initResult));
    Serial.println("Continuing without ESP-NOW...");
    return;
  }
  Serial.println("ESP-NOW initialized.");

  // Register transmitter as peer
  Serial.println("Registering transmitter as peer...");
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); // Clear peerInfo
  memcpy(peerInfo.peer_addr, transmitterMacAddress, 6);
  peerInfo.channel = 1; // Use channel 1 to reduce interference
  peerInfo.encrypt = false;
  esp_err_t peerResult = esp_now_add_peer(&peerInfo);
  if (peerResult != ESP_OK) {
    Serial.print("Failed to add transmitter as peer. Error: ");
    Serial.println(esp_err_to_name(peerResult));
    return;
  }
  Serial.println("Transmitter peer registered.");

  // Register callbacks
  Serial.println("Registering callbacks...");
  esp_err_t cbResult = esp_now_register_recv_cb(OnDataRecv);
  if (cbResult != ESP_OK) {
    Serial.print("Error registering receive callback: ");
    Serial.println(esp_err_to_name(cbResult));
    return;
  }
  cbResult = esp_now_register_send_cb(OnDataSent);
  if (cbResult != ESP_OK) {
    Serial.print("Error registering send callback: ");
    Serial.println(esp_err_to_name(cbResult));
    return;
  }
  Serial.println("Setup complete.");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Data sent to: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " - Delivery Success" : " - Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  if (len != sizeof(struct_message)) {
    Serial.println("Received data size mismatch!");
    return;
  }
  memcpy(&motorData, data, sizeof(struct_message));
  Serial.print("Data received from: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(recv_info->src_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.print("Received motorCommand: ");
  Serial.println(motorData.motorCommand);
  
  if (motorData.motorCommand) {
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Motor ON. Monitoring...");
    monitorMotor();
  } else {
    digitalWrite(RELAY_PIN, LOW);
    motorData.status = 0;
    motorData.voltage = 0;
    motorData.current = 0;
    Serial.println("Motor OFF. Sending status...");
    esp_now_send(transmitterMacAddress, (uint8_t *) &motorData, sizeof(motorData));
  }
}

float readVoltage() {
  int adcValue = analogRead(VOLTAGE_PIN);
  if (adcValue == 0) {
    Serial.println("Warning: Voltage ADC reading is 0. Check wiring.");
  }
  float voltage = (adcValue / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

float readCurrent() {
  const int samples = 10;
  long adcSum = 0;
  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(ACS712_PIN);
    delay(1); // Small delay between readings
  }
  int adcValue = adcSum / samples;
  if (adcValue == 0) {
    Serial.println("Warning: Current ADC reading is 0. Check ACS712 wiring.");
  }
  float voltage = (adcValue / 4095.0) * 3.3;
  float current = ((voltage - ZERO_CURRENT_VOLTAGE) / ACS712_SENSITIVITY);
  return abs(current);
}

void monitorMotor() {
  motorData.voltage = readVoltage();
  motorData.current = readCurrent();
  Serial.print("Voltage: ");
  Serial.print(motorData.voltage);
  Serial.print("V, Current: ");
  Serial.print(motorData.current);
  Serial.println("A");

  if (digitalRead(FAULT_PIN) == HIGH) {
    motorData.status = 6; // Motor faulty
   // digitalWrite(RELAY_PIN, LOW);
    Serial.println("Motor Faulty detected (D2 HIGH).");
  } else if (motorData.voltage == 0) {
    motorData.status = 5; // No voltage
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("No Voltage detected.");
  } else if (motorData.voltage < MIN_VOLTAGE) {
    motorData.status = 4; // Undervoltage
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Under Voltage detected.");
  } else if (motorData.voltage > 0 && motorData.current < 0.01) {
    motorData.status = 3; // Line break
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Line Break detected.");
  } else if (motorData.current > MAX_CURRENT) {
    motorData.status = 1; // Overload
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Overload detected.");
  } else if (motorData.current < MIN_CURRENT) {
    motorData.status = 2; // Dry run
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Dry Run detected.");
  } else {
    motorData.status = 0; // Normal
    Serial.println("Motor running normally.");
  }

  Serial.println("Sending status to transmitter...");
  esp_now_send(transmitterMacAddress, (uint8_t *) &motorData, sizeof(motorData));
}

void loop() {
  if (motorData.motorCommand) {
    monitorMotor(); // Continue monitoring if motor is commanded ON
  } else {
    Serial.println("Loop running...");
    delay(5000); // Print periodically when motor is OFF
  }
}