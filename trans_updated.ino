#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin Definitions
#define BUTTON_PIN 4
#define BUZZER_PIN 5

// Receiver ESP32's MAC Address
uint8_t receiverMacAddress[] = {0x8C, 0x4F, 0x00, 0xAB, 0xA3, 0x24};

// ESP-NOW
esp_now_peer_info_t peerInfo;

// Data structure to send/receive
typedef struct struct_message {
  bool motorCommand;
  float voltage;
  float current;
  int status; // 0: normal, 1: overload, 2: dry run, 3: line break, 4: undervoltage, 5: no voltage, 6: motor faulty
} struct_message;

struct_message motorData;
struct_message incomingData;

// Variables
bool buttonState = false;
bool lastButtonState = false;
unsigned long buttonPressTime = 0;
bool longPressDetected = false;
bool motorRunning = false;
unsigned long lastDataReceived = 0;
const unsigned long DATA_TIMEOUT = 10000; // 10 seconds timeout
int lastStatus = -1; // Track last displayed status

bool readButton() {
  static bool lastState = HIGH;
  static unsigned long lastChange = 0;
  bool currentState = digitalRead(BUTTON_PIN);
  if (currentState != lastState && millis() - lastChange > 50) {
    lastState = currentState;
    lastChange = millis();
  }
  return currentState == LOW;
}

void alertBuzzer(int status) {
  switch (status) {
    case 1: // Overload
      for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, 1000, 200);
        delay(300);
      }
      break;
    case 2: // Dry run
      for (int i = 0; i < 2; i++) {
        tone(BUZZER_PIN, 1000, 500);
        delay(600);
      }
      break;
    case 3: // Line break
      tone(BUZZER_PIN, 1000, 1000);
      break;
    case 4: // Undervoltage
      for (int i = 0; i < 4; i++) {
        tone(BUZZER_PIN, 1000, 100);
        delay(200);
      }
      break;
    case 5: // No voltage
      tone(BUZZER_PIN, 1000, 1500);
      break;
    case 6: // Motor faulty
      for (int i = 0; i < 5; i++) {
        tone(BUZZER_PIN, 1000, 150);
        delay(250);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for Serial to stabilize
  Serial.println("Starting Transmitter Setup (Button/OLED/Buzzer)...");
  
  // Initialize OLED
  Serial.println("Initializing OLED...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Press button to start");
  display.setCursor(0, 20);
  display.write(0x1B); // Left arrow
  display.display();
  Serial.println("OLED initialized.");

  // Initialize pins
  Serial.println("Initializing pins...");
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("Pins initialized.");

  // Initialize ESP-NOW
  Serial.println("Setting WiFi mode to STA...");
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("Transmitter MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW. Halting...");
    while (1) {
      Serial.println("ESP-NOW init failed.");
      delay(5000);
    }
  }
  Serial.println("ESP-NOW initialized.");

  // Register peer
  Serial.println("Registering peer...");
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 1; // Set to channel 1
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer. Halting...");
    while (1) {
      Serial.println("Peer registration failed.");
      delay(5000);
    }
  }
  Serial.println("Peer registered.");

  // Register callbacks
  Serial.println("Registering callbacks...");
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
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
  Serial.println("Data received from receiver.");
  memcpy(&incomingData, data, sizeof(struct_message));
  lastDataReceived = millis();
  if (lastStatus != incomingData.status) {
    lastStatus = incomingData.status;
    alertBuzzer(lastStatus); // Call buzzer on status change
  }
  Serial.print("Voltage: ");
  Serial.print(incomingData.voltage);
  Serial.print("V, Current: ");
  Serial.print(incomingData.current);
  Serial.print("A, Status: ");
  Serial.println(incomingData.status);
  displayData();
}

void displayData() {
  display.clearDisplay();
  display.setCursor(0,0);
  if (!motorRunning) {
    display.println("Motor OFF");
    display.setCursor(0, 20);
    display.write(0x1B);
  } else if (millis() - lastDataReceived > DATA_TIMEOUT) {
    display.println("No data received!");
    lastStatus = -1;
  } else {
    switch (lastStatus) {
      case 0:
        display.println("Motor Running");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.println("V");
        display.print("I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
      case 1:
        display.println("Motor Overload!");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.print("V, I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
      case 2:
        display.println("Dry Run!");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.print("V, I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
      case 3:
        display.println("Line Break!");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.print("V, I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
      case 4:
        display.println("Under Voltage!");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.print("V, I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
      case 5:
        display.println("No Voltage!");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.print("V, I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
      case 6:
        display.println("Motor Faulty!");
        display.print("V: ");
        display.print(incomingData.voltage);
        display.print("V, I: ");
        display.print(incomingData.current);
        display.println("A");
        break;
    }
  }
  display.display();
}

void loop() {
  // Read button with debouncing
  buttonState = readButton();

  if (buttonState && !lastButtonState) {
    buttonPressTime = millis();
    tone(BUZZER_PIN, 1000, 100);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Long press to start");
    display.setCursor(0, 20);
    display.write(0x1B);
    display.display();
    Serial.println("Button pressed.");
  }

  if (buttonState && (millis() - buttonPressTime > 2000) && !longPressDetected) {
    longPressDetected = true;
    tone(BUZZER_PIN, 1000, 500);
    motorRunning = !motorRunning;
    motorData.motorCommand = motorRunning;
    
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Waiting...");
    display.display();
    Serial.println("Long press detected. Sending motor command...");
    
    Serial.print("Sending to receiver MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(receiverMacAddress[i], HEX);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
    esp_now_send(receiverMacAddress, (uint8_t *) &motorData, sizeof(motorData));
  }

  if (!buttonState) {
    longPressDetected = false;
  }

  // Check for timeout
  if (motorRunning && millis() - lastDataReceived > DATA_TIMEOUT) {
    displayData(); // Update OLED with timeout message
  }

  lastButtonState = buttonState;
  delay(10); // Small delay for stability
}