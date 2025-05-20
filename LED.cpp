#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pin definitions
const int doorSensorPin = 2;    // Door open sensor pin
const int pirSensorPin = 3;     // Motion sensor pin
const int redPin = 9;           // Red LED pin
const int greenPin = 10;        // Green LED pin
const int bluePin = 11;         // Blue LED pin
const int potPin = A0;          // Potentiometer pin
const int buttonOnOff = 6;      // On/off button
const int buttonColor = 7;      // Color change button
const int buttonBrightUp = 8;   // Brightness up button
const int buttonBrightDown = 12;// Brightness down button

// Bluetooth configuration on pins 4 (RX) and 5 (TX)
SoftwareSerial bluetoothSerial(4, 5);

// OLED display parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin (-1 if sharing reset with Arduino)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Parameters
const unsigned long autoOffTime = 30000;  // Auto-off time (30 seconds)
const int fadeSpeed = 5;                  // Fade speed (ms)
const int maxBrightness = 255;            // Maximum brightness

// Variables
unsigned long lastActivityTime = 0;
boolean lightState = false;
int currentRed = 0;
int currentGreen = 0;
int currentBlue = 0;
int targetRed = 255;
int targetGreen = 255;
int targetBlue = 255;
String bluetoothCommand = "";

void setup() {
  pinMode(doorSensorPin, INPUT_PULLUP);
  pinMode(pirSensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buttonOnOff, INPUT_PULLUP);
  pinMode(buttonColor, INPUT_PULLUP);
  pinMode(buttonBrightUp, INPUT_PULLUP);
  pinMode(buttonBrightDown, INPUT_PULLUP);

  Serial.begin(9600);
  bluetoothSerial.begin(9600);

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 initialization failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Lighting System"));
  display.println(F("Status: Ready"));
  display.display();

  Serial.println("Closet lighting system ready");
}

void loop() {
  checkSensors();
  processBluetooth();
  checkButtons();
  readPotentiometer();
  updateLights();
  checkAutoOff();
  updateDisplay();
  delay(10);  // Short pause for stability
}

void checkSensors() {
  // Check door sensor (LOW when door is open using INPUT_PULLUP)
  if (digitalRead(doorSensorPin) == LOW) {
    if (!lightState) {
      Serial.println("Door opened - turning on lights");
      setLightOn();
    }
    lastActivityTime = millis();
  }

  // Check motion sensor
  if (digitalRead(pirSensorPin) == HIGH) {
    if (!lightState) {
      Serial.println("Motion detected - turning on lights");
      setLightOn();
    }
    lastActivityTime = millis();
  }
}

void processBluetooth() {
  while (bluetoothSerial.available()) {
    char c = bluetoothSerial.read();
    if (c == '\n') {
      processCommand(bluetoothCommand);
      bluetoothCommand = "";
    } else {
      bluetoothCommand += c;
    }
  }
}

void checkButtons() {
  static bool lastButtonOnOffState = HIGH;
  static bool lastButtonColorState = HIGH;
  static bool lastButtonBrightUpState = HIGH;
  static bool lastButtonBrightDownState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  // Read button states (LOW when pressed with INPUT_PULLUP)
  bool buttonOnOffState = digitalRead(buttonOnOff);
  bool buttonColorState = digitalRead(buttonColor);
  bool buttonBrightUpState = digitalRead(buttonBrightUp);
  bool buttonBrightDownState = digitalRead(buttonBrightDown);

  // Check on/off button
  if (buttonOnOffState != lastButtonOnOffState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonOnOffState == LOW && lastButtonOnOffState == HIGH) {
      // Button was pressed
      if (lightState) {
        setLightOff();
      } else {
        setLightOn();
      }
    }
  }

  // Check color change button
  if (buttonColorState != lastButtonColorState) {
    if (buttonColorState == LOW && lastButtonColorState == HIGH && lightState) {
      // Cycle through predefined colors
      static int colorPreset = 0;
      colorPreset = (colorPreset + 1) % 4;

      switch (colorPreset) {
        case 0: // White
          targetRed = 255; targetGreen = 255; targetBlue = 255;
          break;
        case 1: // Warm
          targetRed = 255; targetGreen = 180; targetBlue = 100;
          break;
        case 2: // Cool
          targetRed = 180; targetGreen = 200; targetBlue = 255;
          break;
        case 3: // Night
          targetRed = 50; targetGreen = 0; targetBlue = 10;
          break;
      }

      lastActivityTime = millis();
    }
  }

  // Check brightness buttons
  if (buttonBrightUpState == LOW && lightState) {
    // Increase brightness
    float ratio = max(targetRed, max(targetGreen, targetBlue)) / 255.0;
    ratio = min(ratio + 0.05, 1.0);

    if (targetRed > 0) targetRed = min(int(targetRed / (ratio - 0.05) * ratio), 255);
    if (targetGreen > 0) targetGreen = min(int(targetGreen / (ratio - 0.05) * ratio), 255);
    if (targetBlue > 0) targetBlue = min(int(targetBlue / (ratio - 0.05) * ratio), 255);

    lastActivityTime = millis();
  }

  if (buttonBrightDownState == LOW && lightState) {
    // Decrease brightness
    float ratio = max(targetRed, max(targetGreen, targetBlue)) / 255.0;
    ratio = max(ratio - 0.05, 0.0);

    if (targetRed > 0) targetRed = max(int(targetRed / (ratio + 0.05) * ratio), 0);
    if (targetGreen > 0) targetGreen = max(int(targetGreen / (ratio + 0.05) * ratio), 0);
    if (targetBlue > 0) targetBlue = max(int(targetBlue / (ratio + 0.05) * ratio), 0);

    // Turn off light if brightness drops to zero
    if (targetRed == 0 && targetGreen == 0 && targetBlue == 0) {
      lightState = false;
    }

    lastActivityTime = millis();
  }

  // Save previous button states
  lastButtonOnOffState = buttonOnOffState;
  lastButtonColorState = buttonColorState;
  lastButtonBrightUpState = buttonBrightUpState;
  lastButtonBrightDownState = buttonBrightDownState;
}

void readPotentiometer() {
  // Read potentiometer value (0-1023)
  int potValue = analogRead(potPin);

  // Convert to brightness (0-255)
  int brightness = map(potValue, 0, 1023, 0, 255);

  // Apply brightness while maintaining color proportions
  if (lightState) {
    float ratio = brightness / 255.0;

    // Maintain proportions between colors
    float redRatio = (float)targetRed / 255.0;
    float greenRatio = (float)targetGreen / 255.0;
    float blueRatio = (float)targetBlue / 255.0;

    // Set new target values
    targetRed = int(255 * redRatio * ratio);
    targetGreen = int(255 * greenRatio * ratio);
    targetBlue = int(255 * blueRatio * ratio);
  }
}

void processCommand(String command) {
  Serial.print("Command received: ");
  Serial.println(command);

  if (command == "ON") {
    setLightOn();
  }
  else if (command == "OFF") {
    setLightOff();
  }
  else if (command.startsWith("COLOR:")) {
    // Format: COLOR:R,G,B (e.g., COLOR:255,0,0 for red)
    String colorValues = command.substring(6);
    int firstComma = colorValues.indexOf(',');
    int secondComma = colorValues.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > 0) {
      targetRed = colorValues.substring(0, firstComma).toInt();
      targetGreen = colorValues.substring(firstComma + 1, secondComma).toInt();
      targetBlue = colorValues.substring(secondComma + 1).toInt();

      Serial.print("Setting RGB color: ");
      Serial.print(targetRed); Serial.print(",");
      Serial.print(targetGreen); Serial.print(",");
      Serial.println(targetBlue);

      lightState = true;
      lastActivityTime = millis();
    }
  }
  else if (command.startsWith("BRIGHTNESS:")) {
    // Format: BRIGHTNESS:50 (brightness percentage)
    int brightness = command.substring(11).toInt();
    brightness = constrain(brightness, 0, 100);

    // Adjust brightness while maintaining color proportions
    float factor = brightness / 100.0;
    targetRed = int(255 * factor);
    targetGreen = int(255 * factor);
    targetBlue = int(255 * factor);

    Serial.print("Setting brightness: ");
    Serial.println(brightness);

    lightState = (brightness > 0);
    lastActivityTime = millis();
  }
  else if (command.startsWith("PRESET:")) {
    // Predefined colors
    String preset = command.substring(7);

    if (preset == "WARM") {
      targetRed = 255; targetGreen = 180; targetBlue = 100;
    }
    else if (preset == "COOL") {
      targetRed = 180; targetGreen = 200; targetBlue = 255;
    }
    else if (preset == "NIGHT") {
      targetRed = 50; targetGreen = 0; targetBlue = 10;
    }

    Serial.print("Setting preset: ");
    Serial.println(preset);

    lightState = true;
    lastActivityTime = millis();
  }
}

void updateLights() {
  // Smooth transition to target RGB values
  if (currentRed < targetRed) currentRed = min(currentRed + 1, targetRed);
  if (currentRed > targetRed) currentRed = max(currentRed - 1, targetRed);

  if (currentGreen < targetGreen) currentGreen = min(currentGreen + 1, targetGreen);
  if (currentGreen > targetGreen) currentGreen = max(currentGreen - 1, targetGreen);

  if (currentBlue < targetBlue) currentBlue = min(currentBlue + 1, targetBlue);
  if (currentBlue > targetBlue) currentBlue = max(currentBlue - 1, targetBlue);

  analogWrite(redPin, currentRed);
  analogWrite(greenPin, currentGreen);
  analogWrite(bluePin, currentBlue);

  // Short pause for smooth transition effect
  delay(fadeSpeed);
}

void updateDisplay() {
  static unsigned long lastDisplayUpdate = 0;

  // Update display every 500ms
  if (millis() - lastDisplayUpdate > 500) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("Lighting System"));
    display.print(F("Status: "));
    display.println(lightState ? F("ON") : F("OFF"));

    display.println(F("RGB Color:"));
    display.print(currentRed); display.print(F(", "));
    display.print(currentGreen); display.print(F(", "));
    display.println(currentBlue);

    display.print(F("Time: "));
    display.print((millis() - lastActivityTime) / 1000);
    display.println(F("s"));

    display.display();
    lastDisplayUpdate = millis();
  }
}

void checkAutoOff() {
  // Auto-off after inactivity period
  if (lightState && (millis() - lastActivityTime > autoOffTime)) {
    Serial.println("Auto-off after inactivity period");
    setLightOff();
  }
}

void setLightOn() {
  // Default white light
  targetRed = maxBrightness;
  targetGreen = maxBrightness;
  targetBlue = maxBrightness;
  lightState = true;
  lastActivityTime = millis();
}

void setLightOff() {
  targetRed = 0;
  targetGreen = 0;
  targetBlue = 0;
  lightState = false;
}
