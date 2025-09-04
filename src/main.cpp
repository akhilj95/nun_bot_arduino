#include <Arduino.h>

// Digital sensor pins
const int digitalPins[6] = {14, 15, 16, 18, 19, 20};
// Analog sensor pins
const int analogPins[2] = {A6, A7};
// Button pins
const int buttonPairing = 28;
const int buttonOnOff = 30;


uint16_t get_gp2d12 (uint16_t value) {
    if (value < 10) value = 10;
    return ((67870.0 / (value - 3.0)) - 40.0);
}

void setup() {
  Serial.begin(115200);
  // Initialize digital pins as input
  for (int i = 0; i < 6; i++) {
    pinMode(digitalPins[i], INPUT);
  }
  // Enable internal pull-up resistors for buttons
  pinMode(buttonPairing, INPUT_PULLUP); // Spring loaded button for BLE pairing
  pinMode(buttonOnOff, INPUT_PULLUP); // Button for turning On and Off something
}

void loop() {
  Serial.println("Digital Sensor Readings:");
  for (int i = 0; i < 6; i++) {
    int digitalValue = digitalRead(digitalPins[i]);
    Serial.print("Digital pin ");
    Serial.print(digitalPins[i]);
    Serial.print(": ");
    Serial.println(digitalValue);
  }

  Serial.println("Analog Sensor Readings:");
  for (int i = 0; i < 2; i++) {
    uint16_t analogValue = analogRead(analogPins[i]);
    uint16_t range = get_gp2d12 (analogValue);
    Serial.print("Analog pin ");
    Serial.print(analogPins[i]);
    Serial.print(": ");
    Serial.println(range);
  }

  int buttonState = digitalRead(buttonPairing);
  Serial.print("Button on pin ");
  Serial.print(buttonPairing);
  if (buttonState == LOW) {
    Serial.println(" is PRESSED");
  } else {
    Serial.println(" is RELEASED");
  }

  buttonState = digitalRead(buttonOnOff);
  Serial.print("Button on pin ");
  Serial.print(buttonOnOff);
  if (buttonState == LOW) {
    Serial.println(" is On");
  } else {
    Serial.println(" is Off");
  }
  
  Serial.println("-----------------------------");
  delay(500);
}