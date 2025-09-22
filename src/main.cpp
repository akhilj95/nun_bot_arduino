#include <Arduino.h>
#include <Wire.h>
#include <BnrOmni.h>

BnrOmni omni;

#define OMNI3MD_ADDRESS 0x30
#define BAUDRATE 115200  // 9600, 19200, 38400, 57600, 115200

// Sensors and buttons pins
const int irProximityPins[5] = {14, 15, 16, 18, 19};
const int irDistancePins[2] = {A6, A7};
const int voltagePins[2] = {A12, A14};
const int buttonPairingPin = 28;
const int buttonOnOffPin = 30;

// Variables to store sensor/button states
int irProximityVals[5];
uint16_t irDistanceVals[2];
float voltage[2];
bool buttonPairingState = false;
bool buttonOnOffState = false;
int enc1, enc2, enc3;
int prevEnc1 = 0, prevEnc2 = 0, prevEnc3 = 0;
unsigned long prevEncTime = 0;
float vel1 = 0.0, vel2 = 0.0, vel3 = 0.0;

unsigned long lastReadTime = 0;
unsigned long lastSendTime = 0;
const unsigned long readInterval = 50; // For IR and Battery readings
const unsigned long sendInterval = 30; // For encoder reading and ROS sending

const float counts_per_revolution = 2880; // Wheel encoder counts per revolution

unsigned long last_command_time = 0;
const unsigned long command_timeout_ms = 5000;  // Stop if no command in 5s

// Global flag to track if the robot is stopped
bool is_stopped = true;

// Parse velocity commands from ROS node, expected format: "[LIN:x,ROT:y,DIR:z]"
void handleCommand(String cmd) {
  int linSpeed = 0, rotSpeed = 0, direction = 0;
  if (cmd.length() == 0) {
    Serial.println("Warning: empty command received, ignoring.");
    return;  // Ignore empty string
  }
  
  int linIndex = cmd.indexOf("LIN:");
  int rotIndex = cmd.indexOf("ROT:");
  int dirIndex = cmd.indexOf("DIR:");

  // Check if all fields exist and are in a valid order
  if (linIndex == -1 || rotIndex == -1 || dirIndex == -1 || 
      linIndex > rotIndex || rotIndex > dirIndex) {
    Serial.print("Warning: malformed command received: ");
    Serial.println(cmd);
    return;  // Ignore malformed commands
  }

  int linEnd = cmd.indexOf(',', linIndex);
  int rotEnd = cmd.indexOf(',', rotIndex);
  int dirEnd = cmd.indexOf(']', dirIndex);

  if (linEnd == -1 || rotEnd == -1 || dirEnd == -1) {
    Serial.print("Warning: incomplete command fields: ");
    Serial.println(cmd);
    return;  // Ignore incomplete
  }

  linSpeed = cmd.substring(linIndex + 4, linEnd).toInt();
  rotSpeed = cmd.substring(rotIndex + 4, rotEnd).toInt();
  direction = cmd.substring(dirIndex + 4, dirEnd).toInt();

  // Constrain values properly
  linSpeed = constrain(linSpeed, 0, 100);
  rotSpeed = constrain(rotSpeed, -100, 100);

  // Direction should be a valid angle 0-360, or default to 0
  if (direction < 0 || direction > 360) {
    direction = 0;
  }

  // Sending the movement commad
  omni.movOmni((byte)linSpeed, rotSpeed, direction);

  last_command_time = millis();

  // Set stopped flag false because new command received and movement issued
  if (is_stopped) {
    is_stopped = false;
    Serial.println("Robot restarted after timeout.");
  }
}

//Calculation for range and voltage
uint16_t get_gp2d12 (uint16_t value) {
    if (value < 10) value = 10;
    return ((67870.0 / (value - 3.0)) - 40.0);
}    

float get_voltage (uint16_t value) {
    return (((float)value/1024)*4.8*5.0);
}

// Read all sensors and button states
void readSensors() {
  for (int i = 0; i < 5; i++) {
    irProximityVals[i] = digitalRead(irProximityPins[i]);
  }
  for (int i = 0; i < 2; i++) {
    uint16_t analogValue = analogRead(irDistancePins[i]);
    irDistanceVals[i] = get_gp2d12(analogValue);
  }
  for (int i = 0; i < 2; i++) {
    uint16_t analogValue = analogRead(voltagePins[i]);
    voltage[i] = get_voltage(analogValue);
  }
  buttonPairingState = (digitalRead(buttonPairingPin) == LOW);
  buttonOnOffState = (digitalRead(buttonOnOffPin) == LOW);
}

// Angular speed from encoder counts/sec
float angular_velocity(float counts_per_sec){
  return ((counts_per_sec * 2.0 * 3.14159265359) / counts_per_revolution);
}

// Wheel speed calculation from encoder values
void calculateWheelVelocity() {
  unsigned long currentTime = millis();  // Current time in ms
  unsigned long deltaTime = currentTime - prevEncTime;  // ms elapsed

  if (deltaTime == 0) {
    // Avoid division by zero if called too fast
    return;
  }

  // Read current encoder counts
  omni.readEncoders(&enc1, &enc2, &enc3);

  delay(2); // giving enough time for i2c
  
  // Calculate delta counts
  int deltaEnc1 = enc1 - prevEnc1;
  int deltaEnc2 = enc2 - prevEnc2;
  int deltaEnc3 = enc3 - prevEnc3;

  // Update previous counts and time
  prevEnc1 = enc1;
  prevEnc2 = enc2;
  prevEnc3 = enc3;
  prevEncTime = currentTime;

  if (abs(deltaEnc1) > counts_per_revolution || abs(deltaEnc2) > counts_per_revolution ||  
     abs(deltaEnc3) > counts_per_revolution) {
    // Something is wrong to make such a big jump. Ignore reading
    return;
  }

  // Calculate velocity counts per second
  vel1 = angular_velocity((deltaEnc1 * 1000.0) / deltaTime);
  vel2 = angular_velocity((deltaEnc2 * 1000.0) / deltaTime);
  vel3 = angular_velocity((deltaEnc3 * 1000.0) / deltaTime);

}

// Send sensor and encoder data back to ROS node
void sendDataROS() {
  Serial.print("VEL:");
  Serial.print(vel1);
  Serial.print(",");
  Serial.print(vel2);
  Serial.print(",");
  Serial.print(vel3);

  Serial.print(";IRP:");
  for (int i=0; i<5; i++) {
    Serial.print(irProximityVals[i]);
    if (i<4) Serial.print(",");
  }

  Serial.print(";IRD:");
  for (int i=0; i<2; i++) {
    Serial.print(irDistanceVals[i]);
    if (i<1) Serial.print(",");
  }

  Serial.print(";VOL:");
  for (int i=0; i<2; i++) {
    Serial.print(voltage[i]);
    if (i<1) Serial.print(",");
  }

  Serial.print(";BTN:");
  Serial.print(buttonPairingState ? "1" : "0");
  Serial.print(",");
  Serial.print(buttonOnOffState ? "1" : "0");
  Serial.println();
}

void setup() {
  Serial.begin(BAUDRATE);
  Serial.setTimeout(100);
  Wire.begin();
  omni.i2cConnect(OMNI3MD_ADDRESS);
  omni.setI2cTimeout(10);
  omni.setPid(980, 100, 300);
  omni.setRamp(75, 950);
  omni.setMinBat(12.2);
  omni.setEncPrescaler(M1, 0);
  omni.setEncPrescaler(M2, 0);
  omni.setEncPrescaler(M3, 0);
  omni.readEncoders(&enc1, &enc2, &enc3);
  omni.stop();

  // Initialize digital sensor and button pins
  for (int i = 0; i < 5; i++) {
    pinMode(irProximityPins[i], INPUT);
  }
  pinMode(buttonPairingPin, INPUT_PULLUP);
  pinMode(buttonOnOffPin, INPUT_PULLUP);

  prevEncTime = millis();
}

void loop() {
  // Read commands from serial non-blocking
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // Periodic sensor reading
  if (millis() - lastReadTime > readInterval) {
    lastReadTime = millis();
    readSensors();
  }

  // Periodic encoder and ROS update
  if (millis() - lastSendTime > sendInterval) {
    lastSendTime = millis();
    calculateWheelVelocity();
    sendDataROS();
  }
  
  if (millis() - last_command_time > command_timeout_ms) {
    if (!is_stopped) {  // Only stop once when timeout happens
      Serial.print("Stopping because of timeout. Last command was at: ");
      Serial.println(last_command_time);
      omni.stop();
      is_stopped = true;
    }
  }
}