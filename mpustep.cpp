
#include <Wire.h>
#include <math.h>

#define MPU_ADDR_1 0x68
#define MPU_ADDR_2 0x69

// Step detection settings
#define STEP_THRESHOLD 0.10
#define RESET_THRESHOLD 0.035
#define MIN_STEP_INTERVAL 450

byte mpuAddress = 0;
byte sensorID = 0;

int stepCount = 0;

float magnitude = 0;
float filteredMagnitude = 0;
float gravity = 1.0;
float movement = 0;

bool stepArmed = true;

unsigned long lastStepTime = 0;


// Read a register
byte readRegister(byte reg) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  if (Wire.requestFrom(mpuAddress, (byte)1) != 1) {
    return 0xFF;
  }

  return Wire.read();
}


// Detect MPU6050 or MPU6500
bool detectMPU() {
  byte addresses[] = {MPU_ADDR_1, MPU_ADDR_2};

  for (byte i = 0; i < 2; i++) {
    mpuAddress = addresses[i];

    Wire.beginTransmission(mpuAddress);

    if (Wire.endTransmission() != 0) {
      continue;
    }

    sensorID = readRegister(0x75);

    if (sensorID == 0x68 || sensorID == 0x70) {
      return true;
    }
  }

  return false;
}


// Read accelerometer
bool readAcceleration(float &result) {

  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(mpuAddress, (byte)6) != 6) {
    return false;
  }

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();

  float ax = rawAX / 16384.0;
  float ay = rawAY / 16384.0;
  float az = rawAZ / 16384.0;

  result = sqrt(ax * ax + ay * ay + az * az);

  return true;
}


void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);

  delay(500);

  Serial.println("STRIDEFUSIONX STEP DETECTOR");

  if (!detectMPU()) {
    Serial.println("MPU SENSOR NOT FOUND!");
    while (1);
  }

  Serial.print("SENSOR: ");

  if (sensorID == 0x68) {
    Serial.println("MPU6050");
  } else {
    Serial.println("MPU6500");
  }

  // Wake up sensor
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Accelerometer range +/- 2g
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(300);

  // Initialize filters from actual sensor reading
  if (readAcceleration(magnitude)) {
    gravity = magnitude;
    filteredMagnitude = magnitude;
  }

  Serial.println("STEP DETECTION READY!");
}


void loop() {

  if (!readAcceleration(magnitude)) {
    return;
  }

  // Smooth the acceleration magnitude
  filteredMagnitude =
      0.3 * magnitude +
      0.7 * filteredMagnitude;

  // Estimate slowly changing gravity component
  gravity =
      0.98 * gravity +
      0.02 * filteredMagnitude;

  // Remove gravity to isolate movement
  movement = filteredMagnitude - gravity;

  unsigned long now = millis();

  // Detect movement peak
  if (stepArmed &&
      movement > STEP_THRESHOLD &&
      now - lastStepTime >= MIN_STEP_INTERVAL) {

    stepCount++;

    lastStepTime = now;
    stepArmed = false;

    Serial.print("STEP DETECTED! TOTAL STEPS: ");
    Serial.println(stepCount);
  }

  // Rearm only after movement settles
  if (!stepArmed && movement < RESET_THRESHOLD) {
    stepArmed = true;
  }

  delay(20);
}
