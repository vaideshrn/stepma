
#include <Wire.h>

// Supported sensor addresses
#define MPU6050_ID 0x68
#define MPU6500_ID 0x70

byte MPU_ADDR = 0;
byte sensorID = 0;

// Printing thresholds
#define ACCEL_THRESHOLD 0.03
#define GYRO_THRESHOLD  2.0

float oldAX = 999, oldAY = 999, oldAZ = 999;
float oldGX = 999, oldGY = 999, oldGZ = 999;


// ---------------- REGISTER WRITE ----------------

bool writeRegister(byte reg, byte value) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}


// ---------------- REGISTER READ ----------------

byte readRegister(byte reg) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  if (Wire.requestFrom(MPU_ADDR, (byte)1) != 1) {
    return 0xFF;
  }

  return Wire.read();
}


// ---------------- SENSOR DETECTION ----------------

bool detectSensor() {

  byte addresses[] = {0x68, 0x69};

  for (byte i = 0; i < 2; i++) {

    MPU_ADDR = addresses[i];

    Wire.beginTransmission(MPU_ADDR);

    if (Wire.endTransmission() != 0) {
      continue;
    }

    sensorID = readRegister(0x75);

    if (sensorID == MPU6050_ID ||
        sensorID == MPU6500_ID) {

      return true;
    }
  }

  return false;
}


// ---------------- SETUP ----------------

void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);

  delay(500);

  Serial.println("\nUNIVERSAL MPU SENSOR");

  if (!detectSensor()) {

    Serial.println("SUPPORTED MPU NOT FOUND!");

    while (1);
  }

  Serial.print("I2C ADDRESS: 0x");
  Serial.println(MPU_ADDR, HEX);

  Serial.print("WHO_AM_I: 0x");
  Serial.println(sensorID, HEX);

  if (sensorID == MPU6050_ID) {
    Serial.println("SENSOR DETECTED: MPU6050");
  }

  else if (sensorID == MPU6500_ID) {
    Serial.println("SENSOR DETECTED: MPU6500");
  }

  // Wake up sensor
  if (!writeRegister(0x6B, 0x00)) {
    Serial.println("POWER CONFIGURATION FAILED!");
    while (1);
  }

  delay(100);

  // Configure accelerometer: +/- 2g
  writeRegister(0x1C, 0x00);

  // Configure gyroscope: +/- 250 degrees/sec
  writeRegister(0x1B, 0x00);

  Serial.println("SENSOR INITIALIZED!");
}


// ---------------- MAIN LOOP ----------------

void loop() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return;
  }

  if (Wire.requestFrom(MPU_ADDR, (byte)14) != 14) {
    return;
  }

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();

  int16_t temp = (Wire.read() << 8) | Wire.read();

  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();


  // Convert accelerometer to g
  float AX = rawAX / 16384.0;
  float AY = rawAY / 16384.0;
  float AZ = rawAZ / 16384.0;

  // Convert gyroscope to degrees/second
  float GX = rawGX / 131.0;
  float GY = rawGY / 131.0;
  float GZ = rawGZ / 131.0;


  // Detect significant changes
  bool accelChanged =
    abs(AX - oldAX) >= ACCEL_THRESHOLD ||
    abs(AY - oldAY) >= ACCEL_THRESHOLD ||
    abs(AZ - oldAZ) >= ACCEL_THRESHOLD;

  bool gyroChanged =
    abs(GX - oldGX) >= GYRO_THRESHOLD ||
    abs(GY - oldGY) >= GYRO_THRESHOLD ||
    abs(GZ - oldGZ) >= GYRO_THRESHOLD;


  if (accelChanged || gyroChanged) {

    Serial.println("\n-------- SENSOR DATA --------");

    Serial.print("ACCEL X: ");
    Serial.print(AX, 2);

    Serial.print(" | Y: ");
    Serial.print(AY, 2);

    Serial.print(" | Z: ");
    Serial.print(AZ, 2);

    Serial.println(" g");


    Serial.print("GYRO X: ");
    Serial.print(GX, 2);

    Serial.print(" | Y: ");
    Serial.print(GY, 2);

    Serial.print(" | Z: ");
    Serial.print(GZ, 2);

    Serial.println(" deg/s");


    oldAX = AX;
    oldAY = AY;
    oldAZ = AZ;

    oldGX = GX;
    oldGY = GY;
    oldGZ = GZ;
  }

  delay(50);
}
