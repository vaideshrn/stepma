
#include <Wire.h>
#include <math.h>

// ---------------- PIN CONFIGURATION ----------------

#define PIEZO_PIN   4
#define FSR_PIN     34
#define BATTERY_PIN 35

#define SDA_PIN     21
#define SCL_PIN     22

// ---------------- MPU6500 CONFIGURATION ----------------

#define MPU_ADDR       0x68
#define WHO_AM_I_REG   0x75
#define EXPECTED_ID    0x70

#define PWR_MGMT_1     0x6B
#define CONFIG_REG     0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B

// ---------------- BATTERY ----------------

const float R1 = 10000.0;
const float R2 = 1000.0;

const float BATTERY_EMPTY = 3.30;
const float BATTERY_FULL = 4.20;

// ---------------- STEP DETECTION ----------------

int stepCount = 0;

float previousMagnitude = 0;

unsigned long lastStepTime = 0;

#define STEP_THRESHOLD 1.25
#define STEP_INTERVAL 300

// ---------------- PIEZO ----------------

int piezoBaseline = 0;

// ---------------- MPU6500 VARIABLES ----------------

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

float magnitude;
float dynamicAccel;

uint8_t mpuAddress = MPU_ADDR;

// ---------------- I2C FUNCTIONS ----------------

bool writeMPURegister(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readMPURegisters(uint8_t reg, uint8_t count,
                      uint8_t *data) {

  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received =
      Wire.requestFrom(mpuAddress, count);

  if (received != count) {
    return false;
  }

  for (int i = 0; i < count; i++) {
    data[i] = Wire.read();
  }

  return true;
}

// ---------------- MPU6500 INITIALIZATION ----------------

bool initializeMPU6500() {

  // Check both possible I2C addresses

  for (uint8_t address : {0x68, 0x69}) {

    mpuAddress = address;

    uint8_t id = 0;

    if (readMPURegisters(
          WHO_AM_I_REG, 1, &id)) {

      Serial.print("WHO_AM_I: 0x");
      Serial.println(id, HEX);

      if (id == EXPECTED_ID) {

        Serial.println("MPU6500 DETECTED!");

        // Wake up sensor
        writeMPURegister(PWR_MGMT_1, 0x01);

        delay(100);

        // Digital low pass filter
        writeMPURegister(CONFIG_REG, 0x03);

        // Gyroscope range: +/-500 degrees/sec
        writeMPURegister(GYRO_CONFIG, 0x08);

        // Accelerometer range: +/-8g
        writeMPURegister(ACCEL_CONFIG, 0x10);

        delay(100);

        return true;
      }
    }
  }

  return false;
}

// ---------------- MPU6500 DATA READING ----------------

bool readMPU6500() {

  uint8_t data[14];

  if (!readMPURegisters(
        ACCEL_XOUT_H, 14, data)) {
    return false;
  }

  int16_t rawAX =
      (int16_t)((data[0] << 8) | data[1]);

  int16_t rawAY =
      (int16_t)((data[2] << 8) | data[3]);

  int16_t rawAZ =
      (int16_t)((data[4] << 8) | data[5]);

  int16_t rawGX =
      (int16_t)((data[8] << 8) | data[9]);

  int16_t rawGY =
      (int16_t)((data[10] << 8) | data[11]);

  int16_t rawGZ =
      (int16_t)((data[12] << 8) | data[13]);

  // Convert acceleration to m/s^2
  // +/-8g range: 4096 LSB/g

  accelX = (rawAX / 4096.0) * 9.80665;
  accelY = (rawAY / 4096.0) * 9.80665;
  accelZ = (rawAZ / 4096.0) * 9.80665;

  // Convert gyroscope to degrees/sec
  // +/-500 dps range: 65.5 LSB/(degree/sec)

  gyroX = rawGX / 65.5;
  gyroY = rawGY / 65.5;
  gyroZ = rawGZ / 65.5;

  return true;
}

// ---------------- SETUP ----------------

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);

  analogSetPinAttenuation(PIEZO_PIN, ADC_11db);
  analogSetPinAttenuation(FSR_PIN, ADC_11db);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  // Initialize I2C

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(100000);

  delay(1000);

  Serial.println("SHOE SENSOR SYSTEM STARTING");

  // Initialize MPU6500

  if (!initializeMPU6500()) {

    Serial.println("MPU6500 NOT FOUND!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6500 CONNECTED");

  // Calibrate piezo baseline

  long total = 0;

  for (int i = 0; i < 200; i++) {

    total += analogRead(PIEZO_PIN);

    delay(5);
  }

  piezoBaseline = total / 200;

  Serial.print("PIEZO BASELINE: ");
  Serial.println(piezoBaseline);

  Serial.println("ALL SENSORS READY");
}

// ---------------- MAIN LOOP ----------------

void loop() {

  // Read FSR

  int fsrValue = analogRead(FSR_PIN);

  // Read Piezo

  int piezoValue = analogRead(PIEZO_PIN);

  int impact = abs(
    piezoValue - piezoBaseline
  );

  // Read MPU6500

  if (!readMPU6500()) {

    Serial.println("MPU6500 READ ERROR");

    delay(100);

    return;
  }

  // Acceleration magnitude

  magnitude = sqrt(
    accelX * accelX +
    accelY * accelY +
    accelZ * accelZ
  );

  // Dynamic acceleration magnitude

  dynamicAccel = abs(
    magnitude - 9.80665
  );

  // MPU6500 step detection

  if (
    dynamicAccel > STEP_THRESHOLD &&
    millis() - lastStepTime > STEP_INTERVAL
  ) {

    stepCount++;

    lastStepTime = millis();

    Serial.println("STEP DETECTED");
  }

  // Update piezo baseline when quiet

  if (impact < 40) {

    piezoBaseline =
      (piezoBaseline * 99 + piezoValue) / 100;
  }

  // Battery voltage

  int batteryRaw = analogRead(BATTERY_PIN);

  float adcVoltage =
    (batteryRaw / 4095.0) * 3.3;

  float batteryVoltage =
    adcVoltage * (R1 + R2) / R2;

  float batteryPercentage =
    (batteryVoltage - BATTERY_EMPTY) /
    (BATTERY_FULL - BATTERY_EMPTY) * 100.0;

  batteryPercentage = constrain(
    batteryPercentage, 0, 100
  );

  // ---------------- SERIAL OUTPUT ----------------

  Serial.print("PIEZO:");
  Serial.print(piezoValue);

  Serial.print(",FSR:");
  Serial.print(fsrValue);

  Serial.print(",ACCEL:");
  Serial.print(magnitude, 2);

  Serial.print(",DYNAMIC:");
  Serial.print(dynamicAccel, 2);

  Serial.print(",ACCEL_X:");
  Serial.print(accelX, 2);

  Serial.print(",ACCEL_Y:");
  Serial.print(accelY, 2);

  Serial.print(",ACCEL_Z:");
  Serial.print(accelZ, 2);

  Serial.print(",GYRO_X:");
  Serial.print(gyroX, 2);

  Serial.print(",GYRO_Y:");
  Serial.print(gyroY, 2);

  Serial.print(",GYRO_Z:");
  Serial.print(gyroZ, 2);

  Serial.print(",BATTERY:");
  Serial.print(batteryVoltage, 2);

  Serial.print(",BATTERY_PERCENT:");
  Serial.print(batteryPercentage, 1);

  Serial.print(",STEPS:");
  Serial.println(stepCount);

  delay(20);
}
