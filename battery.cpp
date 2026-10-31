
#define BATTERY_PIN A0

const float ADC_REFERENCE = 5.0;
const float DIVIDER_RATIO = 1.1;

float readBatteryVoltage() {
  long total = 0;

  for (int i = 0; i < 30; i++) {
    total += analogRead(BATTERY_PIN);
    delay(5);
  }

  float adcValue = total / 30.0;

  float voltage = adcValue * ADC_REFERENCE / 1023.0;

  return voltage * DIVIDER_RATIO;
}

int calculatePercentage(float voltage) {

  if (voltage >= 4.20) return 100;
  if (voltage <= 3.30) return 0;

  return constrain(
    (int)((voltage - 3.30) * 100.0 / 0.90),
    0, 100
  );
}

void setup() {
  Serial.begin(9600);
}

void loop() {

  float batteryVoltage = readBatteryVoltage();

  int percentage = calculatePercentage(batteryVoltage);

  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");

  Serial.print("Battery Percentage: ");
  Serial.print(percentage);
  Serial.println(" %");

  Serial.println("-------------------");

  delay(1000);
}
