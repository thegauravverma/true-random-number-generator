#include <Arduino.h>
#include <Wire.h>
#include "heltec.h"
#include <ESP32Servo.h>

// ------------------- SERVO ----------------------
Servo myServo;
const int SERVO_PIN = 5;

// ------------------- LSM6DSO --------------------
#define LSM6DSO_I2C_ADDR    0x6B
#define LSM6DSO_WHO_AM_I    0x0F
#define LSM6DSO_CTRL1_XL    0x10
#define LSM6DSO_OUTX_L_XL   0x28

// ------------------- LED ----------------------
const int LED_PIN = 21;  // GPIO 21 for PWM
int ledBrightness = 0;

// ------------------- PHOTORESISTOR ----------------------
const int PHOTO_PIN = 7; // GPIO 7 for analog input
int lightLevel = 0;


class LSM6DSO {
public:
  LSM6DSO() {}
  bool begin(TwoWire &wirePort);
  void readAccel(float& x, float& y, float& z);
private:
  void writeRegister(uint8_t reg, uint8_t value);
  void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
  TwoWire* _i2c;
  float _accel_mg_lsb;
};

bool LSM6DSO::begin(TwoWire &wirePort) {
  _i2c = &wirePort;
  uint8_t who = 0;
  readRegisters(LSM6DSO_WHO_AM_I, &who, 1);
  if (who != 0x6C) return false;

  writeRegister(LSM6DSO_CTRL1_XL, 0x48); // 104 Hz, 4g
  _accel_mg_lsb = 0.122f;
  return true;
}

void LSM6DSO::readAccel(float& x, float& y, float& z) {
  uint8_t buffer[6];
  readRegisters(LSM6DSO_OUTX_L_XL, buffer, 6);

  int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);

  x = ax * _accel_mg_lsb / 1000.0f;
  y = ay * _accel_mg_lsb / 1000.0f;
  z = az * _accel_mg_lsb / 1000.0f;
}

void LSM6DSO::writeRegister(uint8_t reg, uint8_t value) {
  _i2c->beginTransmission(LSM6DSO_I2C_ADDR);
  _i2c->write(reg);
  _i2c->write(value);
  _i2c->endTransmission();
}

void LSM6DSO::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
  _i2c->beginTransmission(LSM6DSO_I2C_ADDR);
  _i2c->write(reg);
  _i2c->endTransmission(false);
  _i2c->requestFrom((uint8_t)LSM6DSO_I2C_ADDR, len);
  for (uint8_t i = 0; i < len; i++) buffer[i] = _i2c->read();
}

// ------------------- TWO-WIRE --------------------
TwoWire I2C_LSM = TwoWire(1);
LSM6DSO myIMU;
#define I2C_SDA 41
#define I2C_SCL 42

// ------------------- SETUP ------------------------
void setup() {
  Heltec.begin(true, false, true);  // OLED ON, LoRa OFF, Serial ON
  Serial.begin(115200);

  // Servo setup
  myServo.attach(SERVO_PIN);
  ledcSetup(2, 5000, 8);   // Use Channel 2
  ledcAttachPin(LED_PIN, 2);
   analogReadResolution(12);
  // OLED init message
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Servo + IMU Ready");
  Heltec.display->drawString(0, 12, "Init I2C IMU...");
  Heltec.display->display();

  // Start custom I2C bus
  I2C_LSM.begin(I2C_SDA, I2C_SCL, 400000);

  if (!myIMU.begin(I2C_LSM)) {
    Serial.println("Failed to detect LSM6DSO!");
    Heltec.display->drawString(0, 24, "IMU ERROR");
    Heltec.display->display();
    while (1);
  }

  Serial.println("LSM6DSO OK!");
  Heltec.display->drawString(0, 24, "IMU OK!");
  Heltec.display->display();
  delay(1000);
}

// ------------------- HELPER FUNCTION -------------------
// Move servo smoothly to target angle at a "speed" (ms delay between steps)
void moveServoSmooth(int current, int target, int stepDelay) {
  if (current == target) return;

  int step = (current < target) ? 1 : -1;
  for (int pos = current; pos != target; pos += step) {
    myServo.write(pos);
    delay(stepDelay);
  }
  myServo.write(target); // final position
}

// ------------------- LOOP ------------------------
void loop() {

  static int currentAngle = 90;  // start from mid-position
  int targetAngle = random(0, 181); // random target
  int randomSpeed = random(5, 20);  // delay in ms per step → lower = faster

  moveServoSmooth(currentAngle, targetAngle, randomSpeed);
  currentAngle = targetAngle;
  ledBrightness = random(0, 256);
  ledcWrite(2, ledBrightness); // Write to Channel 2
  lightLevel = analogRead(PHOTO_PIN);

  // Read IMU accelerometer
  float ax, ay, az;
  myIMU.readAccel(ax, ay, az);
  
  Serial.print("Angle: "); Serial.print(currentAngle);
  Serial.print(" | SpeedDelay: "); Serial.print(randomSpeed); Serial.print("ms");
  Serial.print(" | AX: "); Serial.print(ax);
  Serial.print(" AY: "); Serial.print(ay);
  Serial.print(" AZ: "); Serial.println(az);
  Serial.print(" | LED: "); Serial.print(ledBrightness);
  Serial.print(" | Light: "); Serial.println(lightLevel);

  // OLED display
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Angle: " + String(currentAngle));
  Heltec.display->drawString(0, 12, "SpeedDelay: " + String(randomSpeed) + "ms");
  Heltec.display->drawString(0, 24, "AX:" + String(ax));
  Heltec.display->drawString(0, 36, "AY:" + String(ay));
  Heltec.display->drawString(0, 48, "AZ:" + String(az));
  Heltec.display->display();

  delay(500); // small pause before next random move
}
