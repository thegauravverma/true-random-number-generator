#include <Arduino.h>
#include <Wire.h>
#include "heltec.h"
#include <ESP32Servo.h>
#include "mbedtls/sha256.h" // CRYPTO LIBRARY

// ------------------- CONFIGURATION ----------------------
const int POOL_SIZE = 512;        // Size of entropy buffer
uint8_t entropyPool[POOL_SIZE];   // The "Mixing Bowl" for random data
int poolIndex = 0;
uint32_t lastTRNG = 0;            // Store the last generated number

// ------------------- SERVO ----------------------
Servo myServo;
const int SERVO_PIN = 5;

// ------------------- LSM6DSO (Your Class) --------------------
#define LSM6DSO_I2C_ADDR    0x6B
#define LSM6DSO_WHO_AM_I    0x0F
#define LSM6DSO_CTRL1_XL    0x10
#define LSM6DSO_OUTX_L_XL   0x28

// ------------------- LED & PHOTO ----------------------
const int LED_PIN = 21;  
const int PHOTO_PIN = 7; 
int lightLevel = 0;

// ------------------- CLASS DEFINITION --------------------
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

// ------------------- GLOBALS --------------------
TwoWire I2C_LSM = TwoWire(1);
LSM6DSO myIMU;
#define I2C_SDA 41
#define I2C_SCL 42

// ------------------- ENTROPY HELPERS -------------------

// 1. Add a single byte to the pool
void addByteToPool(uint8_t b) {
  entropyPool[poolIndex] = b;
  poolIndex++;
  if (poolIndex >= POOL_SIZE) poolIndex = 0; // Wrap around (Circular Buffer)
}

// 2. Add any variable type (float, int, long) to the pool
// This breaks variables down into their raw bytes
template <typename T>
void addVarToPool(T val) {
  const uint8_t* p = (const uint8_t*)(const void*)&val;
  for (size_t i = 0; i < sizeof(T); i++) {
    addByteToPool(p[i]);
  }
}

// ------------------- SETUP ------------------------
void setup() {
  Heltec.begin(true, false, true);
  Serial.begin(115200);

  myServo.attach(SERVO_PIN);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(LED_PIN, 2);
  analogReadResolution(12);

  // Initialize Pool with some startup noise
  for(int i=0; i<POOL_SIZE; i++) entropyPool[i] = analogRead(PHOTO_PIN) & 0xFF;

  // Custom I2C
  I2C_LSM.begin(I2C_SDA, I2C_SCL, 400000);
  if (!myIMU.begin(I2C_LSM)) {
    Serial.println("IMU Error");
    while (1);
  }

  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "TRNG System");
  Heltec.display->drawString(0, 15, "Initializing...");
  Heltec.display->display();
  delay(1000);
}

// ------------------- MOVEMENT & HARVESTING -------------------

// Updated to Harvest Data WHILE moving
void moveServoAndHarvest(int current, int target, int stepDelay) {
  if (current == target) return;

  int step = (current < target) ? 1 : -1;
  
  for (int pos = current; pos != target; pos += step) {
    myServo.write(pos);
    
    // --- HARVEST ENTROPY HERE ---
    // Every step of the servo creates unique vibration and power noise.
    
    // 1. Read Accelerometer (Vibration)
    float ax, ay, az;
    myIMU.readAccel(ax, ay, az);
    addVarToPool(ax);
    addVarToPool(ay);
    addVarToPool(az);

    // 2. Read Light (Shadows & Power Rail Dip)
    // Reading ADC while servo moves captures voltage noise
    int light = analogRead(PHOTO_PIN);
    addVarToPool(light);

    // 3. Read Timing Jitter
    // The exact microsecond this loop runs varies slightly
    unsigned long timeJitter = micros();
    addVarToPool(timeJitter);

    delay(stepDelay); 
  }
  myServo.write(target);
}

// ------------------- HASHING FUNCTION -------------------
uint32_t generateTrueRandomNumber() {
  // 1. Add Monotonic Counter
  // Ensures that even if sensors read identical values, input is unique
  static unsigned long counter = 0;
  counter++;
  addVarToPool(counter);

  // 2. SHA-256 Hashing (The "Whitening" process)
  byte outputBuffer[32]; // SHA256 produces 32 bytes
  
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0); // 0 for SHA-256
  mbedtls_sha256_update(&ctx, entropyPool, POOL_SIZE);
  mbedtls_sha256_finish(&ctx, outputBuffer);
  mbedtls_sha256_free(&ctx);

  // 3. Convert first 4 bytes to 32-bit Integer
  uint32_t number = 0;
  number |= ((uint32_t)outputBuffer[0] << 24);
  number |= ((uint32_t)outputBuffer[1] << 16);
  number |= ((uint32_t)outputBuffer[2] << 8);
  number |= ((uint32_t)outputBuffer[3]);
  
  return number;
}

// ------------------- LOOP ------------------------
void loop() {
  static int currentAngle = 90;
  
  // --- FEEDBACK LOOP ---
  // Use the PREVIOUS true random number to determine the NEXT action.
  // This creates a physical chaotic chain.
  
  int targetAngle;
  int speedDelay;
  
  if (lastTRNG == 0) {
    // First run seed
    targetAngle = random(10, 170);
    speedDelay = 10;
  } else {
    // Derive action from the last TRNG hash
    targetAngle = lastTRNG % 160 + 10; // Keep within 10-170 deg
    speedDelay = (lastTRNG >> 8) % 15 + 5; // Speed between 5-20ms
  }

  // 1. Move and Harvest
  // The servo moves, creating noise. We record that noise.
  moveServoAndHarvest(currentAngle, targetAngle, speedDelay);
  currentAngle = targetAngle;

  // 2. Randomize LED (Actuator perturbation)
  // Changing LED brightness changes current draw, affecting ADC noise
  int ledVal = (lastTRNG >> 16) % 255;
  ledcWrite(2, ledVal); 

  // 3. Generate New TRNG
  // Hash the pool we just filled
  uint32_t newRandom = generateTrueRandomNumber();
  lastTRNG = newRandom;

  // 4. Output Data
  Serial.print("Pool Index: "); Serial.print(poolIndex);
  Serial.print(" | TRNG (Dec): "); Serial.print(newRandom);
  Serial.print(" | TRNG (Hex): "); Serial.println(newRandom, HEX);

  // 5. OLED Display
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "True RNG Active");
  Heltec.display->drawString(0, 20, String(newRandom));
  Heltec.display->drawString(0, 40, "Hex: " + String(newRandom, HEX));
  Heltec.display->display();

  delay(100); 
}