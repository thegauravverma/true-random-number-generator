#include <Arduino.h>
#include <Wire.h>
#include "heltec.h"
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// mbedtls for HMAC / base64
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha256.h"

const char* IOT_HUB_HOST = "CS244p.azure-devices.net";   // Hostname (no protocol)
const char* DEVICE_ID = "esp32";                        // Device id
const char* DEVICE_PRIMARY_KEY = "CJ4OT7Cp7W2PNHK+iPNGj8IwNI1dIU7ewy+886TlVS4="; // Base64 primary key

// WiFi creds - move to secrets.h if you prefer
const char* WIFI_SSID = "iPhone";
const char* WIFI_PASSWORD = "123456789";

// Optional override of SAS token (if you want to use a precomputed token instead)
// If non-empty, code will use this token and skip generation.
const char* SAS_TOKEN_OVERRIDE = ""; // e.g. "SharedAccessSignature sr=...&sig=...&se=..."

// NTP server for obtaining time (used to compute expiry for SAS token)
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 0;
const int   DAYLIGHT_OFFSET_SEC = 0;

// Azure IoT API version and resource path used in token/signing
// Resource used for string-to-sign = "<host>/devices/<deviceId>"
String resourceUri() {
  String r = String(IOT_HUB_HOST) + "/devices/" + String(DEVICE_ID);
  return r;
}

// Telemetry endpoint path appended to host
String telemetryPath() {
  // device-to-cloud endpoint
  return String("/devices/") + String(DEVICE_ID) + "/messages/events?api-version=2020-09-30";
}

// ------------------- LOGGING HELPERS --------------------
#define LOG_INFO(tag, msg)     Serial.printf("[%s] %s\n", tag, msg)
#define LOG_WARN(tag, msg)     Serial.printf("[WARN:%s] %s\n", tag, msg)
#define LOG_ERROR(tag, msg)    Serial.printf("[ERROR:%s] %s\n", tag, msg)
#define LOG_DEBUG(tag, msg)    Serial.printf("[DEBUG:%s] %s\n", tag, msg)

// ------------------- TRNG CONFIG (from your sketch) ----------------------
const int POOL_SIZE = 512;        // Size of entropy buffer
uint8_t entropyPool[POOL_SIZE];   // The "Mixing Bowl" for random data
int poolIndex = 0;
uint32_t lastTRNG = 0;            // Store the last generated number

// Servo
Servo myServo;
const int SERVO_PIN = 5;

// LSM6DSO definitions (your minimal driver)
#define LSM6DSO_I2C_ADDR    0x6B
#define LSM6DSO_WHO_AM_I    0x0F
#define LSM6DSO_CTRL1_XL    0x10
#define LSM6DSO_OUTX_L_XL   0x28

// LED & photoresistor
const int LED_PIN = 21;
const int PHOTO_PIN = 7;

// I2C and IMU object
TwoWire I2C_LSM = TwoWire(1);

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
} myIMU;

// ---------- LSM6DSO Implementation ----------
bool LSM6DSO::begin(TwoWire &wirePort) {
  _i2c = &wirePort;
  uint8_t who = 0;
  readRegisters(LSM6DSO_WHO_AM_I, &who, 1);
  if (who != 0x6C) return false;
  writeRegister(LSM6DSO_CTRL1_XL, 0x48); 
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

// ------------------- ENTROPY HELPERS -------------------
void addByteToPool(uint8_t b) {
  entropyPool[poolIndex] = b;
  poolIndex++;
  if (poolIndex >= POOL_SIZE) poolIndex = 0; // Wrap around
}

template <typename T>
void addVarToPool(T val) {
  const uint8_t* p = (const uint8_t*)(const void*)&val;
  for (size_t i = 0; i < sizeof(T); i++) {
    addByteToPool(p[i]);
  }
}

// ------------------- GENERATE & WHITEN -------------------
uint32_t generateTrueRandomNumber() {
  static unsigned long counter = 0;
  counter++;
  addVarToPool(counter);

  // SHA-256 over the full pool
  uint8_t outputBuffer[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, entropyPool, POOL_SIZE);
  mbedtls_sha256_finish(&ctx, outputBuffer);
  mbedtls_sha256_free(&ctx);

  // Take first 4 bytes as big-endian uint32
  uint32_t number = 0;
  number |= ((uint32_t)outputBuffer[0] << 24);
  number |= ((uint32_t)outputBuffer[1] << 16);
  number |= ((uint32_t)outputBuffer[2] << 8);
  number |= ((uint32_t)outputBuffer[3]);
  return number;
}

// ------------------- MOVEMENT & HARVESTING -------------------
void moveServoAndHarvest(int current, int target, int stepDelay) {
  if (current == target) return;
  int step = (current < target) ? 1 : -1;
  for (int pos = current; pos != target; pos += step) {
    myServo.write(pos);

    // Harvest entropy each step
    float ax, ay, az;
    myIMU.readAccel(ax, ay, az);
    addVarToPool(ax); addVarToPool(ay); addVarToPool(az);

    int light = analogRead(PHOTO_PIN);
    addVarToPool(light);

    unsigned long timeJitter = micros();
    addVarToPool(timeJitter);

    delay(stepDelay);
  }
  myServo.write(target);
}

// ------------------- UTIL: URL ENCODE -------------------
String urlEncode(const String &str) {
  String encoded;
  encoded.reserve(str.length());
  for (size_t i = 0; i < str.length(); i++) {
    char c = str[i];
    if ( (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'Z') ||
         (c >= 'a' && c <= 'z') ||
         c == '-' || c == '_' || c == '.' || c == '~' ) {
      encoded += c;
    } else {
      char buf[8];
      sprintf(buf, "%%%02X", (uint8_t)c);
      encoded += buf;
    }
  }
  return encoded;
}


bool base64Decode(const char* b64, uint8_t* out, size_t* outLen) {
  size_t inLen = strlen(b64);
  int ret = mbedtls_base64_decode(out, *outLen, outLen, (const unsigned char*)b64, inLen);
  return (ret == 0);
}

bool base64Encode(const uint8_t* in, size_t inLen, char* out, size_t* outLen) {
  int ret = mbedtls_base64_encode((unsigned char*)out, *outLen, outLen, in, inLen);
  return (ret == 0);
}

String generateSasToken(uint32_t expirySecondsFromNow = 3600) {
  if (strlen(SAS_TOKEN_OVERRIDE) > 0) {
    LOG_INFO("SAS", "Using override SAS token.");
    return String(SAS_TOKEN_OVERRIDE);
  }

  // Ensure time is available; expect NTP sync before calling
  time_t now = time(nullptr);
  if (now < 1600000000) {
    LOG_WARN("SAS", "System time seems incorrect (no NTP). Attempting NTP sync...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    delay(2000);
    now = time(nullptr);
    if (now < 1600000000) {
      LOG_WARN("SAS", "NTP sync failed or time still invalid. SAS expiry may be wrong.");
      // continue; expiry will be relative to whatever time() returns
    }
  }

  uint32_t expiry = (uint32_t)(now + expirySecondsFromNow);
  String resource = resourceUri(); // e.g. host/devices/deviceId
  String resourceEnc = urlEncode(resource);

  // stringToSign = resourceEnc + "\n" + expiry
  String stringToSign = resourceEnc + "\n" + String(expiry);

  // decode primary key (base64)
  size_t keyBufLen = 128;
  uint8_t keyBuf[128];
  size_t keyOutLen = keyBufLen;
  if (!base64Decode(DEVICE_PRIMARY_KEY, keyBuf, &keyOutLen)) {
    LOG_ERROR("SAS", "Base64 decode of primary key failed.");
    return String();
  }

  // HMAC-SHA256
  const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (md_info == nullptr) {
    LOG_ERROR("SAS", "md_info null");
    return String();
  }

  mbedtls_md_context_t md_ctx;
  mbedtls_md_init(&md_ctx);
  if (mbedtls_md_setup(&md_ctx, md_info, 1) != 0) {
    LOG_ERROR("SAS", "mbedtls_md_setup failed");
    mbedtls_md_free(&md_ctx);
    return String();
  }

  if (mbedtls_md_hmac_starts(&md_ctx, keyBuf, keyOutLen) != 0) {
    LOG_ERROR("SAS", "HMAC starts failed");
    mbedtls_md_free(&md_ctx);
    return String();
  }
  if (mbedtls_md_hmac_update(&md_ctx, (const unsigned char*)stringToSign.c_str(), stringToSign.length()) != 0) {
    LOG_ERROR("SAS", "HMAC update failed");
    mbedtls_md_free(&md_ctx);
    return String();
  }
  unsigned char hmac[32];
  if (mbedtls_md_hmac_finish(&md_ctx, hmac) != 0) {
    LOG_ERROR("SAS", "HMAC finish failed");
    mbedtls_md_free(&md_ctx);
    return String();
  }
  mbedtls_md_free(&md_ctx);

  // base64 encode signature
  char b64sig[128];
  size_t b64sigLen = sizeof(b64sig);
  if (!base64Encode(hmac, sizeof(hmac), b64sig, &b64sigLen)) {
    LOG_ERROR("SAS", "Base64 encode of signature failed");
    return String();
  }
  b64sig[b64sigLen] = 0; // null-terminate

  // url-encode signature
  String sigEnc = urlEncode(String(b64sig));

  // Build SAS token string
  String sas = "SharedAccessSignature sr=" + resourceEnc + "&sig=" + sigEnc + "&se=" + String(expiry);
  return sas;
}

// ------------------- HTTP POST to Azure -------------------
bool postTelemetryToIoTHub(const String& sasToken, const char* jsonPayload) {
  WiFiClientSecure client;
  client.setCACert(NULL); // Use system CA bundle - in many ESP32 builds this is fine.
  // If you have a CA PEM, call client.setCACert(AZURE_ROOT_CA);

  HTTPClient http;
  String url = String("https://") + String(IOT_HUB_HOST) + telemetryPath();

  if (!http.begin(client, url)) {
    LOG_ERROR("HTTP", "http.begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", sasToken.c_str());

  LOG_DEBUG("HTTP", ("POST: " + url).c_str());
  int httpCode = http.POST((uint8_t*)jsonPayload, strlen(jsonPayload));

  if (httpCode == 204 || (httpCode >=200 && httpCode < 300)) {
    LOG_INFO("HTTP", "Telemetry accepted (204).");
    http.end();
    return true;
  } else {
    LOG_ERROR("HTTP", ("HTTP fail (" + String(httpCode) + ")").c_str());
    String resp = http.getString();
    LOG_DEBUG("HTTP", ("Body: " + resp).c_str());
    http.end();
    return false;
  }
}

// ------------------- WiFi helper -------------------
bool connectWiFi(uint8_t maxAttempts = 30) {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  LOG_INFO("WIFI", "Connecting...");
  uint8_t attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts) {
    ++attempt;
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    LOG_INFO("WIFI", "Connected.");
    LOG_DEBUG("WIFI", ("IP: " + WiFi.localIP().toString()).c_str());
    return true;
  }
  LOG_ERROR("WIFI", "Failed to connect.");
  return false;
}

// ------------------- Globals for telemetry timing -------------------
unsigned long lastTelemetryTick = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 1000; // send every TRNG cycle (approx every loop)

// ------------------- Setup & Loop -------------------
void setup() {
  Heltec.begin(true, false, true);
  Serial.begin(115200);
  delay(200);

  // Servo, LED, ADC config
  myServo.attach(SERVO_PIN);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(LED_PIN, 2);
  analogReadResolution(12);

  // Init entropy pool with ADC noise
  for (int i = 0; i < POOL_SIZE; i++) {
    entropyPool[i] = analogRead(PHOTO_PIN) & 0xFF;
  }

  // I2C and IMU
  #define I2C_SDA 41
  #define I2C_SCL 42
  I2C_LSM.begin(I2C_SDA, I2C_SCL, 400000);
  if (!myIMU.begin(I2C_LSM)) {
    Serial.println("IMU Error");
    while (1) delay(1000);
  }

  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "TRNG System");
  Heltec.display->drawString(0, 15, "Initializing...");
  Heltec.display->display();
  delay(1000);

  // Attempt WiFi connect and NTP (for SAS expiry)
  connectWiFi(20);
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  LOG_INFO("SYSTEM", "Setup complete.");
}

void loop() {
  static int currentAngle = 90;

  // Derive next action from lastTRNG (feedback chain)
  int targetAngle;
  int speedDelay;

  if (lastTRNG == 0) {
    targetAngle = random(10, 170);
    speedDelay = 10;
  } else {
    targetAngle = lastTRNG % 160 + 10;
    speedDelay = (lastTRNG >> 8) % 15 + 5;
  }

  // Move servo and harvest entropy
  moveServoAndHarvest(currentAngle, targetAngle, speedDelay);
  currentAngle = targetAngle;

  // Change LED brightness (actuator perturbation)
  int ledVal = (lastTRNG >> 16) % 255;
  ledcWrite(2, ledVal);

  // Generate new TRNG from pool
  uint32_t newRandom = generateTrueRandomNumber();
  lastTRNG = newRandom;

  // Output logs + OLED
  Serial.print("Pool Index: "); Serial.print(poolIndex);
  Serial.print(" | TRNG (Dec): "); Serial.print(newRandom);
  Serial.print(" | TRNG (Hex): "); Serial.println(newRandom, HEX);

  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "True RNG Active");
  Heltec.display->drawString(0, 20, String(newRandom));
  Heltec.display->drawString(0, 40, "Hex: " + String(newRandom, HEX));
  Heltec.display->display();

  // Compose telemetry JSON
  StaticJsonDocument<256> doc;
  doc["trng"] = (uint32_t)newRandom;
  char hexBuff[11]; 
  sprintf(hexBuff, "0x%08X", newRandom);
  doc["hex"] = hexBuff;
  doc["poolIndex"] = poolIndex;

  char outJson[256];
  size_t outLen = serializeJson(doc, outJson, sizeof(outJson));
  LOG_DEBUG("JSON", ("Serialized size: " + String(outLen)).c_str());

  if (millis() - lastTelemetryTick >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTick = millis();

    if (WiFi.status() != WL_CONNECTED) {
      LOG_WARN("WIFI", "Not connected. Reconnecting...");
      if (!connectWiFi(30)) {
        LOG_ERROR("WIFI", "Reconnect failed. Skipping telemetry.");
      }
    }

    String sas = generateSasToken(3600);
    if (sas.length() == 0) {
      LOG_ERROR("SAS", "SAS token generation failed. Skipping telemetry.");
    } else {
      if (!postTelemetryToIoTHub(sas, outJson)) {
        LOG_ERROR("HTTP", "Telemetry POST failed.");
      }
    }
  }

  delay(100); // small pause between TRNG cycles
}
