// =========================================================
// MPU-6050 (acelerômetro + giroscópio) — leitura I2C direta
// Referência para portagem ao hdrop_pose (ESP-IDF)
// =========================================================
// SDA -> A4 | SCL -> A5 | VCC -> 5V | GND -> GND
// XDA, XCL, AD0, INT -> não conectar
//
// Nota: no ESP32, o heading vem do QMC5883L (hdrop_heading).
// Este módulo fornece accel/gyro complementar ao sistema de pose.
//
// Para compilar isolado: platformio.ini -> build_src_filter = +<mpu.cpp>
// =========================================================

#include <Arduino.h>
#include <Wire.h>

const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_PWR_MGMT_2 = 0x6C;
const uint8_t REG_ACCEL_XOUT = 0x3B;
const uint8_t REG_WHO_AM_I   = 0x75;

const float ACCEL_SCALE = 16384.0f;  // ±2g  -> LSB/g
const float GYRO_SCALE  =   131.0f;  // ±250°/s -> LSB/(°/s)

float accel_x, accel_y, accel_z;  // em g
float gyro_x,  gyro_y,  gyro_z;   // em °/s
bool  mpu_ok = false;
unsigned long t_prev = 0;

void mpuEscreve(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

uint8_t mpuLe(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.read();
}

int16_t lePar() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

bool mpuInit() {
  mpuEscreve(REG_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  mpuEscreve(REG_PWR_MGMT_1, 0x01);  // acorda (clock PLL X)
  delay(10);
  mpuEscreve(REG_PWR_MGMT_2, 0x00);  // tira do standby
  mpuEscreve(0x1B, 0x00);            // gyro ±250
  mpuEscreve(0x1C, 0x00);            // accel ±2g
  delay(10);
  uint8_t who = mpuLe(REG_WHO_AM_I);
  return (who != 0x00 && who != 0xFF);
}

void mpuLeMotion() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)MPU_ADDR, 14) < 14) return;

  accel_x = lePar() / ACCEL_SCALE;
  accel_y = lePar() / ACCEL_SCALE;
  accel_z = lePar() / ACCEL_SCALE;
  lePar();  // temperatura (ignora)
  gyro_x  = lePar() / GYRO_SCALE;
  gyro_y  = lePar() / GYRO_SCALE;
  gyro_z  = lePar() / GYRO_SCALE;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  Serial.println();
  Serial.println("===== MPU-6050 =====");
  Serial.println("Scan I2C...");
  int n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  dispositivo em 0x");
      Serial.println(addr, HEX);
      n++;
    }
  }
  if (n == 0) Serial.println("  NENHUM dispositivo (confira SDA/SCL/solda)");

  mpu_ok = mpuInit();
  Serial.print("WHO_AM_I = 0x"); Serial.println(mpuLe(REG_WHO_AM_I), HEX);
  Serial.print("MPU ok: ");      Serial.println(mpu_ok ? "SIM" : "NAO");
  Serial.println("====================");

  t_prev = millis();
}

void loop() {
  if (millis() - t_prev < 50) return;  // ~20 Hz
  t_prev = millis();

  if (mpu_ok) mpuLeMotion();

  // Log para Teleplot
  Serial.print(">ax:"); Serial.print(accel_x, 3);
  Serial.print(",ay:"); Serial.print(accel_y, 3);
  Serial.print(",az:"); Serial.print(accel_z, 3);
  Serial.print(",gx:"); Serial.print(gyro_x, 2);
  Serial.print(",gy:"); Serial.print(gyro_y, 2);
  Serial.print(",gz:"); Serial.println(gyro_z, 2);
}
