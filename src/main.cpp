// =========================================================
// PROJETO H-DROP — código principal (Arduino Uno prototipagem)
// 3 ultrassônicos com filtro 5 camadas + MPU-6050
// Referência para portagem ao ESP-IDF (ETAPA 6 + ETAPA 2)
// =========================================================
// Ultrassônicos (trigger/echo):
//   Frontal  (0°)   -> TRIG=2,  ECHO=3
//   Esquerdo (+90°) -> TRIG=4,  ECHO=5
//   Direito  (-90°) -> TRIG=9,  ECHO=10
// MPU-6050 (I2C):
//   SDA -> A4 | SCL -> A5 | VCC -> 5V | GND -> GND
// Alimentação sensores: fonte MB102 5V com GND comum ao Arduino
// =========================================================

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// ULTRASSÔNICOS — filtro 5 camadas (portar para hdrop_avoidance)
// ============================================================
const int TRIG[3] = {2, 4, 9};   // frontal, esquerdo, direito
const int ECHO[3] = {3, 5, 10};

const float DIST_MIN_CM    =  20.0f;
const float DIST_MAX_CM    = 300.0f;
const float JUMP_THRESH_CM =  50.0f;

const float THRESH_FRONT_CM = 100.0f;  // AVO_D_THRESH_FRONT
const float THRESH_SIDE_CM  =  60.0f;  // AVO_D_THRESH_SIDE

const int   N_MA      = 6;
const float ALPHA_EMA = 0.75f;
const int   ALARM_ON  = 2;
const int   ALARM_OFF = 3;

const unsigned long PULSE_TIMEOUT = 18000UL;

struct USSensor {
    float    last_valid;
    float    window[N_MA];
    uint8_t  win_idx;
    float    dist_ema;
    uint8_t  cnt_below;
    uint8_t  cnt_above;
    bool     alarm;
    float    thresh;
};

USSensor usSensor[3];

static void usInit(USSensor &s, float threshold) {
    s.last_valid = DIST_MAX_CM;
    for (int i = 0; i < N_MA; i++) s.window[i] = DIST_MAX_CM;
    s.win_idx   = 0;
    s.dist_ema  = DIST_MAX_CM;
    s.cnt_below = 0;
    s.cnt_above = 0;
    s.alarm     = false;
    s.thresh    = threshold;
}

static float lerUltraBruto(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(5);
    digitalWrite(trig, HIGH);
    delayMicroseconds(15);
    digitalWrite(trig, LOW);
    unsigned long dur = pulseIn(echo, HIGH, PULSE_TIMEOUT);
    if (dur == 0) return -1.0f;
    return (dur * 0.0343f) / 2.0f;
}

static void usFiltrar(USSensor &s, float raw) {
    // Camada 1: validação de range [20, 300] cm
    if (raw < DIST_MIN_CM || raw > DIST_MAX_CM) {
        raw = s.last_valid;
    } else {
        // Camada 2: rejeição de salto > 50 cm
        if (fabsf(raw - s.last_valid) > JUMP_THRESH_CM) {
            raw = 0.7f * s.last_valid + 0.3f * raw;
        }
        s.last_valid = raw;
    }

    // Camada 3: média móvel N=6
    s.window[s.win_idx] = raw;
    s.win_idx = (s.win_idx + 1) % N_MA;
    float soma = 0.0f;
    for (int i = 0; i < N_MA; i++) soma += s.window[i];

    // Camada 4: EMA α=0.75
    s.dist_ema = ALPHA_EMA * s.dist_ema + (1.0f - ALPHA_EMA) * (soma / N_MA);

    // Camada 5: histerese alarme (≥2 ativa, ≥3 desativa)
    if (s.dist_ema < s.thresh) {
        s.cnt_above = 0;
        if (s.cnt_below < 255) s.cnt_below++;
        if (s.cnt_below >= ALARM_ON) s.alarm = true;
    } else {
        s.cnt_below = 0;
        if (s.cnt_above < 255) s.cnt_above++;
        if (s.cnt_above >= ALARM_OFF) s.alarm = false;
    }
}

// ============================================================
// MPU-6050 — acelerômetro + giroscópio (portar para hdrop_pose)
// ============================================================
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_PWR_MGMT_2 = 0x6C;
const uint8_t REG_ACCEL_XOUT = 0x3B;
const uint8_t REG_WHO_AM_I   = 0x75;

const float ACCEL_SCALE = 16384.0f;  // ±2g
const float GYRO_SCALE  =   131.0f;  // ±250°/s

float accel_x, accel_y, accel_z;
float gyro_x,  gyro_y,  gyro_z;
bool  mpu_ok = false;

static void mpuEscreve(uint8_t reg, uint8_t valor) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(valor);
    Wire.endTransmission();
}

static uint8_t mpuLe(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 1);
    return Wire.read();
}

static int16_t lePar() {
    return (int16_t)((Wire.read() << 8) | Wire.read());
}

static bool mpuInit() {
    mpuEscreve(REG_PWR_MGMT_1, 0x80); delay(100);  // reset
    mpuEscreve(REG_PWR_MGMT_1, 0x01); delay(10);   // acorda (clock PLL X)
    mpuEscreve(REG_PWR_MGMT_2, 0x00);               // tira do standby
    mpuEscreve(0x1B, 0x00);                         // gyro ±250°/s
    mpuEscreve(0x1C, 0x00);                         // accel ±2g
    delay(10);
    uint8_t who = mpuLe(REG_WHO_AM_I);
    return (who != 0x00 && who != 0xFF);
}

static void mpuLeMotion() {
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

// ============================================================
// SETUP / LOOP
// ============================================================
const unsigned long Ts_ms = 50UL;
unsigned long t_prev_ms = 0;

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 3; i++) {
        pinMode(TRIG[i], OUTPUT);
        pinMode(ECHO[i], INPUT);
        digitalWrite(TRIG[i], LOW);
    }

    float thresholds[3] = {THRESH_FRONT_CM, THRESH_SIDE_CM, THRESH_SIDE_CM};
    for (int i = 0; i < 3; i++) usInit(usSensor[i], thresholds[i]);

    Wire.begin();
    mpu_ok = mpuInit();
    Serial.print("MPU-6050: ");
    Serial.println(mpu_ok ? "OK" : "NAO detectado");

    t_prev_ms = millis();
}

void loop() {
    if (millis() - t_prev_ms < Ts_ms) return;
    t_prev_ms += Ts_ms;

    // Ultrassônicos
    for (int i = 0; i < 3; i++) {
        usFiltrar(usSensor[i], lerUltraBruto(TRIG[i], ECHO[i]));
        delay(5);
    }

    // MPU
    if (mpu_ok) mpuLeMotion();

    // ---- Teleplot ----
    // Obstáculos (espelha obs:{f,l,r} do hdrop_avoidance)
    Serial.print(">f:");  Serial.print(usSensor[0].dist_ema, 1);
    Serial.print(",l:");  Serial.print(usSensor[1].dist_ema, 1);
    Serial.print(",r:");  Serial.print(usSensor[2].dist_ema, 1);
    Serial.print(",af:"); Serial.print(usSensor[0].alarm ? 1 : 0);
    Serial.print(",al:"); Serial.print(usSensor[1].alarm ? 1 : 0);
    Serial.print(",ar:"); Serial.print(usSensor[2].alarm ? 1 : 0);
    // IMU
    Serial.print(",ax:"); Serial.print(accel_x, 3);
    Serial.print(",ay:"); Serial.print(accel_y, 3);
    Serial.print(",az:"); Serial.print(accel_z, 3);
    Serial.print(",gx:"); Serial.print(gyro_x, 2);
    Serial.print(",gy:"); Serial.print(gyro_y, 2);
    Serial.print(",gz:"); Serial.println(gyro_z, 2);
}
