// =========================================================
// 3 SENSORES ULTRASSÔNICOS — filtro 5 camadas
// Referência para portagem ao hdrop_avoidance (ESP-IDF ETAPA 6)
// =========================================================
// Frontal  (0°)   -> TRIG = pino 2  | ECHO = pino 3
// Esquerdo (+90°) -> TRIG = pino 4  | ECHO = pino 5
// Direito  (-90°) -> TRIG = pino 9  | ECHO = pino 10
//
// Alimentação: VCC -> 5V, GND -> GND (trilhas comum + inferior ligada)
//
// Saída Teleplot: >f:<cm>,l:<cm>,r:<cm>,af:<0|1>,al:<0|1>,ar:<0|1>
//   f/l/r = distância filtrada (cm) | af/al/ar = alarme de obstáculo
//
// Thresholds espelham os defines do hdrop_avoidance:
//   AVO_D_THRESH_FRONT = 100 cm | AVO_D_THRESH_SIDE = 60 cm
// =========================================================

#include <Arduino.h>

// ---------- Pinos ----------
const int TRIG[3] = {2, 4, 9};   // frontal, esquerdo, direito
const int ECHO[3] = {3, 5, 10};

// ---------- Range ----------
const float DIST_MIN_CM     =  20.0f;   // abaixo = ponto-cego do sensor
const float DIST_MAX_CM     = 300.0f;   // acima = sem obstáculo relevante
const float JUMP_THRESH_CM  =  50.0f;   // salto > 50 cm → suaviza

// ---------- Thresholds de alarme ----------
const float THRESH_FRONT_CM = 100.0f;   // AVO_D_THRESH_FRONT
const float THRESH_SIDE_CM  =  60.0f;   // AVO_D_THRESH_SIDE

// ---------- Filtros ----------
const int   N_MA        = 6;      // janela média móvel
const float ALPHA_EMA   = 0.75f;  // coeficiente EMA (dist_f = α·ant + (1-α)·media)
const int   ALARM_ON    = 2;      // leituras abaixo do limiar para ativar alarme
const int   ALARM_OFF   = 3;      // leituras acima do limiar para desativar alarme

// ---------- Amostragem ----------
const unsigned long Ts_ms          = 50UL;
const unsigned long PULSE_TIMEOUT  = 18000UL;  // ~300 cm: 300×2/0.0343 ≈ 17493 µs

// ---------- Estrutura por sensor ----------
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

USSensor      sensor[3];
unsigned long t_prev_ms = 0;

// =========================
// Inicialização de um sensor
// =========================
static void sensorInit(USSensor &s, float threshold) {
    s.last_valid = DIST_MAX_CM;
    for (int i = 0; i < N_MA; i++) s.window[i] = DIST_MAX_CM;
    s.win_idx   = 0;
    s.dist_ema  = DIST_MAX_CM;
    s.cnt_below = 0;
    s.cnt_above = 0;
    s.alarm     = false;
    s.thresh    = threshold;
}

// =========================
// Leitura bruta (cm) ou -1 se timeout
// =========================
static float lerBruto(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(5);
    digitalWrite(trig, HIGH);
    delayMicroseconds(15);
    digitalWrite(trig, LOW);
    unsigned long dur = pulseIn(echo, HIGH, PULSE_TIMEOUT);
    if (dur == 0) return -1.0f;
    return (dur * 0.0343f) / 2.0f;
}

// =========================
// Filtragem em 5 camadas
// (portar fielmente para hdrop_avoidance no ESP-IDF)
// =========================
static void filtrar(USSensor &s, float raw) {
    // Camada 1: validação de range — fora de [20, 300] cm ou timeout (-1)
    if (raw < DIST_MIN_CM || raw > DIST_MAX_CM) {
        raw = s.last_valid;  // substitui por último válido sem atualizar cache
    } else {
        // Camada 2: rejeição de salto — blend se variação > 50 cm
        if (fabsf(raw - s.last_valid) > JUMP_THRESH_CM) {
            raw = 0.7f * s.last_valid + 0.3f * raw;
        }
        s.last_valid = raw;
    }

    // Camada 3: média móvel circular N=6
    s.window[s.win_idx] = raw;
    s.win_idx = (s.win_idx + 1) % N_MA;
    float soma = 0.0f;
    for (int i = 0; i < N_MA; i++) soma += s.window[i];
    float media = soma / N_MA;

    // Camada 4: EMA α=0.75
    s.dist_ema = ALPHA_EMA * s.dist_ema + (1.0f - ALPHA_EMA) * media;

    // Camada 5: histerese de alarme (≥2 ativa, ≥3 desativa)
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

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 3; i++) {
        pinMode(TRIG[i], OUTPUT);
        pinMode(ECHO[i], INPUT);
        digitalWrite(TRIG[i], LOW);
    }

    float thresholds[3] = {THRESH_FRONT_CM, THRESH_SIDE_CM, THRESH_SIDE_CM};
    for (int i = 0; i < 3; i++) sensorInit(sensor[i], thresholds[i]);

    t_prev_ms = millis();
}

void loop() {
    if (millis() - t_prev_ms < Ts_ms) return;
    t_prev_ms += Ts_ms;

    for (int i = 0; i < 3; i++) {
        filtrar(sensor[i], lerBruto(TRIG[i], ECHO[i]));
        delay(5);
    }

    // Teleplot — formato espelha obs:{"f","l","r","st"} do hdrop_avoidance
    Serial.print(">f:");  Serial.print(sensor[0].dist_ema, 1);
    Serial.print(",l:");  Serial.print(sensor[1].dist_ema, 1);
    Serial.print(",r:");  Serial.print(sensor[2].dist_ema, 1);
    Serial.print(",af:"); Serial.print(sensor[0].alarm ? 1 : 0);
    Serial.print(",al:"); Serial.print(sensor[1].alarm ? 1 : 0);
    Serial.print(",ar:"); Serial.println(sensor[2].alarm ? 1 : 0);
}
