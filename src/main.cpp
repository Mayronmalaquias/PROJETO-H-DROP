#include <Arduino.h>

// =========================
// Definição de pinos
// =========================
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;
const int REF_PIN  = A1;
const int PWM_PIN  = 6;

// =========================
// Parâmetros do sistema
// =========================
float Ts    = 0.08f;
float K     = 3.0f;
float alpha = 0.75f;

// Filtro da referência
float alpha_ref = 0.85f;

// =========================
// Faixa do sensor / projeto
// =========================
const float DIST_MIN_VALIDA   = 10.0f;
const float DIST_MAX_VALIDA   = 300.0f;

const float DIST_MIN_PROJETO  = 10.0f;
const float DIST_MAX_PROJETO  = 300.0f;
const float DIST_RANGE        = DIST_MAX_PROJETO - DIST_MIN_PROJETO;

// Rejeição / amortecimento de saltos
const float SALTO_MAX_CM = 30.0f;
const float PESO_SALTO_GRANDE_ANTERIOR = 0.7f;
const float PESO_SALTO_GRANDE_NOVO     = 0.3f;

// Ultrassom
const unsigned long PULSE_TIMEOUT_US = 30000UL;

// Atuador
const int PWM_MIN_EFETIVO = 60;

// Robustez de leitura
const int FALHAS_TOLERADAS = 2;     // poucas falhas ainda toleradas
const int FALHAS_MAX       = 5;     // acima disso: sem leitura confiável

// Se precisar inverter o sentido do controle
const bool INVERTER_CONTROLE = false;

// Distância de risco para obstáculo
const float DIST_OBSTACULO_CM = 30.0f;

// Queda brusca para indicar que algo apareceu de repente
const float DELTA_OBSTACULO_CM = 20.0f;

// Confirmação de estado
const int CONFIRMACAO_OBSTACULO = 3;
const int CONFIRMACAO_LIVRE      = 3;

// =========================
// Variáveis do sistema
// =========================
float ref = 0.0f;
float ref_filtrado = 0.0f;

float distancia_cm_valida    = 50.0f;
float distancia_cm_anterior  = 50.0f;
float distancia_cm_filtrada  = 50.0f;

float y_bruta = 0.0f;
float y       = 0.0f;
float e       = 0.0f;
float u       = 0.0f;
float u_prev  = 0.0f;

bool primeira_leitura_valida = true;
bool sensor_ok = false;
bool sem_leitura = false;

int falhas_consecutivas = 0;

bool obstaculo_detectado = false;
bool obstaculo_apareceu  = false;

int contador_obstaculo = 0;
int contador_livre = 0;

unsigned long t_prev_ms = 0;

// =========================
// Leitura segura do sensor
// =========================
bool lerDistanciaUltrassom(float &dist_cm) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duracao = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
  if (duracao == 0) return false;

  float medida = (duracao * 0.0343f) / 2.0f;

  if (medida < DIST_MIN_VALIDA || medida > DIST_MAX_VALIDA) {
    return false;
  }

  dist_cm = medida;
  return true;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);
  analogWrite(PWM_PIN, 0);

  Serial.begin(115200);

  y = (distancia_cm_valida - DIST_MIN_PROJETO) / DIST_RANGE;
  y = constrain(y, 0.0f, 1.0f);

  ref_filtrado = analogRead(REF_PIN) / 1023.0f;
  ref_filtrado = constrain(ref_filtrado, 0.0f, 1.0f);

  t_prev_ms = millis();
}

void loop() {
  if (millis() - t_prev_ms >= (unsigned long)(Ts * 1000.0f)) {
    t_prev_ms += (unsigned long)(Ts * 1000.0f);

    // =========================
    // 1. Referência com filtro
    // =========================
    float ref_bruto = analogRead(REF_PIN) / 1023.0f;
    ref_bruto = constrain(ref_bruto, 0.0f, 1.0f);

    if (fabs(ref_bruto - ref_filtrado) > 0.005f) {
      ref_filtrado = alpha_ref * ref_filtrado + (1.0f - alpha_ref) * ref_bruto;
    }

    ref = ref_filtrado;

    // =========================
    // 2. Leitura do sensor
    // =========================
    float nova_distancia_cm = 0.0f;
    bool leitura_ok = lerDistanciaUltrassom(nova_distancia_cm);

    distancia_cm_anterior = distancia_cm_valida;
    obstaculo_apareceu = false;

    if (leitura_ok) {
      falhas_consecutivas = 0;
      sem_leitura = false;
      sensor_ok = true;

      if (primeira_leitura_valida) {
        distancia_cm_valida = nova_distancia_cm;
        primeira_leitura_valida = false;
      } else {
        float salto = fabs(nova_distancia_cm - distancia_cm_valida);

        if (salto <= SALTO_MAX_CM) {
          distancia_cm_valida = nova_distancia_cm;
        } else {
          distancia_cm_valida =
            PESO_SALTO_GRANDE_ANTERIOR * distancia_cm_valida +
            PESO_SALTO_GRANDE_NOVO     * nova_distancia_cm;
        }
      }

      if ((distancia_cm_anterior - distancia_cm_valida) > DELTA_OBSTACULO_CM) {
        obstaculo_apareceu = true;
      }

    } else {
      falhas_consecutivas++;

      // tolera poucas falhas sem derrubar imediatamente a confiança
      if (falhas_consecutivas <= FALHAS_TOLERADAS) {
        sensor_ok = true;
        sem_leitura = false;
      }
      // entre toleradas e máximo, ainda segura último valor, mas já indica instabilidade
      else if (falhas_consecutivas <= FALHAS_MAX) {
        sensor_ok = false;
        sem_leitura = true;
      }
      // acima do máximo, assume sem leitura confiável
      else {
        sensor_ok = false;
        sem_leitura = true;
        contador_obstaculo = 0;
        contador_livre = 0;
      }
    }

    // =========================
    // 3. Normalização
    // =========================
    y_bruta = (distancia_cm_valida - DIST_MIN_PROJETO) / DIST_RANGE;
    y_bruta = constrain(y_bruta, 0.0f, 1.0f);

    // =========================
    // 4. Filtro exponencial da saída
    // =========================
    y = alpha * y + (1.0f - alpha) * y_bruta;
    y = constrain(y, 0.0f, 1.0f);

    distancia_cm_filtrada = y * DIST_RANGE + DIST_MIN_PROJETO;

    // =========================
    // 5. Lógica de detecção correta
    // =========================
    if (sensor_ok) {
      if (distancia_cm_valida <= DIST_OBSTACULO_CM) {
        contador_obstaculo++;
        contador_livre = 0;

        if (contador_obstaculo >= CONFIRMACAO_OBSTACULO) {
          obstaculo_detectado = true;
        }
      } else {
        contador_livre++;
        contador_obstaculo = 0;

        if (contador_livre >= CONFIRMACAO_LIVRE) {
          obstaculo_detectado = false;
        }
      }
    } else {
      // sem leitura confiável → não afirma obstáculo
      obstaculo_detectado = false;
      contador_obstaculo = 0;
      contador_livre = 0;
    }

    // =========================
    // 6. Erro
    // =========================
    e = ref - y;
    if (INVERTER_CONTROLE) {
      e = -e;
    }

    // =========================
    // 7. Controle integral com
    //    redução de ganho perto do alvo
    // =========================
    float erro_abs = fabs(e);
    float ganho_efetivo = K;

    if (erro_abs < 0.05f) {
      ganho_efetivo = K * 0.4f;
    }

    float incremento = ganho_efetivo * Ts * e;

    if ((u_prev >= 255.0f && incremento > 0.0f) ||
        (u_prev <= 0.0f   && incremento < 0.0f)) {
      u = u_prev;
    } else {
      u = u_prev + incremento;
    }

    u = constrain(u, 0.0f, 255.0f);

    int pwm = (int)u;

    if (pwm > 0 && pwm < PWM_MIN_EFETIVO) {
      pwm = PWM_MIN_EFETIVO;
    }

    pwm = constrain(pwm, 0, 255);

    analogWrite(PWM_PIN, pwm);
    u_prev = u;

    // =========================
    // 8. Logs para Teleplot
    // =========================
    float u_n   = u / 255.0f;
    float pwm_n = pwm / 255.0f;

    Serial.print(">");

    Serial.print("ref:");
    Serial.print(ref, 3);
    Serial.print(",");

    Serial.print("y_bruta:");
    Serial.print(y_bruta, 3);
    Serial.print(",");

    Serial.print("y_filtrada:");
    Serial.print(y, 3);
    Serial.print(",");

    Serial.print("erro:");
    Serial.print(e, 3);
    Serial.print(",");

    Serial.print("u:");
    Serial.print(u_n, 3);
    Serial.print(",");

    Serial.print("pwm:");
    Serial.print(pwm_n, 3);
    Serial.print(",");

    Serial.print("sensor_ok:");
    Serial.print(sensor_ok ? 1 : 0);
    Serial.print(",");

    Serial.print("sem_leitura:");
    Serial.print(sem_leitura ? 1 : 0);
    Serial.print(",");

    Serial.print("obstaculo:");
    Serial.print(obstaculo_detectado ? 1 : 0);
    Serial.print(",");

    Serial.print("apareceu:");
    Serial.println(obstaculo_apareceu ? 1 : 0);
  }
}