#include <Arduino.h>

// Definição de Pinos
const int TRIG_PIN  = 8;  // Pino de disparo (RX no sensor)
const int ECHO_PIN  = 9;  // Pino de retorno (TX no sensor)
const int REF_PIN   = A1; // Potenciômetro para definir a distância alvo
const int PWM_PIN   = 6;  // Saída para o motor/atuador

// Parâmetros do Controlador
float K  = 0.5f;   // Ganho Integral (Reduzido para ultrassom ser mais estável)
float Ts = 0.06f;  // Tempo de amostragem aumentado para 60ms (ideal para som)

// Normalização
const float DIST_MAX_PROJETO = 300.0f; // 300cm (3 metros) = 1.0 no gráfico

// Sinais do Sistema
float ref = 0.0f;
float y   = 0.0f;
float e   = 0.0f;
float u   = 0.0f;
float u_prev = 0.0f;

// Zona morta do motor (ajuste conforme seu motor)
const int PWM_MIN_EFETIVO = 100;

// Gerenciamento de tempo
unsigned long t_prev_ms = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inicializa o trigger em LOW
  digitalWrite(TRIG_PIN, LOW);

  // Velocidade de 115200 para o gráfico ficar bem fluido no VS Code
  Serial.begin(115200);

  t_prev_ms = millis();
}

void loop() {
  // Executa o controle a cada Ts segundos (60ms)
  if (millis() - t_prev_ms >= (Ts * 1000)) {
    t_prev_ms += (Ts * 1000);

    // ----------------------------
    // 1. Referência (Setpoing)
    // ----------------------------
    // Lê o potenciômetro e normaliza de 0.0 a 1.0
    ref = analogRead(REF_PIN) / 1023.0f;

    // ----------------------------
    // 2. Leitura do Sensor Ultrassônico (y)
    // ----------------------------
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(15); 
    digitalWrite(TRIG_PIN, LOW);

    // Timeout de 40ms para cobrir até ~6 metros
    long duracao = pulseIn(ECHO_PIN, HIGH, 40000);
    
    float distancia_cm = (duracao * 0.0343f) / 2.0f;

    // Se houver erro de leitura (duracao=0), mantemos o y anterior para evitar picos
    if (duracao > 0) {
      y = distancia_cm / DIST_MAX_PROJETO;
    }

    // Limitadores para manter o gráfico limpo
    if (y < 0.0f) y = 0.0f;
    if (y > 1.2f) y = 1.2f;

    // ----------------------------
    // 3. Controle Integral
    // ----------------------------
    // Erro: Se ref > y, o objeto está mais perto do que a meta
    e = ref - y;

    // Equação de recorrência do Integrador
    u = u_prev + K * Ts * e;

    // Saturação do sinal de controle (PWM vai de 0 a 255)
    if (u < 0.0f)   u = 0.0f;
    if (u > 255.0f) u = 255.0f;

    int pwm = (int)u;

    // Tratamento de zona morta
    if (pwm > 0 && pwm < PWM_MIN_EFETIVO) {
      pwm = PWM_MIN_EFETIVO;
    }

    analogWrite(PWM_PIN, pwm);

    // ----------------------------
    // 4. Log para Teleplot / Serial Plotter
    // ----------------------------
    float u_n   = u / 255.0f;   // Esforço de controle normalizado (0-1)
    float pwm_n = pwm / 255.0f; // PWM real normalizado (0-1)

    Serial.print(">");
    Serial.print("ref:");
    Serial.print(ref, 3);
    Serial.print(",");

    Serial.print("dist_y:");
    Serial.print(y, 3);
    Serial.print(",");

    Serial.print("controle_u:");
    Serial.print(u_n, 3);
    Serial.print(",");

    Serial.print("pwm_saida:");
    Serial.println(pwm_n, 3);

    u_prev = u;
  }
}