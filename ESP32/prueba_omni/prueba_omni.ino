// Puente H #1 - Lado izquierdo
#define ENA1  12   // PWM
#define IN1   14
#define IN2   27

#define ENB1  33   // PWM
#define IN3   26
#define IN4   25

// Puente H #2 - Lado derecho
#define ENA2  4    // PWM
#define IN5   16
#define IN6   17

#define ENB2  19   // PWM
#define IN7   5
#define IN8   18

// Canales PWM
#define PWM_FREQ    1000
#define PWM_RES     8
#define PWM_CH_ENA1 0
#define PWM_CH_ENB1 1
#define PWM_CH_ENA2 2
#define PWM_CH_ENB2 3

void setup() {
  // Configurar pines como salida
  int motorPins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8};
  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  // Configurar PWM para los 4 EN
  ledcSetup(PWM_CH_ENA1, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA1, PWM_CH_ENA1);

  ledcSetup(PWM_CH_ENB1, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB1, PWM_CH_ENB1);

  ledcSetup(PWM_CH_ENA2, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA2, PWM_CH_ENA2);

  ledcSetup(PWM_CH_ENB2, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB2, PWM_CH_ENB2);
}

// Funciones para control
void motorForward(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorBackward(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void motorStop(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void loop() {
  int speed = 200; // Velocidad PWM (0 a 255)

  // -------- MOVER HACIA ADELANTE --------
  motorForward(IN1, IN2);   // Motor 1
  motorForward(IN3, IN4);   // Motor 2
  motorForward(IN5, IN6);   // Motor 3
  motorForward(IN7, IN8);   // Motor 4

  // Enviar PWM
  ledcWrite(PWM_CH_ENA1, speed);
  ledcWrite(PWM_CH_ENB1, speed);
  ledcWrite(PWM_CH_ENA2, speed);
  ledcWrite(PWM_CH_ENB2, speed);

  delay(3000); // 3 segundos hacia adelante

  // -------- CAMBIO DE SENTIDO: HACIA ATRÁS --------
  motorBackward(IN1, IN2);  // Motor 1
  motorBackward(IN3, IN4);  // Motor 2
  motorBackward(IN5, IN6);  // Motor 3
  motorBackward(IN7, IN8);  // Motor 4

  delay(3000); // 3 segundos hacia atrás

  // -------- DETENER TODO --------
  motorStop(IN1, IN2);
  motorStop(IN3, IN4);
  motorStop(IN5, IN6);
  motorStop(IN7, IN8);

  // Apagar PWM
  ledcWrite(PWM_CH_ENA1, 0);
  ledcWrite(PWM_CH_ENB1, 0);
  ledcWrite(PWM_CH_ENA2, 0);
  ledcWrite(PWM_CH_ENB2, 0);

  delay(3000); // 3 segundos en reposo
}
