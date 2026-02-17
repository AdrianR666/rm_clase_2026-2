#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Encoder.h>

// WiFi
const char* ssid = "MiRedWifi";
const char* password = "123456789";

// UDP
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];
float setpoint1 =100, setpoint2 = 100;

// Encoders
ESP32Encoder encoder1, encoder2;

// Pines de motor
#define IN1 18
#define IN2 19
#define ENA 5

#define IN3 32
#define IN4 33
#define ENB 25

// Pines de encoder
#define ENC1_A 4
#define ENC1_B 15
#define ENC2_A 26
#define ENC2_B 27

// PWM
const int PWM_FREQ = 1000;
const int PWM_RES = 8;
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
const int PWM_MAX = 255;
const int DEAD_ZONE = 50;
const float Ts = 0.1; // Tiempo de muestreo en segundos

// PID motor 1
double err1 = 0, err1_prev = 0, err1_prev2 = 0, out1 = 0, out1_prev = 0;
double Kp1 = 1.2, Ki1 = 2.5, Kd1 = 0.001;

// PID motor 2
double err2 = 0, err2_prev = 0, err2_prev2 = 0, out2 = 0, out2_prev = 0;
double Kp2 = 1, Ki2 = 0, Kd2 = 0;

// RPM
double rpm1 = 0, rpm2 = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);

  // Motores
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES); ledcAttachPin(ENA, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES); ledcAttachPin(ENB, PWM_CH2);

  // Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder1.attachFullQuad(ENC1_A, ENC1_B);
  encoder2.attachFullQuad(ENC2_A, ENC2_B);
  encoder1.clearCount();
  encoder2.clearCount();

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Udp.begin(localUdpPort);
  Serial.println("WiFi conectada");
  lastTime = millis();
}

void loop() {
  // Leer UDP
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(incomingPacket, 255);
    incomingPacket[len] = 0;
    String msg = String(incomingPacket);
    int comma = msg.indexOf(',');
    if (comma > 0) {
      setpoint1 = msg.substring(0, comma).toFloat();
      setpoint2 = msg.substring(comma + 1).toFloat();
    }
  }

  // PID cada 100 ms
  if (millis() - lastTime >= 100) {
    long c1 = encoder1.getCount();
    long c2 = encoder2.getCount();
    encoder1.clearCount();
    encoder2.clearCount();

    rpm1 = (c1 / 360.0) * 60.0 / Ts;
    rpm2 = (c2 / 360.0) * 60.0 / Ts;

    // PID motor 1
    out1 = computePID(setpoint1, rpm1, out1_prev, err1, err1_prev, err1_prev2, Kp1, Ki1, Kd1);
    moverMotor(out1, IN1, IN2, PWM_CH1);

    // PID motor 2
    out2 = computePID(setpoint2, rpm2, out2_prev, err2, err2_prev, err2_prev2, Kp2, Ki2, Kd2);
    moverMotor(out2, IN3, IN4, PWM_CH2);

    Serial.printf("SP1:%.1f R1:%.1f | SP2:%.1f R2:%.1f\n", setpoint1, rpm1, setpoint2, rpm2);
    Serial.printf("MOTOR 2 >> SP: %.2f | RPM: %.2f | OUT: %.2f\n", setpoint2, rpm2, out2);  

    lastTime = millis();
  }
}

double computePID(double sp, double rpm, double &prevOut, double &e, double &e1, double &e2, double Kp, double Ki, double Kd) {
  e = sp - rpm;
  double output = prevOut +
      (Kp + Kd / Ts) * e +
      (-Kp + Ki * Ts - 2 * Kd / Ts) * e1 +
      (Kd / Ts) * e2;

  // Actualizar errores anteriores
  prevOut = output;
  e2 = e1;
  e1 = e;

  // Zona muerta
  if (abs(output) > 0 && abs(output) < DEAD_ZONE)
    output = (output > 0) ? DEAD_ZONE : -DEAD_ZONE;

  return constrain(output, -PWM_MAX, PWM_MAX);
}

void moverMotor(double vel, int in1, int in2, int pwmCh) {
  int pwm = abs((int)vel);
  if (vel > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (vel < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  ledcWrite(pwmCh, pwm);
}
