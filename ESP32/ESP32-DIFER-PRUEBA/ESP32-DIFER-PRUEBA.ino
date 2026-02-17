#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Encoder.h>

// WiFi
const char* ssid = "MiRedWifi";
const char* password = "123456789";
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];

// Parámetros globales
const int PWM_MAX = 255;
const int DEAD_ZONE = 50;
const float Ts = 0.1; // 100ms

// Clase PID
class PIDController {
public:
  float Kp, Ki, Kd;
  float prevOut = 0;
  float e = 0, e1 = 0, e2 = 0;

  PIDController(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd) {}

  float compute(float setpoint, float measurement) {
    e = setpoint - measurement;
    float output = prevOut +
      (Kp + Kd / Ts) * e +
      (-Kp + Ki * Ts - 2 * Kd / Ts) * e1 +
      (Kd / Ts) * e2;

    prevOut = output;
    e2 = e1;
    e1 = e;

    if (abs(output) > 0 && abs(output) < DEAD_ZONE)
      output = (output > 0) ? DEAD_ZONE : -DEAD_ZONE;

    return constrain(output, -PWM_MAX, PWM_MAX);
  }
};

// Clase Motor
class MotorControl {
public:
  int in1, in2, pwmChannel;
  MotorControl(int _in1, int _in2, int pwmPin, int channel, int freq = 1000, int res = 8) {
    in1 = _in1; in2 = _in2; pwmChannel = channel;
    pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
    ledcSetup(pwmChannel, freq, res);
    ledcAttachPin(pwmPin, pwmChannel);
  }

  void move(float vel) {
    int pwm = abs((int)vel);
    if (vel > 0) {
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    } else if (vel < 0) {
      digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    }
    ledcWrite(pwmChannel, pwm);
  }
};

// Instancias
ESP32Encoder encoder1, encoder2;
PIDController pid1(1.2, 2.5, 0.001);
PIDController pid2(1.2, 2.5, 0.001);
MotorControl motor1(18, 19, 5, 0);   // IN1, IN2, ENA, canal
MotorControl motor2(32, 33, 25, 1);  // IN3, IN4, ENB, canal

// Variables de control
float setpoint1 = 0, setpoint2 = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder1.attachFullQuad(4, 15);
  encoder2.attachFullQuad(26, 27);
  encoder1.clearCount();
  encoder2.clearCount();

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

  // Cada 100ms
  if (millis() - lastTime >= 100) {
    long c1 = encoder1.getCount(); encoder1.clearCount();
    long c2 = encoder2.getCount(); encoder2.clearCount();

    float rpm1 = (c1 / 360.0) * 60.0 / Ts;
    float rpm2 = (c2 / 360.0) * 60.0 / Ts;

    float out1 = pid1.compute(setpoint1, rpm1);
    float out2 = pid2.compute(setpoint2, rpm2);

    motor1.move(out1);
    motor2.move(out2);

    Serial.printf("SP1:%.1f R1:%.1f | SP2:%.1f R2:%.1f\n", setpoint1, rpm1, setpoint2, rpm2);
    lastTime = millis();
  }
}
