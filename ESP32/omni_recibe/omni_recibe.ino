#include <WiFi.h>
#include <WiFiUdp.h>


// ========== CONFIGURACIÓN WiFi ==========
const char* ssid = "MiRedWifi";
const char* password = "123456789";
WiFiUDP udp;
unsigned int localUdpPort = 58625;
const int packetSize = 8;  // 4 int16_t = 8 bytes

// ========== PINES Puente H ==========
#define ENA1  12   // PWM
#define IN1   14
#define IN2   27

#define ENB1  33   // PWM
#define IN3   26
#define IN4   25

#define ENA2  4    // PWM
#define IN5   16
#define IN6   17

#define ENB2  19   // PWM
#define IN7   5
#define IN8   18

// ========== Inicialización de motores ==========
void setupMotor(int canal_pwm, int en, int in1, int in2) {
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  ledcAttachPin(en, canal_pwm);
  ledcSetup(canal_pwm, 1000, 8);  // canal, frecuencia 1kHz, resolución 8 bits
}

// ========== Aplicar velocidad a motor ==========
void setMotor(int canal_pwm, int in1, int in2, int velocidad) {
  int vel = constrain(abs(velocidad), 0, 255);
  if (velocidad > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (velocidad < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(canal_pwm, vel);
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
 

  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_11dBm); // o incluso WIFI_POWER_8_5dBm
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi. IP local: " + WiFi.localIP().toString());

  udp.begin(localUdpPort);
  Serial.printf("Esperando paquetes UDP en el puerto %d...\n", localUdpPort);

  // Inicializar motores con canales PWM únicos (0 a 3)
  setupMotor(0, ENA1, IN1, IN2);  // Motor 1
  setupMotor(1, ENB1, IN3, IN4);  // Motor 2
  setupMotor(2, ENA2, IN5, IN6);  // Motor 3
  setupMotor(3, ENB2, IN7, IN8);  // Motor 4
}

// ========== Loop ==========
void loop() {
  if (udp.parsePacket() >= packetSize) {
    int16_t velocidades[4];
    udp.read((char*)velocidades, packetSize);

    // Aplicar velocidades
    setMotor(0, IN1, IN2, velocidades[0]); // Motor 1 - ENA1
    setMotor(1, IN3, IN4, velocidades[1]); // Motor 2 - ENB1
    setMotor(2, IN5, IN6, velocidades[2]); // Motor 3 - ENA2
    setMotor(3, IN7, IN8, velocidades[3]); // Motor 4 - ENB2

    // Mostrar en Serial
    Serial.printf("UDP recibido: v1=%d, v2=%d, v3=%d, v4=%d\n",
                  velocidades[0], velocidades[1],
                  velocidades[2], velocidades[3]);
  }
}
