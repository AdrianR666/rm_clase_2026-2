#include <WiFi.h>
#include <WiFiUdp.h>

// Configuración de red WiFi
const char* ssid = "MiRedWifi";         // ← Cambia esto
const char* password = "123456789"; // ← Cambia esto

// Configuración de UDP
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];  // Buffer para almacenar los paquetes UDP
String data = "";          // Variable para almacenar los datos recibidos

// Pines de los motores (ajústalos si usas un driver distinto)
const int IN1 = 18;
const int IN2 = 19;
const int IN3 = 32;
const int IN4 = 33;
const int ENA = 5;
const int ENB = 25;

const int PWM_MAX = 255;   // Valor máximo de PWM
const float maxSpeed = 100.0; // Velocidad máxima esperada desde UDP

void setup() {
  Serial.begin(115200);

  // Configura pines de motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Conecta a Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Conectado a WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Inicia UDP
  Udp.begin(localUdpPort);
  Serial.print("Esperando mensajes en el puerto UDP: ");
  Serial.println(localUdpPort);
}

void loop() {
  // Revisa si hay datos entrantes en UDP
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Lee el paquete UDP
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Asegura que la cadena se termine
    }

    // Muestra el paquete recibido
    Serial.print("Mensaje UDP recibido: ");
    Serial.println(incomingPacket);

    // Procesa los datos (se espera un formato tipo "v1,v2")
    int sepIndex = String(incomingPacket).indexOf(',');
    if (sepIndex > 0) {
      float vel1 = String(incomingPacket).substring(0, sepIndex).toFloat();
      float vel2 = String(incomingPacket).substring(sepIndex + 1).toFloat();
      moverMotor(vel1, IN1, IN2, ENA);
      moverMotor(-vel2, IN3, IN4, ENB);
    }
  }
}

// Función para mover los motores según las velocidades recibidas
void moverMotor(float vel, int in1, int in2, int pwmPin) {
  int pwm = min((int)(abs(vel) / maxSpeed * PWM_MAX), PWM_MAX);

  if (vel > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (vel < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwmPin, pwm); // Usa ledcWrite si estás en ESP32 moderno
}
