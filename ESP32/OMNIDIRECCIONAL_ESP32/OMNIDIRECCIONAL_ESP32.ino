#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ====================== Configuración de WiFi y UDP ======================
const char* ssid = "MiRedWifi";
const char* password = "123456789";
const int udpPort = 58625;  // Puerto para recibir datos UDP
WiFiUDP udp;
byte packetBuffer[16];  // Buffer para recibir los datos (4 floats de 4 bytes)

// ===================== Configuración del Motor Shield =====================
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

// ===================== Función para mapear la velocidad =====================
int mapearVelocidad(float vel) {
    return constrain(int(vel), -255, 255);  // Limita la velocidad entre -255 y 255
}

// ====================== Configuración ======================
void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi...");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nConectado a WiFi. Dirección IP: " + WiFi.localIP().toString());
    udp.begin(udpPort);
    
    // Inicializar el Motor Shield
    if (!AFMS.begin()) {
        Serial.println("No se encontró el Motor Shield");
        while (1);
    }
}

// ====================== Loop principal ======================
void loop() {
    int packetSize = udp.parsePacket();
    
    if (packetSize == 16) {  // 4 valores float de 4 bytes cada uno
        udp.read(packetBuffer, 16);

        float velocidades[4];
        memcpy(velocidades, packetBuffer, 16);  // Convertir bytes a floats

        int v1 = mapearVelocidad(velocidades[0]);
        int v2 = mapearVelocidad(velocidades[1]);
        int v3 = mapearVelocidad(velocidades[2]);
        int v4 = mapearVelocidad(velocidades[3]);

        Serial.printf("Velocidades recibidas -> v1: %d, v2: %d, v3: %d, v4: %d\n", v1, v2, v3, v4);

        // Enviar velocidades a los motores
        motor1->setSpeed(abs(v1));
        motor2->setSpeed(abs(v2));
        motor3->setSpeed(abs(v3));
        motor4->setSpeed(abs(v4));

        motor1->run(v1 >= 0 ? FORWARD : BACKWARD);
        motor2->run(v2 >= 0 ? FORWARD : BACKWARD);
        motor3->run(v3 >= 0 ? FORWARD : BACKWARD);
        motor4->run(v4 >= 0 ? FORWARD : BACKWARD);
    }
}
