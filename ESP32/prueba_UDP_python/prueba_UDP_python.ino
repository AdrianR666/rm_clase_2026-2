#include <WiFi.h>
#include <WiFiUdp.h>

// Configuración de red Wi-Fi
const char* ssid = "TP-Link_8960";       // Cambia esto por el nombre de tu red Wi-Fi
const char* password = "53899736"; // Cambia esto por la contraseña de tu red Wi-Fi

// Configuración del socket UDP
WiFiUDP udp;
unsigned int localPort = 12345; // Puerto para escuchar mensajes UDP

void setup() {
    Serial.begin(115200);

    // Conectar a la red Wi-Fi
    Serial.println("Conectando a WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println("\nConectado a la red WiFi");
    Serial.print("IP de la ESP32: ");
    Serial.println(WiFi.localIP());

    // Iniciar el servidor UDP
    udp.begin(localPort);
    Serial.printf("Servidor UDP escuchando en el puerto %d\n", localPort);
}

void loop() {
    char incomingPacket[255]; // Buffer para almacenar el mensaje recibido
    int packetSize = udp.parsePacket(); // Verificar si hay datos entrantes

    if (packetSize) {
        // Leer el mensaje recibido
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = '\0'; // Asegurarse de que el mensaje es un string válido
        }

        Serial.printf("Mensaje recibido desde %s:%d: %s\n",
                      udp.remoteIP().toString().c_str(),
                      udp.remotePort(),
                      incomingPacket);

        // Responder al remitente (opcional)
        String respuesta = "Mensaje recibido correctamente";
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print(respuesta);
        udp.endPacket();
    }
}
