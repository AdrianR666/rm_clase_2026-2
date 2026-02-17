# rm_clase_2026-2
Repositorio para la clase de robótica móvil del Doctor Víctor Javier González Villela, semestre 2026-2


Este repositorio contiene el conjunto de archivos utilizados en la clase de **Robótica Móvil**, enfocados en la implementación y control de robots utilizando una combinación de:

- **ESP32** como plataforma de control embebido (programada desde el IDE de Arduino).
- **Python** para el desarrollo de interfaces, procesamiento de datos y comunicación remota.

## ¿Qué encontrarás aquí?

El repositorio está dividido en dos carpetas principales:

- `arduino/`: Contiene los archivos `.ino` y otros necesarios para programar una ESP32 desde el IDE de Arduino. Estos programas generalmente manejan la conexión WiFi, la recepción de comandos vía UDP y el control de motores o sensores.

- `python/`: Scripts en Python que permiten la comunicación con el ESP32 (por ejemplo, envío de comandos de velocidad), procesamiento de imágenes (si se usa cámara) y lógica de control en general.


## Consideraciones importantes

### Para el entorno de **Python**:

- Se **recomienda** el uso de un entorno virtual para mantener organizadas las dependencias, aunque no es obligatorio. (***Manual en carpeta python**)
- Se **recomienda** el uso de Visual Studio Code, pero no es obligatorio 

#### Librerías necesarias:

Asegúrate de instalar las siguientes versiones específicas para evitar problemas de compatibilidad:

- pip install numpy==2.2.1
- pip install websockets==14.2
- pip install opencv-contrib-python==4.10.0.84

### 🔧 ESP32 – Consideraciones importantes

- El código está diseñado para ser ejecutado en una **ESP32** mediante el **IDE de Arduino**.
- Debes instalar el **core ESP32 de Espressif Systems**, y asegurarte de tener la siguiente versión:
  ESP32 by Espressif Systems – Versión 2.0.11
Esto se puede configurar desde el **Gestor de tarjetas** en el IDE de Arduino. Usar otra versión podría causar errores de compilación o comportamiento inesperado.

- Las siguientes librerías deben incluirse en el código para habilitar la conectividad WiFi y la comunicación UDP:
#include <WiFi.h>
#include <WiFiUdp.h>
