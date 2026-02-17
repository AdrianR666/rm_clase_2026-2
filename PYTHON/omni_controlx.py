import numpy as np
import socket
import time
import pygame

# ====================== Configuración de UDP ======================
UDP_IP = "192.168.137.96"  # Dirección IP de la ESP32
UDP_PORT = 58625
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ===================== Cinemática Inversa =========================
def calcular_velocidades_ruedas(vx, vy, w):
    r = 0.030    # Radio llanta [m]
    b = 0.1425   # Distancia eje longitudinal [m]
    d = 0.078    # Distancia eje transversal [m]

    rueda1 = (vx / r) - (vy / r) - (((b + d) / r) * w)
    rueda2 = (vx / r) + (vy / r) + (((b + d) / r) * w)
    rueda3 = (vx / r) + (vy / r) - (((b + d) / r) * w)
    rueda4 = (vx / r) - (vy / r) + (((b + d) / r) * w)
    return rueda1, rueda2, rueda3, rueda4

# =================== Enviar Velocidades por UDP ===================
def enviar_velocidades_udp(v1, v2, v3, v4):
    try:
        data = np.array([v1, v2, v3, v4], dtype=np.float32).tobytes()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> v1: {v1}, v2: {v2}, v3: {v3}, v4: {v4}")
    except Exception as e:
        print(f"Error al enviar datos UDP: {e}")

# ==================== Mapeo de Velocidades ========================
def mapear_velocidad(vel):
    return max(-255, min(255, int(vel)))

# ================== Inicializar Joystick ==========================
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Control Xbox conectado. Usa los sticks para mover el robot (Ctrl+C para salir).")

# ==================== Loop Principal ==============================
try:
    while True:
        pygame.event.pump()
        ly = joystick.get_axis(0)   # X stick izquierdo
        lx = -joystick.get_axis(1)  # Y stick izquierdo (invertido)
        rx = joystick.get_axis(3)   # X stick derecho (rotación)

        # Escalar a velocidades (ajusta coeficientes según tu robot)
        vx = lx * 25    # [m/s]
        vy = ly * 25     # [m/s]
        w  = rx * 50     # [rad/s]

        # Calcular velocidades ruedas
        r1, r2, r3, r4 = calcular_velocidades_ruedas(vx, vy, w)
        # Mapear a rango [-255, 255]
        v1, v2, v3, v4 = mapear_velocidad(r1), mapear_velocidad(r2), mapear_velocidad(r3), mapear_velocidad(r4)

        # Enviar
        enviar_velocidades_udp(v1, v2, v3, v4)

        time.sleep(0.05)  # 20 Hz

except KeyboardInterrupt:
    enviar_velocidades_udp(0, 0, 0, 0)
    print("Robot detenido. Programa finalizado.")
