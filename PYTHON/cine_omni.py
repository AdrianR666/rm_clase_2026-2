import numpy as np
import socket
import time

# ========== Configuración de UDP ==========
UDP_IP = "192.168.137.140"  # IP del ESP32
UDP_PORT = 58625  # Puerto del ESP32
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ========== Cinemática Inversa ==========
def calcular_velocidades_ruedas(vx, vy, w):
    r = 0.030
    b = 0.1425
    d = 0.078
    rueda1 = (vx / r) - (vy / r) - (((b + d) / r) * w)
    rueda2 = (vx / r) + (vy / r) + (((b + d) / r) * w)
    rueda3 = (vx / r) + (vy / r) - (((b + d) / r) * w)
    rueda4 = (vx / r) - (vy / r) + (((b + d) / r) * w)
    return rueda1, rueda2, rueda3, rueda4

# ========== Mapear velocidad ==========
def mapear_velocidad_l298n(vel):
    return max(-255, min(255, int(vel)))

# ========== Enviar por UDP ==========
def enviar_velocidades_udp(v1, v2, v3, v4):
    try:
        data = np.array([v1, v2, v3, v4], dtype=np.int16).tobytes()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> v1: {v1}, v2: {v2}, v3: {v3}, v4: {v4}")
    except Exception as e:
        print(f"Error al enviar datos UDP: {e}")

# ========== Loop principal ==========
print("Introduce las velocidades vx, vy, w (m/s, rad/s). Enter para mantener el último valor. Ctrl+C para salir.\n")

vx, vy, w = 0.0, 0.0, 0.0  # Valores iniciales

try:
    while True:
        entrada_vx = input("vx: ")
        entrada_vy = input("vy: ")
        entrada_w = input("w: ")

        if entrada_vx.strip() != "":
            vx = float(entrada_vx)
        if entrada_vy.strip() != "":
            vy = float(entrada_vy)
        if entrada_w.strip() != "":
            w = float(entrada_w)

        # Calcular velocidades y enviar
        rueda1, rueda2, rueda3, rueda4 = calcular_velocidades_ruedas(vx, vy, w)
        v1 = mapear_velocidad_l298n(rueda1)
        v2 = mapear_velocidad_l298n(rueda2)
        v3 = mapear_velocidad_l298n(rueda3)
        v4 = mapear_velocidad_l298n(rueda4)

        enviar_velocidades_udp(v1, v2, v3, v4)
        time.sleep(0.1)

except KeyboardInterrupt:
    enviar_velocidades_udp(0, 0, 0, 0)
    print("\nPrograma detenido.")
