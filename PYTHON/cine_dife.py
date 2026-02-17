# cine_dife.py  
import socket
import math

# Configuración del socket UDP
UDP_IP = "192.168.137.58"  # Dirección IP de la ESP32
UDP_PORT = 12345           # Puerto UDP para comunicación

# Cinemática del robot diferencial
R = 0.05    # Radio de las ruedas (m)
L = 0.20    # Distancia entre ruedas (m)

def enviar_velocidades(v, w):
    """
    Calcula la velocidad de cada rueda y la envía por UDP.
    """
    # Cinemática inversa
    v_r = (2 * v + w * L) / (2 * R)  # Velocidad rueda derecha
    v_l = (2 * v - w * L) / (2 * R)  # Velocidad rueda izquierda
    
    # Formateo de datos
    data = f"{v_r},{v_l}".encode()

    # Envío por UDP
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Velocidades enviadas: Derecha = {v_r} m/s, Izquierda = {v_l} m/s")

# Ejemplo de uso
if __name__ == "__main__":
    while True:
        try:
            v = float(input("Ingresa velocidad lineal (m/s): "))
            w = float(input("Ingresa velocidad angular (rad/s): "))
            enviar_velocidades(v, w)
        except ValueError:
            print("Entrada no válida. Intenta de nuevo.")

