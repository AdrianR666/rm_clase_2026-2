# xbox_dife.py
import socket
import struct
import pygame

# ========== Configuración UDP ==========
UDP_IP = "192.168.137.102"   # IP de tu ESP32
UDP_PORT = 12345             # Puerto de escucha en ESP32

# Cinemática del robot diferencial
R = 0.05   # Radio de rueda (m)
L = 0.20   # Distancia entre ruedas (m)

# Inicializar joystick con pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def enviar_velocidades(v, w):
    # Cinemática diferencial
    v_r = (2 * v + w * L) / (2 * R)   # rueda derecha
    v_l = (2 * v - w * L) / (2 * R)   # rueda izquierda

    # Escalar a PWM [-255, 255]
    pwm_r = max(-255, min(255, int(v_r * 100)))
    pwm_l = max(-255, min(255, int(v_l * 100)))

    # Formateo de datos
    data = f"{pwm_r},{pwm_l}".encode()

    # Envío por UDP
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Velocidades enviadas: Derecha = {pwm_r} m/s, Izquierda = {pwm_l} m/s")

# ========== Loop principal ==========
print("Control Xbox listo. Joystick izquierdo = avanzar/retroceder, derecho = girar")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Leer ejes del control Xbox
    vx = -joystick.get_axis(1)   # Eje Y joystick izq (adelante/atrás)
    w  =  joystick.get_axis(3)   # Eje X joystick der (giro)

    # Escalamos velocidades
    v = vx * 0.2    # velocidad lineal máx 0.5 m/s
    w = w * 1.0     # velocidad angular máx 1 rad/s

    # Enviar velocidades
    enviar_velocidades(v, w)

    pygame.time.wait(100)
