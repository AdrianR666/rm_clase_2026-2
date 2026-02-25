import cv2
import numpy as np

# --- Parámetros de cámara ---
camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([0.0906358995, 0.318998982,
                        -0.00288859938, 0.000180882259, -2.99885280])

# --- Configuración ---
marker_length = 0.1  # metros
ids_roles = {0: "Referencia", 1: "Robot", 2: "Objetivo"}

# --- Funciones auxiliares ---
def pose_to_marker0(rvec_i, tvec_i, rvec_0, tvec_0):
    """Transforma la pose del marcador i al marco del marcador 0"""
    R_i, _ = cv2.Rodrigues(rvec_i)
    R_0, _ = cv2.Rodrigues(rvec_0)

    R_0_i = R_0.T @ R_i
    t_0_i = R_0.T @ (tvec_i - tvec_0)

    return R_0_i, t_0_i

def calculate_distance_between_markers(rvec_a, tvec_a, rvec_b, tvec_b):
    """Calcula la distancia real entre dos marcadores en el marco de la cámara"""
    tvec_a = tvec_a.flatten()
    tvec_b = tvec_b.flatten()
    
    # Distancia euclidiana en el marco de la cámara
    distance = np.linalg.norm(tvec_b - tvec_a)
    return distance

def yaw_from_R(R):
    """Extrae el ángulo de guiñada (yaw) de una matriz de rotación"""
    return np.arctan2(R[1, 0], R[0, 0])

def draw_distance_info(frame, distances, centers_2d):
    """Dibuja información de distancias en el frame"""
    y_offset = 30
    
    # Título
    cv2.putText(
        frame,
        "DISTANCIAS ENTRE MARCADORES:",
        (10, y_offset),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )
    
    y_offset += 25
    for (id1, id2), distance in distances.items():
        distance_cm = distance * 100
        nombre1 = ids_roles.get(id1, f"ID{id1}")
        nombre2 = ids_roles.get(id2, f"ID{id2}")
        
        texto = f"{nombre1} -> {nombre2}: {distance_cm:.1f} cm"
        cv2.putText(
            frame,
            texto,
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1
        )
        y_offset += 20
        
        # Dibujar línea con el color correspondiente
        if id1 in centers_2d and id2 in centers_2d:
            if (id1, id2) == (0, 1) or (id2, id1) == (0, 1):
                color = (0, 255, 0)  # Verde para Referencia-Robot
            elif (id1, id2) == (0, 2) or (id2, id1) == (0, 2):
                color = (255, 0, 0)  # Azul para Referencia-Objetivo
            elif (id1, id2) == (1, 2) or (id2, id1) == (1, 2):
                color = (0, 0, 255)  # Rojo para Robot-Objetivo
            else:
                color = (255, 255, 0)  # Amarillo para otros
            
            cv2.line(frame, centers_2d[id1], centers_2d[id2], color, 2)

# --- Inicializa cámara ---
cap = cv2.VideoCapture(1)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

print("=== MEDIDOR DE DISTANCIAS ENTRE MARCADORES ===")
print(f"Tamaño del marcador: {marker_length*100} cm")
print("Marcadores esperados: ID 0 (Referencia), ID 1 (Robot), ID 2 (Objetivo)")
print("Presiona ESC para salir")
print("-" * 40)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)

    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        poses_3d = {}
        centers_2d = {}
        distancias = {}  # Diccionario para almacenar distancias entre pares

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i]
            tvec = tvecs[i]

            cv2.drawFrameAxes(frame, camera_matrix,
                              dist_coeffs, rvec, tvec, 0.03)

            # Obtener centro del marcador para visualización
            imgpt, _ = cv2.projectPoints(
                np.array([[0, 0, 0]], dtype=np.float32),
                rvec, tvec, camera_matrix, dist_coeffs
            )

            centers_2d[marker_id] = tuple(imgpt[0][0].astype(int))
            poses_3d[marker_id] = (rvec, tvec)

            # Mostrar ID y rol
            rol = ids_roles.get(marker_id, "Desconocido")
            cv2.putText(
                frame,
                f"{rol}",
                (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                2
            )

        # --- CALCULAR DISTANCIAS ENTRE TODOS LOS PARES DE MARCADORES ---
        if len(poses_3d) >= 2:
            # Obtener todos los IDs presentes
            ids_presentes = list(poses_3d.keys())
            
            # Calcular distancia para cada par único
            for i in range(len(ids_presentes)):
                for j in range(i+1, len(ids_presentes)):
                    id1 = ids_presentes[i]
                    id2 = ids_presentes[j]
                    
                    rvec1, tvec1 = poses_3d[id1]
                    rvec2, tvec2 = poses_3d[id2]
                    
                    distancia = calculate_distance_between_markers(
                        rvec1, tvec1, rvec2, tvec2
                    )
                    
                    distancias[(id1, id2)] = distancia
                    
                    # Mostrar en consola si es el par principal
                    if (id1 == 0 and id2 == 1) or (id1 == 1 and id2 == 0):
                        print(f"Distancia Referencia-Robot: {distancia*100:.1f} cm")
                    elif (id1 == 0 and id2 == 2) or (id1 == 2 and id2 == 0):
                        print(f"Distancia Referencia-Objetivo: {distancia*100:.1f} cm")
                    elif (id1 == 1 and id2 == 2) or (id1 == 2 and id2 == 1):
                        print(f"Distancia Robot-Objetivo: {distancia*100:.1f} cm")

        # --- MEDIDAS EN EL MARCO DEL MARCADOR 0 (tu código original) ---
        if all(k in poses_3d for k in [0, 1, 2]):
            rvec_0, tvec_0 = poses_3d[0]

            R0_1, t0_1 = pose_to_marker0(
                poses_3d[1][0], poses_3d[1][1], rvec_0, tvec_0
            )
            R0_2, t0_2 = pose_to_marker0(
                poses_3d[2][0], poses_3d[2][1], rvec_0, tvec_0
            )

            # Distancias reales en marco del marcador 0
            d01 = np.linalg.norm(t0_1)
            d02 = np.linalg.norm(t0_2)

            # Mostrar información adicional en el frame
            y_offset = 150
            cv2.putText(
                frame,
                f"En marco del marcador 0:",
                (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (200, 200, 200),
                2
            )
            cv2.putText(
                frame,
                f"Robot: {d01*100:.1f} cm",
                (10, y_offset + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )
            cv2.putText(
                frame,
                f"Objetivo: {d02*100:.1f} cm",
                (10, y_offset + 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1
            )

        # Dibujar toda la información de distancias
        if distancias:
            draw_distance_info(frame, distancias, centers_2d)

        # Dibujar marcadores detectados
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Mostrar contador de marcadores detectados
    num_markers = len(centers_2d) if 'centers_2d' in locals() else 0
    cv2.putText(
        frame,
        f"Marcadores: {num_markers}",
        (frame.shape[1] - 150, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 0),
        2
    )

    cv2.imshow("Sistema ArUco – Medición de distancias", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

print("\n--- PROGRAMA FINALIZADO ---")
cap.release()
cv2.destroyAllWindows()