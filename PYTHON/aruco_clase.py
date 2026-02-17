import cv2
import numpy as np
import cv2.aruco as aruco

# Definir el diccionario de marcadores ArUco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Cargar los parámetros de detección
detector_params = aruco.DetectorParameters()

# Inicializar la captura de video
cap = cv2.VideoCapture(0)

# Definir matriz de cámara y coeficientes de distorsión (ajustar según la cámara usada)
# =====================================================
# ========== PARÁMETROS DE CÁMARA ======================
# =====================================================
camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([0.0906358995, 0.318998982,
                        -0.00288859938, 0.000180882259, -2.99885280])


# Diccionario para almacenar trayectorias por ID
trajectories = {}
colors = {}  # Colores asignados a cada marcador

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detectar los marcadores
    detector = aruco.ArucoDetector(dictionary, detector_params)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i, corner in enumerate(corners):
            marker_id = ids[i][0]
            
            # Dibujar los marcadores detectados
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Calcular el centro del marcador
            c_x = int(np.mean(corner[0][:, 0]))
            c_y = int(np.mean(corner[0][:, 1]))
            
            # Estimar posición y orientación
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)
            
            if rvecs is not None and tvecs is not None:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.05)
                pos_text = f"ID: {marker_id} Pos: ({tvecs[0][0][0]:.2f}, {tvecs[0][0][1]:.2f}, {tvecs[0][0][2]:.2f})"
                ori_text = f"Ori: ({rvecs[0][0][0]:.2f}, {rvecs[0][0][1]:.2f}, {rvecs[0][0][2]:.2f})"
                cv2.putText(frame, pos_text, (20, 40 + marker_id * 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, ori_text, (20, 60 + marker_id * 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Asignar color único si es un nuevo marcador
            if marker_id not in colors:
                colors[marker_id] = tuple(np.random.randint(0, 255, 3).tolist())
            
            # Guardar la trayectoria
            if marker_id not in trajectories:
                trajectories[marker_id] = []
            trajectories[marker_id].append((c_x, c_y))
            
            # Dibujar la trayectoria
            for j in range(1, len(trajectories[marker_id])):
                cv2.line(frame, trajectories[marker_id][j-1], trajectories[marker_id][j], colors[marker_id], 2)
            
            # Mostrar posición
            cv2.putText(frame, f"Pos: ({c_x}, {c_y})", (c_x+10, c_y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[marker_id], 2)
    
    # Mostrar el frame
    cv2.imshow('ArUco Tracking', frame)
    
    # Salir con la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
