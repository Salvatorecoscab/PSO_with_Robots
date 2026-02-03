import cv2
import numpy as np
import time
import os

# --- CONFIGURACIÓN ---
MARKER_LENGTH = 0.08  # 8 cm convertidos a metros
ARUCO_DICT = cv2.aruco.DICT_4X4_50
CALIB_FILE = "calibracion_charuco.yml"

def load_calibration(file_path):
    if not os.path.exists(file_path):
        print(f"ERROR: No se encuentra el archivo {file_path}")
        return None, None
    
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_coeffs = cv_file.getNode("dist_coeff").mat()
    cv_file.release()
    return camera_matrix, dist_coeffs

def main():
    # 1. Cargar la calibración real
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILE)
    if camera_matrix is None: return

    # 2. Configurar Detector
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    # 3. Puntos del marcador en el mundo real (3D)
    # Definimos el centro del marcador como el origen (0,0,0)
    obj_points = np.array([
        [-MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
        [ MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
        [ MARKER_LENGTH/2, -MARKER_LENGTH/2, 0],
        [-MARKER_LENGTH/2, -MARKER_LENGTH/2, 0]
    ], dtype=np.float32)

    cap = cv2.VideoCapture(0) # O tu URL de cámara IP

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("WARN: frame no disponible, reintentando...")
            time.sleep(0.1)
            continue
        

        corners, ids, _ = detector.detectMarkers(frame)
        all_tvecs = []
        if ids is not None:
            for i in range(len(ids)):
                # 4. Pose Estimation con datos de calibración reales
                ret_pnp, rvec, tvec = cv2.solvePnP(obj_points, corners[i], camera_matrix, dist_coeffs)

                if ret_pnp:
                    all_tvecs.append(tvec)
                    # Dibujar ejes (el tamaño del eje es 0.05m = 5cm)
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    # Coordenadas en metros
                    x, y, z = tvec.flatten()
                    
                    # --- LÓGICA PARA EL ROBOT ---
                    # Z es la distancia frontal al marcador
                    # X es el desplazamiento lateral (positivo a la derecha)
                    distancia_total = np.sqrt(x**2 + y**2 + z**2)

                    text = f"ID: {ids[i][0]} | Z: {z:.3f}m | Lat: {x:.3f}m"
                    print (text)
                    cv2.putText(frame, text, (10, 30 + (i*25)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Robot Vision - Calibrado", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()