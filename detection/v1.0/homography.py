import cv2
import numpy as np
import time
import os

# --- CONFIGURACIÓN ---
MARKER_LENGTH = 0.08  # 8 cm en metros
ARUCO_DICT = cv2.aruco.DICT_4X4_50
CALIB_FILE = "calibracion_charuco.yml"

# IDs de los marcadores que delimitan el espacio
BORDER_IDS = {7, 8, 9, 10}
BORDER_ORDER = [10, 9, 8, 7]  # <-- ajusta esto según tu disposición física
# Orden esperado de los marcadores delimitadores (esquina sup-izq, sup-der, inf-der, inf-izq)
# Este orden DEBE coincidir con la disposición física de tus marcadores.
# Lo ajustas según cómo los tengas colocados.
# BORDER_ORDER = [10, 9, 8, 7]  # <-- ajusta esto según tu disposición física

def load_calibration(file_path):
    if not os.path.exists(file_path):
        print(f"ERROR: No se encuentra el archivo {file_path}")
        return None, None
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_coeffs = cv_file.getNode("dist_coeff").mat()
    cv_file.release()
    return camera_matrix, dist_coeffs


def get_marker_center(corner):
    """Retorna el centro de un marcador a partir de sus 4 esquinas (en píxeles)."""
    return corner.reshape(4, 2).mean(axis=0)


def compute_homography(border_centers_ordered, dst_size=(640, 480)):
    """
    Calcula la homografía desde los 4 centros de los marcadores delimitadores
    hacia un rectángulo de destino de tamaño dst_size.

    border_centers_ordered: lista de 4 puntos [sup-izq, sup-der, inf-der, inf-izq] en píxeles
    dst_size: (ancho, alto) del rectángulo destino en píxeles
    """
    w, h = dst_size
    # Esquinas del rectángulo destino (normalizado al tamaño dado)
    dst_points = np.array([
        [0, 0],       # sup-izq
        [w - 1, 0],   # sup-der
        [w - 1, h - 1],  # inf-der
        [0, h - 1]    # inf-izq
    ], dtype=np.float32)

    src_points = np.array(border_centers_ordered, dtype=np.float32)
    H, _ = cv2.findHomography(src_points, dst_points)
    return H


def apply_homography_to_point(H, point_2d):
    """
    Aplica la homografía a un punto 2D (píxeles) y retorna su posición
    en el sistema de coordenadas del espacio delimitado.

    point_2d: (x, y) en píxeles de la imagen original
    Retorna: (x', y') en el sistema del rectángulo destino
    """
    # Formato homogéneo: [x, y, 1]
    pt = np.array([point_2d[0], point_2d[1], 1.0], dtype=np.float64)
    transformed = H @ pt
    # Dividir por la coordenada homogénea w
    transformed /= transformed[2]
    return transformed[0], transformed[1]


def main():
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILE)
    if camera_matrix is None:
        return

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    obj_points = np.array([
        [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
        [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
        [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
        [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0]
    ], dtype=np.float32)

    # Tamaño del rectángulo virtual donde se mapea el espacio
    DEST_W, DEST_H = 640, 480

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("WARN: frame no disponible, reintentando...")
            time.sleep(0.1)
            continue

        corners, ids, _ = detector.detectMarkers(frame)

        # --- Diccionario: id -> (corner, tvec) ---
        detected = {}
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                ret_pnp, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i], camera_matrix, dist_coeffs
                )
                if ret_pnp:
                    detected[marker_id] = {
                        "corner": corners[i],
                        "rvec": rvec,
                        "tvec": tvec
                    }
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --- PASO 1: Verificar si los 4 delimitadores están presentes ---
        border_detected = all(bid in detected for bid in BORDER_ORDER)

        H = None  # Homografía (se calcula solo si los 4 están visibles)

        if border_detected:
            # Obtener centros ordenados según BORDER_ORDER
            border_centers = [
                get_marker_center(detected[bid]["corner"]) for bid in BORDER_ORDER
            ]
            H = compute_homography(border_centers, dst_size=(DEST_W, DEST_H))

            # Dibujar el polígono del espacio delimitado en la imagen
            poly = np.array(border_centers, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [poly], isClosed=True, color=(0, 255, 255), thickness=2)

        # --- PASO 2: Para los marcadores interiores, aplicar homografía ---
        interior_ids = [mid for mid in detected if mid not in BORDER_IDS]

        for mid in interior_ids:
            marker_corners = detected[mid]["corner"].reshape(4, 2)
            c0 = marker_corners[0] # Superior Izquierda
            c1 = marker_corners[1] # Superior Derecha
            center = get_marker_center(detected[mid]["corner"])
            x, y, z = detected[mid]["tvec"].flatten()
            # Centro del borde frontal
            front_midpoint = (c0 + c1) / 2

            # 3. Calcular el vector director respecto al centro del marcador
            v_x = front_midpoint[0] - center[0]
            v_y = front_midpoint[1] - center[1]
            angle_rad = np.arctan2(-v_y, v_x)
            angle_deg = np.degrees(angle_rad)

            # 5. (Opcional) Normalizar a 0-360°
            if angle_deg < 0:
                angle_deg += 360
            # Texto base: posición 3D respecto a la cámara
            text_base = f"ID:{mid} Z:{z:.3f}m Lat:{x:.3f}m"

            if H is not None:
                # Mapear el centro del marcador interior al espacio delimitado
                mx, my = apply_homography_to_point(H, center)

                # Coordenadas normalizadas (0 a 1) dentro del espacio
                nx = mx / (DEST_W - 1)
                ny = my / (DEST_H - 1)

                text_homo = f" | Pos:[{nx:.2f}, {ny:.2f}], Angle:{angle_deg:.1f}°"
                text_base += text_homo
                
                # Dibujar punto en la imagen original
                cv2.circle(frame, (int(center[0]), int(center[1])), 8, (0, 0, 255), -1)
            else:
                text_base += " | Sin espacio definido"

            # print(text_base)
            cv2.putText(frame, text_base, (10, 30 + (interior_ids.index(mid) * 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mensaje de estado de los delimitadores
        if not border_detected:
            missing = [bid for bid in BORDER_ORDER if bid not in detected]
            cv2.putText(frame, f"Falta ver marcadores: {missing}", (10, frame.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("Robot Vision - Homografía", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()




