import cv2
import numpy as np
import multiprocessing as mp
import time
import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List


@dataclass
class MarkerData:
    """Clase para almacenar los datos de un marcador detectado."""
    id: int
    position: Tuple[float, float]
    angle: float
    z_distance: float
    lateral_distance: float
    center_pixels: Tuple[float, float]
    timestamp: float

    def __repr__(self):
        return (f"Marker(id={self.id}, pos=({self.position[0]:.2f}, {self.position[1]:.2f}), "
                f"angle={self.angle:.1f}°, z={self.z_distance:.3f}m)")


class ArucoTracker:
    """
    Sistema de tracking de marcadores ArUco con homografía.
    Usa un proceso separado para la cámara - compatible con macOS.
    """

    def __init__(self, 
                 marker_length: float = 0.08,
                 aruco_dict: int = cv2.aruco.DICT_4X4_50,
                 calib_file: str = "calibracion_charuco.yml",
                 border_ids: set = {7, 8, 9, 10},
                 border_order: list = [10, 9, 8, 7],
                 dest_size: tuple = (640, 480),
                 camera_index: int = 0):
        
        self.marker_length = marker_length
        self.aruco_dict = aruco_dict
        self.calib_file = calib_file
        self.border_ids = border_ids
        self.border_order = border_order
        self.dest_w, self.dest_h = dest_size
        self.camera_index = camera_index

        # Manager para compartir datos entre procesos
        self._manager = mp.Manager()
        self._markers_dict = self._manager.dict()
        self._running = self._manager.Value('b', False)
        self._show_viz = self._manager.Value('b', False)
        
        self._process = None

    @staticmethod
    def _camera_process(markers_dict, running, show_viz, config):
        """Proceso separado que maneja la cámara y detección."""
        
        # Desempaquetar configuración
        marker_length = config['marker_length']
        aruco_dict = config['aruco_dict']
        calib_file = config['calib_file']
        border_ids = config['border_ids']
        border_order = config['border_order']
        dest_w = config['dest_w']
        dest_h = config['dest_h']
        camera_index = config['camera_index']

        # Cargar calibración
        if not os.path.exists(calib_file):
            print(f"ERROR: No se encuentra {calib_file}")
            return
        
        cv_file = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_coeffs = cv_file.getNode("dist_coeff").mat()
        cv_file.release()

        # Configurar detector
        dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        obj_points = np.array([
            [-marker_length/2,  marker_length/2, 0],
            [ marker_length/2,  marker_length/2, 0],
            [ marker_length/2, -marker_length/2, 0],
            [-marker_length/2, -marker_length/2, 0]
        ], dtype=np.float32)

        # Abrir cámara
        cap = cv2.VideoCapture(camera_index)
        
        while running.value:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.1)
                continue

            corners, ids, _ = detector.detectMarkers(frame)

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
                        if show_viz.value:
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                            rvec, tvec, 0.05)

                if show_viz.value:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Verificar delimitadores
            border_detected = all(bid in detected for bid in border_order)
            H = None

            if border_detected:
                border_centers = [
                    corners.reshape(4, 2).mean(axis=0) 
                    for bid in border_order 
                    for corners in [detected[bid]["corner"]]
                ]
                
                dst_points = np.array([
                    [0, 0], [dest_w-1, 0], [dest_w-1, dest_h-1], [0, dest_h-1]
                ], dtype=np.float32)
                
                src_points = np.array(border_centers, dtype=np.float32)
                H, _ = cv2.findHomography(src_points, dst_points)

                if show_viz.value:
                    poly = np.array(border_centers, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(frame, [poly], isClosed=True, color=(0, 255, 255), thickness=2)

            # Procesar marcadores interiores
            interior_ids = [mid for mid in detected if mid not in border_ids]
            current_time = time.time()

            # Limpiar diccionario compartido
            markers_dict.clear()

            for mid in interior_ids:
                marker_corners = detected[mid]["corner"].reshape(4, 2)
                center = marker_corners.mean(axis=0)
                x, y, z = detected[mid]["tvec"].flatten()

                # Calcular ángulo
                c0, c1 = marker_corners[0], marker_corners[1]
                front_mid = (c0 + c1) / 2
                v_x = front_mid[0] - center[0]
                v_y = front_mid[1] - center[1]
                angle = np.degrees(np.arctan2(-v_y, v_x))
                if angle < 0:
                    angle += 360

                if H is not None:
                    # Aplicar homografía
                    pt = np.array([center[0], center[1], 1.0])
                    transformed = H @ pt
                    transformed /= transformed[2]
                    nx = transformed[0] / (dest_w - 1)
                    ny = transformed[1] / (dest_h - 1)

                    # Guardar en diccionario compartido
                    markers_dict[mid] = {
                        'id': mid,
                        'position': (nx, ny),
                        'angle': angle,
                        'z_distance': z,
                        'lateral_distance': x,
                        'center_pixels': (float(center[0]), float(center[1])),
                        'timestamp': current_time
                    }

                    if show_viz.value:
                        cv2.circle(frame, (int(center[0]), int(center[1])), 8, (0, 0, 255), -1)
                        text = f"ID:{mid} Pos:[{nx:.2f},{ny:.2f}] A:{angle:.1f}°"
                        cv2.putText(frame, text, (10, 30 + interior_ids.index(mid)*25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if show_viz.value:
                if not border_detected:
                    missing = [bid for bid in border_order if bid not in detected]
                    cv2.putText(frame, f"Falta: {missing}", (10, frame.shape[0]-20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.imshow("ArUco Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    running.value = False

        cap.release()
        if show_viz.value:
            cv2.destroyAllWindows()

    def start(self, show_visualization: bool = False):
        """Inicia el tracker en un proceso separado."""
        if self._process is not None and self._process.is_alive():
            print("WARN: El tracker ya está en ejecución")
            return

        self._running.value = True
        self._show_viz.value = show_visualization

        config = {
            'marker_length': self.marker_length,
            'aruco_dict': self.aruco_dict,
            'calib_file': self.calib_file,
            'border_ids': self.border_ids,
            'border_order': self.border_order,
            'dest_w': self.dest_w,
            'dest_h': self.dest_h,
            'camera_index': self.camera_index
        }

        self._process = mp.Process(
            target=self._camera_process,
            args=(self._markers_dict, self._running, self._show_viz, config),
            daemon=True
        )
        self._process.start()

    def stop(self):
        """Detiene el tracker."""
        if self._process is None:
            return

        self._running.value = False
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
        self._process = None

    def get_marker(self, marker_id: int) -> Optional[MarkerData]:
        """Obtiene los datos de un marcador específico."""
        data = self._markers_dict.get(marker_id)
        if data is None:
            return None
        return MarkerData(**data)

    def get_all_markers(self) -> Dict[int, MarkerData]:
        """Obtiene todos los marcadores detectados."""
        return {
            mid: MarkerData(**data) 
            for mid, data in self._markers_dict.items()
        }


    def get_marker_position(self, marker_id: int) -> Optional[Tuple[float, float]]:
        """Obtiene la posición normalizada de un marcador."""
        data = self._markers_dict.get(marker_id)
        return data['position'] if data else None

    def get_marker_angle(self, marker_id: int) -> Optional[float]:
        """Obtiene el ángulo de orientación de un marcador."""
        data = self._markers_dict.get(marker_id)
        return data['angle'] if data else None

    def is_marker_visible(self, marker_id: int) -> bool:
        """Verifica si un marcador está siendo detectado."""
        return marker_id in self._markers_dict

    def __enter__(self):
        """Soporte para context manager."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Soporte para context manager."""
        self.stop()


# if __name__ == "__main__":
#     # Ejemplo básico de uso
#     tracker = ArucoTracker(
#         marker_length=0.08,
#         calib_file="calibracion_charuco.yml",
#         border_ids={7, 8, 9, 10},
#         border_order=[10, 9, 8, 7]
#     )

#     # Iniciar con o sin visualización
#     tracker.start(show_visualization=True)

#     try:
#         while True:
#             time.sleep(0.5)
            
#             # Obtener marcadores
#             markers = tracker.get_all_markers()
            
#             if markers:
#                 for marker_id, data in markers.items():
#                     print(f"ID {marker_id}: Pos={data.position}, Ángulo={data.angle:.1f}°")
            
#             # O consultar uno específico
#             marker_1 = tracker.get_marker(1)
#             if marker_1:
#                 print(f"Marcador 1 en: {marker_1.position}")

#     except KeyboardInterrupt:
#         print("\nDeteniendo...")
#     finally:
#         tracker.stop()