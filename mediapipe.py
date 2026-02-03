import cv2
import numpy as np
from collections import deque
import math
import time

# -----------------------------
# Configuración
# -----------------------------
TARGET_FREQS = [2.0, 3.5, 5.5]   # Hz (deben coincidir con los ESP8266)
FREQ_LABELS  = ["ESP1", "ESP2", "ESP3"]

# Ventana temporal para estimar frecuencia:
WINDOW_SECONDS = 3.0            # 2-4s suele ir bien
MAX_TRACK_DIST = 40             # px, para asociar blobs entre frames
MIN_BLOB_AREA  = 3
MAX_BLOB_AREA  = 5000

# Umbral de brillo para encontrar LEDs (ajustar):
THRESH_VALUE = 240              # 0-255 (si el LED es muy brillante en escena)
USE_ADAPTIVE = False            # si hay variaciones fuertes de iluminación, puedes probar True

# -----------------------------
# Goertzel: energía en una frecuencia objetivo
# -----------------------------
def goertzel_power(x, fs, f):
    """
    x: señal 1D (float), ya centrada (media ~0)
    fs: frecuencia de muestreo (FPS)
    f: frecuencia objetivo (Hz)
    Devuelve una medida de potencia en esa frecuencia.
    """
    n = len(x)
    if n < 8 or fs <= 0:
        return 0.0

    # Frecuencia digital
    w = 2.0 * math.pi * f / fs
    cos_w = math.cos(w)
    sin_w = math.sin(w)

    s_prev = 0.0
    s_prev2 = 0.0
    for sample in x:
        s = sample + 2.0 * cos_w * s_prev - s_prev2
        s_prev2 = s_prev
        s_prev = s

    # Magnitud^2 aproximada
    real = s_prev - s_prev2 * cos_w
    imag = s_prev2 * sin_w
    return real*real + imag*imag

# -----------------------------
# Estructura de track
# -----------------------------
class Track:
    def __init__(self, tid, cx, cy, maxlen):
        self.id = tid
        self.cx = cx
        self.cy = cy
        self.signal = deque(maxlen=maxlen)  # intensidad media alrededor del blob
        self.last_seen = time.time()
        self.label = "?"
        self.best_freq = None
        self.score = 0.0

def associate_tracks(tracks, detections, max_dist):
    """
    tracks: lista Track
    detections: lista (cx, cy, intensity)
    retorna: asignaciones (track_index -> det_index), dets_no_asignadas
    """
    if not tracks:
        return {}, list(range(len(detections)))

    assigned = {}
    used_dets = set()

    for ti, tr in enumerate(tracks):
        best_di = None
        best_d = 1e9
        for di, (cx, cy, inten) in enumerate(detections):
            if di in used_dets:
                continue
            d = (tr.cx - cx)**2 + (tr.cy - cy)**2
            if d < best_d:
                best_d = d
                best_di = di
        if best_di is not None and math.sqrt(best_d) <= max_dist:
            assigned[ti] = best_di
            used_dets.add(best_di)

    unassigned = [i for i in range(len(detections)) if i not in used_dets]
    return assigned, unassigned

# -----------------------------
# Main
# -----------------------------
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la cámara (VideoCapture(0)).")

    # Intentar fijar FPS (no siempre funciona)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Intentar reducir variaciones (dependiente del driver):
    # En algunas cámaras: 0.25 = manual, 0.75 = auto (puede variar)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    # cap.set(cv2.CAP_PROP_EXPOSURE, -6)

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps is None or fps <= 1:
        fps = 30.0

    maxlen = int(WINDOW_SECONDS * fps)
    if maxlen < 20:
        maxlen = 20

    tracks = []
    next_id = 1

    print(f"FPS usado: {fps:.2f}, ventana: {maxlen} muestras (~{maxlen/fps:.2f}s)")
    print("Teclas: q para salir")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("WARN: frame no disponible, reintentando...")
            time.sleep(0.05)
            continue    
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detección de puntos brillantes (LEDs)
        if USE_ADAPTIVE:
            th = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
                31, -10
            )
        else:
            _, th = cv2.threshold(gray, THRESH_VALUE, 255, cv2.THRESH_BINARY)

        # Limpieza morfológica
        th = cv2.medianBlur(th, 5)
        th = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=1)

        # Contornos / blobs
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = int(x + w / 2)
            cy = int(y + h / 2)

            # Intensidad promedio local (en gris) en una ROI pequeña
            r = 6
            x0 = max(0, cx - r); x1 = min(gray.shape[1], cx + r)
            y0 = max(0, cy - r); y1 = min(gray.shape[0], cy + r)
            roi = gray[y0:y1, x0:x1]
            inten = float(np.mean(roi)) if roi.size else float(gray[cy, cx])

            detections.append((cx, cy, inten))

        # Asociar detecciones a tracks existentes
        assigned, unassigned = associate_tracks(tracks, detections, MAX_TRACK_DIST)

        now = time.time()

        # Actualizar tracks asignados
        for ti, di in assigned.items():
            cx, cy, inten = detections[di]
            tr = tracks[ti]
            tr.cx, tr.cy = cx, cy
            tr.signal.append(inten)
            tr.last_seen = now

        # Crear nuevos tracks para detecciones no asignadas
        for di in unassigned:
            cx, cy, inten = detections[di]
            tr = Track(next_id, cx, cy, maxlen=maxlen)
            tr.signal.append(inten)
            tr.last_seen = now
            tracks.append(tr)
            next_id += 1

        # Eliminar tracks viejos
        tracks = [t for t in tracks if (now - t.last_seen) < 1.0]  # 1s sin ver -> borrar

        # Clasificación por frecuencia (Goertzel)
        for tr in tracks:
            if len(tr.signal) < int(0.6 * maxlen):  # espera a tener suficiente señal
                tr.label = "..."
                tr.best_freq = None
                tr.score = 0.0
                continue

            x = np.array(tr.signal, dtype=np.float32)
            x = x - np.mean(x)  # quitar DC
            # Ventana para reducir fuga espectral
            win = np.hanning(len(x)).astype(np.float32)
            xw = x * win

            powers = [goertzel_power(xw, fps, f) for f in TARGET_FREQS]
            best_i = int(np.argmax(powers))
            tr.best_freq = TARGET_FREQS[best_i]
            tr.score = powers[best_i]
            tr.label = f"{FREQ_LABELS[best_i]} ({TARGET_FREQS[best_i]:.1f}Hz)"

        # Dibujar
        vis = frame.copy()
        for tr in tracks:
            cv2.circle(vis, (tr.cx, tr.cy), 10, (0, 255, 0), 2)
            cv2.putText(vis, tr.label, (tr.cx + 12, tr.cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Blink tracker", vis)
        # (opcional) ver máscara:
        # cv2.imshow("th", th)
        print()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()