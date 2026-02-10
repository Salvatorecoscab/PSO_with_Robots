import cv2
import subprocess
import threading
import time

# Iniciar scrcpy grabando a archivo
scrcpy_proc = subprocess.Popen([
    'scrcpy',
    '--max-size=640',
    '--max-fps=30',
    '--no-audio',
    '--video-bit-rate=4M',
    '--record=stream.mp4'
], stderr=subprocess.DEVNULL)

# Esperar a que se cree el archivo
time.sleep(3)

# Leer el archivo mientras se está escribiendo
cap = cv2.VideoCapture('stream.mp4')

print("Iniciando captura...")

while True:
    ret, frame = cap.read()
    
    if not ret:
        # Si no hay frame, esperar un poco y continuar
        time.sleep(0.01)
        # Reintentar abrir el video por si hay más frames
        cap.set(cv2.CAP_PROP_POS_FRAMES, cap.get(cv2.CAP_PROP_POS_FRAMES))
        continue
    
    cv2.imshow('Android Camera', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
scrcpy_proc.terminate()
cv2.destroyAllWindows()

# Limpiar
import os
if os.path.exists('stream.mp4'):
    os.remove('stream.mp4')