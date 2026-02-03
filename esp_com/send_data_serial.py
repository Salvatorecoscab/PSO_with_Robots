try:
    import serial
except ImportError:
    print("Error: pyserial is not installed. Install it with 'pip install pyserial'.")
    exit(1)
import json

# Configura el puerto del ESP8266 conectado por USB
try:
    ser = serial.Serial('/dev/cu.usbserial-0001', 115200)
except AttributeError:
    print("Error: The 'serial' module does not have 'Serial'. This usually happens if you installed 'serial' instead of 'pyserial' or if there is a file named 'serial.py' in your directory.")
    exit(1)

def enviar_coordenadas(id_robot, x, y):
    data = {"id": id_robot, "x": x, "y": y}
    msg = json.dumps(data) + "\n"
    ser.write(msg.encode('utf-8'))

if __name__ == "__main__":
    # Ejemplo de envío de coordenadas
    enviar_coordenadas(1, 10.5, 20.3)
    enviar_coordenadas(2, 15.0, 25.7)