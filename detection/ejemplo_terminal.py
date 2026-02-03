"""
Ejemplo simple: Terminal interactiva para consultar posiciones de marcadores.
"""

import time
from ArucoTracker import ArucoTracker

import serial
import time







def main():
    # Iniciar tracker
    tracker = ArucoTracker(
        marker_length=0.08,
        calib_file="calibracion_charuco.yml",
        border_ids={7, 8, 9, 10},
        border_order=[10, 9, 8, 7]
    )
    
    print("Iniciando tracker...")
    tracker.start(show_visualization=True)  # Cambia a False si no quieres ver la cámara
    time.sleep(1)
    
    # print("\nTracker activo. Comandos:")
    # print("  <número> - Ver marcador (ej: 1)")
    # print("  todos    - Ver todos")
    # print("  salir    - Terminar\n")
    # Configura el puerto correcto (COMx en Windows, /dev/ttyUSBx en Linux/Mac)
    puerto = '/dev/cu.usbserial-6' 
    baudios = 115200

    try:
        esp8266 = serial.Serial(puerto, baudios, timeout=1)
        time.sleep(2) # Espera a que se asiente la conexión
        print("Conectado al ESP8266. Escribe algo:")

        while True:
            marker = tracker.get_marker(int(3))
            if marker:
                print(f"ID {marker.id}:")
                print(f"  Posición: {marker.position}")
                print(f"  Ángulo: {marker.angle:.1f}°")
                print(f"  Distancia Z: {marker.z_distance:.3f}m")
            else:
                print(f"Marcador {3} no visible")
            mensaje = f"ID {marker.id}: Pos={marker.position}, Ángulo={marker.angle:.1f}°, Z={marker.z_distance:.3f}m" if marker else f"Marcador {3} no visible"
            esp8266.write((mensaje + '\n').encode('utf-8'))
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nPrograma finalizado.")
    except Exception as e:
        print(f"Error: {e}")

    finally:
        if 'esp8266' in locals():
            esp8266.close()
        tracker.stop()
        print("Tracker detenido")


if __name__ == "__main__":
    main()

