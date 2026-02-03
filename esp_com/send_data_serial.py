import serial
import time

# Configura el puerto correcto (COMx en Windows, /dev/ttyUSBx en Linux/Mac)
puerto = '/dev/cu.usbserial-6' 
baudios = 115200

try:
    esp8266 = serial.Serial(puerto, baudios, timeout=1)
    time.sleep(2) # Espera a que se asiente la conexión
    print("Conectado al ESP8266. Escribe algo:")

    while True:
        mensaje = input(">> ")
        esp8266.write((mensaje + '\n').encode('utf-8'))
        
except KeyboardInterrupt:
    print("\nPrograma finalizado.")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'esp8266' in locals():
        esp8266.close()