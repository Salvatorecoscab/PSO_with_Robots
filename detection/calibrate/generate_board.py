import cv2

# Configuracion del tablero
# 7 cuadrados de ancho, 5 de alto. 
# Importante: El diccionario debe coincidir con el que uses en tu codigo de deteccion.
arduino_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((7, 5), 0.04, 0.02, arduino_dict)

# Generar la imagen para imprimir
img = board.generateImage((2000, 1500))
cv2.imwrite("tablero_charuco.png", img)
print("Tablero guardado como tablero_charuco.png. ¡Imprímelo!")
