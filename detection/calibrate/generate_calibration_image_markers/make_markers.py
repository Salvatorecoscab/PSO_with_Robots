import cv2

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Tamaño en pixeles del PNG (más grande = mejor impresión)
marker_px = 500

for marker_id in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
    img = cv2.aruco.generateImageMarker(DICT, marker_id, marker_px)
    cv2.imwrite(f"aruco_{marker_id}.png", img)

print("Listo: arucos creados")
