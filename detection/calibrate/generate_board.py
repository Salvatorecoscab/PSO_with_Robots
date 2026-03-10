import cv2


arduino_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((7, 5), 0.04, 0.02, arduino_dict)


img = board.generateImage((2000, 1500))
cv2.imwrite("charuco_board.png", img)
print("Board saved as charuco_board.png. Print it!")
