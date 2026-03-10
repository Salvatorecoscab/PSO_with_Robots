import cv2
print(cv2.__version__)
import numpy as np
import time
print("cv2.aruco attributes:")
for attr in dir(cv2.aruco):
    if "calibrate" in attr.lower():
        print(attr)

# 1. Configuration and capture
charuco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((7, 5), 0.25, 0.1875, charuco_dict)
detector = cv2.aruco.CharucoDetector(board)

cap = cv2.VideoCapture(0)
all_charuco_corners = []
all_charuco_ids = []
image_size = None

print("Move the board around the screen. Press 'c' to capture frame, 'q' to calibrate.")
cont =0
while True:
    ret, frame = cap.read()
    if not ret or frame is None:
            print("WARN: frame not available, retrying...")
            time.sleep(0.05)
            continue  
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    # Detectar ChArUco
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    display_frame = frame.copy()
    if charuco_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(display_frame, charuco_corners, charuco_ids)

    cv2.imshow("Calibrating ChArUco", display_frame)
    
    key = cv2.waitKey(1)


    if key == ord('c') and charuco_ids is not None:
        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        print(f"Capture {len(all_charuco_corners)} saved.")
    
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 2. Final calibration
if len(all_charuco_corners) > 10:
    ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None
    )

    # Store calibration results
    cv_file = cv2.FileStorage("calibration_charuco.yml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    cv_file.release()
    print("Calibration successful. File 'calibration_charuco.yml' created.")
else:
    print("You need at least 10-15 captures for good accuracy.")