import cv2
print(f"OpenCV version: {cv2.__version__}")
print("cv2.aruco attributes:")
for attr in dir(cv2.aruco):
    if "calibrate" in attr.lower():
        print(attr)