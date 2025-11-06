import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
for i in range(5):
    img = aruco.generateImageMarker(aruco_dict, i, 150)
    cv2.imwrite(f"marker_{i}.png", img)