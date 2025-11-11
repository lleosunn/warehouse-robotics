import cv2
import cv2.aruco as aruco
import numpy as np

def create_detector_parameters():
    """Handle both new and old OpenCV ArUco APIs."""
    if hasattr(aruco, "DetectorParameters"):
        # OpenCV ≥ 4.10
        return aruco.DetectorParameters()
    elif hasattr(aruco, "DetectorParameters_create"):
        # Older OpenCV
        return aruco.DetectorParameters_create()
    else:
        raise RuntimeError("Cannot find DetectorParameters class in cv2.aruco")

def draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length=0.03):
    """Compatible replacement for aruco.drawAxis."""
    if hasattr(cv2, "drawFrameAxes"):
        # OpenCV ≥ 4.10
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length)
    elif hasattr(aruco, "drawAxis"):
        # Older OpenCV
        aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length)
    else:
        print("⚠️ drawAxis/drawFrameAxes not available in this OpenCV build.")

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Error: Could not open camera.")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = create_detector_parameters()

    # Example calibration (replace with your actual calibration for accuracy)
    camera_matrix = np.array([[1417.0403096571977, 0.0, 957.459776490542],
                              [0.0, 1420.2720211645985, 570.5911420017325],
                              [0.0, 0.0, 1.0]], dtype=np.float32)
    dist_coeffs = np.array([
        [0.08959465840442786, -0.750613499067229, 0.00020226506264608155, -0.009160126115283456, 2.7073995410808753]
    ])
    marker_size = 0.105 

    print("📷 Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to grab frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, corner in enumerate(corners):
                marker_id = int(ids[i])
                print(f"\n🟩 Detected Marker ID: {marker_id}")

                marker_points = np.array([
                    [-marker_size/2,  marker_size/2, 0],
                    [ marker_size/2,  marker_size/2, 0],
                    [ marker_size/2, -marker_size/2, 0],
                    [-marker_size/2, -marker_size/2, 0]
                ], dtype=np.float32)

                image_points = corner.reshape(-1, 2)
                success, rvec, tvec = cv2.solvePnP(marker_points, image_points,
                                                   camera_matrix, dist_coeffs)
                if not success:
                    print("  ❌ Pose estimation failed.")
                    continue

                draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec)

                x, y, z = tvec.flatten().tolist()
                print(f"  📍 Position (m): x={x:.3f}, y={y:.3f}, z={z:.3f}")

                # Convert rotation vector → Euler angles
                R, _ = cv2.Rodrigues(rvec)
                sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
                singular = sy < 1e-6
                if not singular:
                    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
                    pitch = np.degrees(np.arctan2(-R[2, 0], sy))
                    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                else:
                    roll = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
                    pitch = np.degrees(np.arctan2(-R[2, 0], sy))
                    yaw = 0
                print(f"  🔄 Orientation (°): roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}")

        else:
            cv2.putText(frame, "No markers detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("ArUco Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()