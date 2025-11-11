#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

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

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (in radians) to quaternion (x, y, z, w)."""
    # Roll (x-axis rotation)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        # --- Publishers ---
        self.pose_pub = self.create_publisher(Pose, '/aruco_pose', 10)

        # --- Camera setup ---
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Error: Could not open camera.")
            raise RuntimeError("Camera not accessible.")

        # --- ArUco setup ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = create_detector_parameters()

        # Example calibration (replace with your actual calibration for accuracy)
        self.camera_matrix = np.array([[1417.0403096571977, 0.0, 957.459776490542],
                                      [0.0, 1420.2720211645985, 570.5911420017325],
                                      [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([
            [0.08959465840442786, -0.750613499067229, 0.00020226506264608155, -0.009160126115283456, 2.7073995410808753]
        ])
        self.marker_size = 0.105

        # --- Run periodically (same rate as camera capture) ---
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 Hz

        self.get_logger().info("📷 ArUco Detection Node started. Press Ctrl+C to stop.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, corner in enumerate(corners):
                marker_id = int(ids[i])
                self.get_logger().info(f"\n🟩 Detected Marker ID: {marker_id}")

                marker_points = np.array([
                    [-self.marker_size/2,  self.marker_size/2, 0],
                    [ self.marker_size/2,  self.marker_size/2, 0],
                    [ self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)

                image_points = corner.reshape(-1, 2)
                success, rvec, tvec = cv2.solvePnP(marker_points, image_points,
                                                   self.camera_matrix, self.dist_coeffs)
                if not success:
                    self.get_logger().warn("  ❌ Pose estimation failed.")
                    continue

                # Draw axes exactly as in standalone script (axis_length=0.03)
                draw_axes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length=0.03)

                x, y, z = tvec.flatten().tolist()
                self.get_logger().info(f"  📍 Position (m): x={x:.3f}, y={y:.3f}, z={z:.3f}")

                # Convert rotation vector → Euler angles (same calculation as standalone)
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
                self.get_logger().info(f"  🔄 Orientation (°): roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}")

                # Convert Euler angles to quaternion for ROS message
                roll_rad = np.radians(roll)
                pitch_rad = np.radians(pitch)
                yaw_rad = np.radians(yaw)
                qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

                # Publish pose message
                pose_msg = Pose()
                pose_msg.position.x = float(x)
                pose_msg.position.y = float(y)
                pose_msg.position.z = float(z)
                pose_msg.orientation.x = float(qx)
                pose_msg.orientation.y = float(qy)
                pose_msg.orientation.z = float(qz)
                pose_msg.orientation.w = float(qw)
                self.pose_pub.publish(pose_msg)

        else:
            cv2.putText(frame, "No markers detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("ArUco Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

