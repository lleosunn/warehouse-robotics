#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_matrix, euler_from_matrix

class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_node')

        # --- Publishers ---
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)

        # --- Camera setup ---
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open camera.")
            raise RuntimeError("Camera not accessible.")

        # --- ArUco setup ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = self.create_detector_parameters()

        # Example calibration (replace with your actual camera parameters)
        self.camera_matrix = np.array([[1417.0403096571977, 0.0, 957.459776490542],
                              [0.0, 1420.2720211645985, 570.5911420017325],
                              [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([
        [0.08959465840442786, -0.750613499067229, 0.00020226506264608155, -0.009160126115283456, 2.7073995410808753]
        ], dtype=np.float32)
        self.marker_size = 0.105

        # --- Run periodically (30 Hz) ---
        self.timer = self.create_timer(1/30.0, self.timer_callback)

        self.get_logger().info("📷 ArUco Pose Node started. Press Ctrl+C to stop.")

    def create_detector_parameters(self):
        """Handle both new and old OpenCV ArUco APIs."""
        if hasattr(aruco, "DetectorParameters"):
            return aruco.DetectorParameters()
        elif hasattr(aruco, "DetectorParameters_create"):
            return aruco.DetectorParameters_create()
        else:
            raise RuntimeError("Cannot find DetectorParameters in cv2.aruco")

    def draw_axes(self, img, corners, ids, rvecs, tvecs, camera_matrix, dist_coeffs):
        # Handle both new and old OpenCV APIs for drawAxis
        if hasattr(aruco, "drawAxis"):
            for rvec, tvec in zip(rvecs, tvecs):
                aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
        elif hasattr(aruco, "drawFrameAxes"):
            for rvec, tvec in zip(rvecs, tvecs):
                aruco.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            marker_points = np.array([
                [-self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2, -self.marker_size/2, 0],
                [-self.marker_size/2, -self.marker_size/2, 0]
            ], dtype=np.float32)

            rvecs = []
            tvecs = []

            for i, corner in enumerate(corners):
                image_points = corner.reshape(-1, 2)
                success, rvec, tvec = cv2.solvePnP(marker_points, image_points,
                                                   self.camera_matrix, self.dist_coeffs)
                if not success:
                    self.get_logger().warn("❌ Pose estimation failed.")
                    continue

                rvecs.append(rvec)
                tvecs.append(tvec)

                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Convert to quaternion
                M = np.eye(4)
                M[:3, :3] = R
                q = quaternion_from_matrix(M)

                # Compute Euler angles in radians and convert to degrees
                roll, pitch, yaw = euler_from_matrix(M)
                roll_deg = np.degrees(roll)
                pitch_deg = np.degrees(pitch)
                yaw_deg = np.degrees(yaw)

                # Build Pose message
                pose_msg = Pose()
                pose_msg.position.x = float(tvec[0])
                pose_msg.position.y = float(tvec[1])
                pose_msg.position.z = float(tvec[2])
                pose_msg.orientation.x = float(q[0])
                pose_msg.orientation.y = float(q[1])
                pose_msg.orientation.z = float(q[2])
                pose_msg.orientation.w = float(q[3])

                self.pose_pub.publish(pose_msg)

                # Extract position values correctly (tvec is 3x1 array)
                x, y, z = tvec.flatten()
                self.get_logger().info(f"🟩 Detected Marker ID: {int(ids[i])}")
                self.get_logger().info(f"Position (x, y, z): {x:.3f}, {y:.3f}, {z:.3f}")
                self.get_logger().info(f"Euler angles (deg) Roll: {roll_deg:.1f}, Pitch: {pitch_deg:.1f}, Yaw: {yaw_deg:.1f}")

            # Draw axes for all detected markers
            self.draw_axes(frame, corners, ids, rvecs, tvecs, self.camera_matrix, self.dist_coeffs)

        # Optional visualization (comment out for headless)
        cv2.imshow("ArUco Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()