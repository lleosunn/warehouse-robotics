#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_matrix

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
        self.camera_matrix = np.array([[533.0, 0, 960],
                                       [0, 533.0, 540],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.marker_size = 0.127  # meters

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

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for i, corner in enumerate(corners):
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
                    self.get_logger().warn("❌ Pose estimation failed.")
                    continue

                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Convert to quaternion
                M = np.eye(4)
                M[:3, :3] = R
                q = quaternion_from_matrix(M)

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
                self.get_logger().info(f"🟩 Marker {int(ids[i])} | "
                                       f"x={tvec[0][0]:.3f}, y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}")

                # Draw coordinate axes arrows on the image
                axis_len = 0.03  # meters (match reference visualization)
                axis_points = np.array([
                    [0.0, 0.0, 0.0],           # origin
                    [axis_len, 0.0, 0.0],       # X axis end
                    [0.0, axis_len, 0.0],       # Y axis end
                    [0.0, 0.0, axis_len]        # Z axis end
                ], dtype=np.float32)

                imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                o = tuple(imgpts[0].ravel().astype(int))
                x = tuple(imgpts[1].ravel().astype(int))
                y = tuple(imgpts[2].ravel().astype(int))
                z = tuple(imgpts[3].ravel().astype(int))

                cv2.arrowedLine(frame, o, x, (0, 0, 255), 2, tipLength=0.2)   # X - red
                cv2.arrowedLine(frame, o, y, (0, 255, 0), 2, tipLength=0.2)   # Y - green
                cv2.arrowedLine(frame, o, z, (255, 0, 0), 2, tipLength=0.2)   # Z - blue

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