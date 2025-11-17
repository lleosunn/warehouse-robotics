#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R  # ✅ NEW

def rvec_tvec_to_T(rvec, tvec):
    """Convert OpenCV rvec/tvec to a 4x4 transform matrix."""
    R_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R_mat
    T[:3, 3] = tvec.flatten()
    return T

def invert_T(T):
    """Invert a 4x4 rigid transform."""
    R_mat = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=np.float32)
    Tinv[:3, :3] = R_mat.T
    Tinv[:3, 3] = -R_mat.T @ t
    return Tinv

class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_node')

        # --- Publisher ---
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)

        # --- Camera setup ---
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open camera.")
            raise RuntimeError("Camera not accessible.")

        # --- ArUco setup ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = self.create_detector_parameters()

        # Load your calibration
        self.camera_matrix = np.array([
            [628.2060513684523, 0.0, 334.5792569768152],
            [0.0, 627.5840905155266, 252.94919633119522],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [
                0.018873878948639438,
                0.1382933190183097,
                -0.0014933935941659632,
                0.006031010738591554,
                -0.7083862634771421
            ]
        ], dtype=np.float32)

        self.marker_size = 0.105
        self.have_world = False
        self.T_world_cam = None

        # Timer @ 30 Hz
        self.timer = self.create_timer(1/30.0, self.timer_callback)
        self.get_logger().info("📷 ArUco Pose Node with WORLD frame started.")

    def create_detector_parameters(self):
        if hasattr(aruco, "DetectorParameters"):
            return aruco.DetectorParameters()
        elif hasattr(aruco, "DetectorParameters_create"):
            return aruco.DetectorParameters_create()
        else:
            raise RuntimeError("Cannot find DetectorParameters in cv2.aruco")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ Failed to grab frame.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None:
            cv2.imshow("ArUco Pose Estimation", frame)
            cv2.waitKey(1)
            return

        aruco.drawDetectedMarkers(frame, corners, ids)

        # Marker geometry for solvePnP
        marker_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        for i, corner in enumerate(corners):
            marker_id = int(ids[i])
            image_points = corner.reshape(-1, 2)

            success, rvec, tvec = cv2.solvePnP(
                marker_points, image_points,
                self.camera_matrix, self.dist_coeffs
            )
            if not success:
                continue

            # Convert OpenCV vectors → full transform matrix
            T_cam_marker = rvec_tvec_to_T(rvec, tvec)

            # -----------------------------
            # Marker 1 = WORLD FRAME ORIGIN
            # -----------------------------
            if marker_id == 1:
                self.T_world_cam = invert_T(T_cam_marker)
                self.have_world = True
                self.get_logger().info("🌍 Marker 1 sets WORLD frame.")

            # -----------------------------
            # Marker 0 = ROBOT MARKER
            # -----------------------------
            if marker_id == 0 and self.have_world:
                T_world_m1 = self.T_world_cam @ T_cam_marker
                pos = T_world_m1[:3, 3]

                # Extract world-frame rotation
                R_mat = T_world_m1[:3, :3]
                rot = R.from_matrix(R_mat)

                q_xyzw = rot.as_quat()                  # x, y, z, w
                roll, pitch, yaw = rot.as_euler('xyz', degrees=True)

                # Publish pose
                pose_msg = Pose()
                pose_msg.position.x = float(pos[0])
                pose_msg.position.y = float(pos[1])
                pose_msg.position.z = float(pos[2])

                pose_msg.orientation.x = float(q_xyzw[0])
                pose_msg.orientation.y = float(q_xyzw[1])
                pose_msg.orientation.z = float(q_xyzw[2])
                pose_msg.orientation.w = float(q_xyzw[3])

                self.pose_pub.publish(pose_msg)

                self.get_logger().info(
                    f"🤖 Marker 0 WORLD pose: "
                    f"x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}, "
                    f"yaw={yaw:.1f}°"
                )

        cv2.imshow("ArUco Pose Estimation", frame)
        cv2.waitKey(1)

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