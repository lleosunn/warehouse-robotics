#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R


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


class DualCameraDetectionNode(Node):
    def __init__(self):
        super().__init__('dual_camera_detection')

        self.num_agents = 2
        self.bridge = CvBridge()

        # Publishers for each camera's calculated coordinates
        # Camera0 publishes to /camera0/pose_1, /camera0/pose_2, etc.
        # Camera1 publishes to /camera1/pose_1, /camera1/pose_2, etc.
        self.camera0_pose_pubs = [
            self.create_publisher(Pose, f'/camera0/pose_{i+1}', 10)
            for i in range(self.num_agents)
        ]
        self.camera1_pose_pubs = [
            self.create_publisher(Pose, f'/camera1/pose_{i+1}', 10)
            for i in range(self.num_agents)
        ]

        # Subscribers for camera image topics
        self.camera0_sub = self.create_subscription(
            Image,
            '/camera0/image_raw',
            self.camera0_callback,
            10
        )
        self.camera1_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.camera1_callback,
            10
        )

        # --- ArUco setup ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = self.create_detector_parameters()

        # Camera calibration (same for both cameras - adjust if different)
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

        self.marker_size = 0.105  # units are in meters

        # World frame transforms for each camera
        self.camera0_have_world = False
        self.camera0_T_world_cam = None
        self.camera1_have_world = False
        self.camera1_T_world_cam = None

        self.get_logger().info("📷 Dual Camera Detection Node started.")
        self.get_logger().info("   Subscribing to /camera0/image_raw and /camera1/image_raw")
        self.get_logger().info("   Publishing to /camera0/pose_* and /camera1/pose_*")

    def create_detector_parameters(self):
        if hasattr(aruco, "DetectorParameters"):
            return aruco.DetectorParameters()
        elif hasattr(aruco, "DetectorParameters_create"):
            return aruco.DetectorParameters_create()
        else:
            raise RuntimeError("Cannot find DetectorParameters in cv2.aruco")

    def camera0_callback(self, msg):
        """Process images from camera0."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Failed to convert camera0 image: {e}")
            return

        self.process_frame(
            frame,
            camera_id=0,
            pose_pubs=self.camera0_pose_pubs,
            have_world=self.camera0_have_world,
            T_world_cam=self.camera0_T_world_cam,
            set_world_callback=self.set_camera0_world
        )

    def camera1_callback(self, msg):
        """Process images from camera1."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Failed to convert camera1 image: {e}")
            return

        self.process_frame(
            frame,
            camera_id=1,
            pose_pubs=self.camera1_pose_pubs,
            have_world=self.camera1_have_world,
            T_world_cam=self.camera1_T_world_cam,
            set_world_callback=self.set_camera1_world
        )

    def set_camera0_world(self, T_world_cam):
        self.camera0_have_world = True
        self.camera0_T_world_cam = T_world_cam

    def set_camera1_world(self, T_world_cam):
        self.camera1_have_world = True
        self.camera1_T_world_cam = T_world_cam

    def process_frame(self, frame, camera_id, pose_pubs, have_world, T_world_cam, set_world_callback):
        """Process a frame for ArUco detection and publish poses."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None:
            return

        # Marker geometry for solvePnP
        marker_points = np.array([
            [-self.marker_size / 2,  self.marker_size / 2, 0],
            [ self.marker_size / 2,  self.marker_size / 2, 0],
            [ self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
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

            # Marker 0 = WORLD FRAME ORIGIN
            if marker_id == 0:
                new_T_world_cam = invert_T(T_cam_marker)
                set_world_callback(new_T_world_cam)
                T_world_cam = new_T_world_cam
                have_world = True
                self.get_logger().info(f"🌍 Camera{camera_id}: Marker 0 sets WORLD frame.")

            # ROBOT MARKERS
            if have_world and T_world_cam is not None and 1 <= marker_id <= self.num_agents:
                T_world_m = T_world_cam @ T_cam_marker
                pos = T_world_m[:3, 3]
                R_mat = T_world_m[:3, :3]
                rot = R.from_matrix(R_mat)
                q_xyzw = rot.as_quat()
                roll, pitch, yaw = rot.as_euler('xyz', degrees=True)

                pose_msg = Pose()
                pose_msg.position.x = float(pos[0])
                pose_msg.position.y = float(pos[1])
                pose_msg.position.z = float(pos[2])

                pose_msg.orientation.x = float(q_xyzw[0])
                pose_msg.orientation.y = float(q_xyzw[1])
                pose_msg.orientation.z = float(q_xyzw[2])
                pose_msg.orientation.w = float(q_xyzw[3])

                pose_pubs[marker_id - 1].publish(pose_msg)

                self.get_logger().info(
                    f"🤖 Camera{camera_id} Marker {marker_id} WORLD pose: "
                    f"x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}, "
                    f"yaw={yaw:.1f}°"
                )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
