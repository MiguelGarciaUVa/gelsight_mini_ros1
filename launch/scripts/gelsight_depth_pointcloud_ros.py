#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

# gsrobotics-style imports (ajusta rutas si cambian)
from config import GSConfig
from utilities.reconstruction import Reconstruction3D
from utilities.gelsightmini import GelSightMini
from utilities.logger import log_message


class GelSightDepthPointCloudROS:
    """
    ROS wrapper that reproduces gsrobotics demo_view3D pipeline:
    - captures GelSightMini frames
    - runs Reconstruction3D.get_depthmap() -> depth_map + contact_mask (+ grads)
    - publishes depth, mask, and PointCloud2
    """

    def __init__(self):
        self.bridge = CvBridge()

        # Params
        self.gs_config_path = rospy.get_param("~gs_config", "default_config.json")

        # Camera selection (overrides config if set)
        self.device_index_override = rospy.get_param("~device_index", None)  # int or None

        # ROS frames / topics
        self.frame_id = rospy.get_param("~frame_id", "gelsight_frame")
        self.publish_image_raw = bool(rospy.get_param("~publish_image_raw", True))
        self.publish_gradients = bool(rospy.get_param("~publish_gradients", False))

        # PointCloud
        self.publish_pointcloud = bool(rospy.get_param("~publish_pointcloud", True))
        self.pc_step = int(rospy.get_param("~pc_step", 2))  # downsample pixels

        # Z/XY scaling (if you want to control cloud size)
        # If you know your mm/px from config/calib, set xy_scale accordingly.
        # Otherwise keep something small for visualization.
        self.xy_scale = float(rospy.get_param("~xy_scale", 1.0))  # units per pixel in "depth units"
        self.z_scale = float(rospy.get_param("~z_scale", 1.0))    # scale applied to depth_map before publishing

        # Publishers (all relative -> respect namespace)
        self.pub_img = rospy.Publisher("image_raw", Image, queue_size=1)
        self.pub_mask = rospy.Publisher("contact_mask", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("depth", Image, queue_size=1)
        self.pub_points = rospy.Publisher("points", PointCloud2, queue_size=1)

        self.pub_gradx = rospy.Publisher("grad_x", Image, queue_size=1)
        self.pub_grady = rospy.Publisher("grad_y", Image, queue_size=1)

        # Services
        self.srv_reset = rospy.Service("reset_reconstruction", Trigger, self.handle_reset_reconstruction)

        # Load config
        log_message(f"[ROS] Loading config: {self.gs_config_path}")
        gs_config = GSConfig(self.gs_config_path)
        self.config = gs_config.config

        # Init reconstruction
        self.reconstruction = Reconstruction3D(
            image_width=self.config.camera_width,
            image_height=self.config.camera_height,
            use_gpu=self.config.use_gpu,
        )
        if self.reconstruction.load_nn(self.config.nn_model_path) is None:
            raise RuntimeError(f"Failed to load NN model: {self.config.nn_model_path}")

        # Init camera
        self.cam = GelSightMini(
            target_width=self.config.camera_width,
            target_height=self.config.camera_height,
        )
        devices = self.cam.get_device_list()
        log_message(f"[ROS] Available camera devices: {devices}")

        if self.device_index_override is not None:
            cam_index = int(self.device_index_override)
        else:
            cam_index = int(self.config.default_camera_index)

        self.cam.select_device(cam_index)
        self.cam.start()
        log_message(f"[ROS] GelSightMini started (device_index={cam_index})")

        # Publish rate
        self.rate_hz = float(rospy.get_param("~rate", 0.0))  # 0 => loop as fast as frames arrive
        self.rate = rospy.Rate(self.rate_hz) if self.rate_hz > 0 else None

    def handle_reset_reconstruction(self, _req):
        # For now we just respond OK; if Reconstruction3D had state, you could reset it here.
        # You can also reload the model if you want.
        return TriggerResponse(success=True, message="Reconstruction reset (no-op)")

    def depth_to_pointcloud(self, depth_map, stamp):
        """
        Convert depth_map (H,W) float -> PointCloud2.
        This is a simple surface cloud: x,y from pixel coordinates, z from depth_map.
        """
        H, W = depth_map.shape
        step = max(1, self.pc_step)

        us = np.arange(0, W, step, dtype=np.int32)
        vs = np.arange(0, H, step, dtype=np.int32)
        uu, vv = np.meshgrid(us, vs)

        z = depth_map[vv, uu].astype(np.float32) * self.z_scale
        valid = np.isfinite(z)

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z  = z[valid].astype(np.float32)

        # Surface-like coordinates (not pinhole camera). xy_scale lets you tune units.
        cx = (W - 1) / 2.0
        cy = (H - 1) / 2.0
        x = (uu - cx) * self.xy_scale
        y = (vv - cy) * self.xy_scale

        points = np.stack([x, y, z], axis=1)

        header = Header(stamp=stamp, frame_id=self.frame_id)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        return pc2.create_cloud(header, fields, points)

    def spin(self):
        while not rospy.is_shutdown():
            frame = self.cam.update(dt=0)
            if frame is None:
                if self.rate:
                    self.rate.sleep()
                continue

            # OJO: en tu demo convierten BGR->RGB antes de pasar a reconstruction
            # y usan markers_threshold.
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            depth_map, contact_mask, grad_x, grad_y = self.reconstruction.get_depthmap(
                image=frame_rgb,
                markers_threshold=(self.config.marker_mask_min, self.config.marker_mask_max),
            )

            if depth_map is None or np.isnan(depth_map).any():
                if self.rate:
                    self.rate.sleep()
                continue

            stamp = rospy.Time.now()

            # Publish image_raw (RGB)
            if self.publish_image_raw:
                msg_img = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
                msg_img.header.stamp = stamp
                msg_img.header.frame_id = self.frame_id
                self.pub_img.publish(msg_img)

            # Publish contact_mask (mono8)
            # In demo: contact_mask is 0/1 float; convert to 0/255 uint8
            cm = (contact_mask * 255).astype(np.uint8)
            msg_mask = self.bridge.cv2_to_imgmsg(cm, encoding="mono8")
            msg_mask.header.stamp = stamp
            msg_mask.header.frame_id = self.frame_id
            self.pub_mask.publish(msg_mask)

            # Publish depth (32FC1)
            depth32 = depth_map.astype(np.float32) * self.z_scale
            msg_depth = self.bridge.cv2_to_imgmsg(depth32, encoding="32FC1")
            msg_depth.header.stamp = stamp
            msg_depth.header.frame_id = self.frame_id
            self.pub_depth.publish(msg_depth)

            # Optional gradients
            if self.publish_gradients:
                gx = grad_x.astype(np.float32)
                gy = grad_y.astype(np.float32)

                msg_gx = self.bridge.cv2_to_imgmsg(gx, encoding="32FC1")
                msg_gx.header.stamp = stamp
                msg_gx.header.frame_id = self.frame_id
                self.pub_gradx.publish(msg_gx)

                msg_gy = self.bridge.cv2_to_imgmsg(gy, encoding="32FC1")
                msg_gy.header.stamp = stamp
                msg_gy.header.frame_id = self.frame_id
                self.pub_grady.publish(msg_gy)

            # Publish pointcloud
            if self.publish_pointcloud:
                pc_msg = self.depth_to_pointcloud(depth_map, stamp)
                self.pub_points.publish(pc_msg)

            if self.rate:
                self.rate.sleep()


def main():
    rospy.init_node("gelsight_depth_pointcloud_ros", anonymous=False)
    node = GelSightDepthPointCloudROS()
    node.spin()


if __name__ == "__main__":
    main()
