#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2


class GelSightPointCloudNode:
    def __init__(self):
        # Params
        self.frame_id = rospy.get_param("~frame_id", "gelsight_frame")

        # mask/depth proxy params
        self.blur_ksize = int(rospy.get_param("~blur_ksize", 7))            # impar
        self.mask_threshold = float(rospy.get_param("~mask_threshold", 25)) # diff threshold
        self.min_depth = float(rospy.get_param("~min_depth", 0.0))          # para filtrar (m)
        self.max_depth = float(rospy.get_param("~max_depth", 0.01))         # (m) clip

        # baseline
        self.baseline_frames = int(rospy.get_param("~baseline_frames", 20))
        self.baseline_gray = None
        self._baseline_buffer = []

        # Pointcloud camera model (aproximado)
        # Puedes ajustarlo: para GelSight mini "imagen" no es una cámara pinhole real,
        # pero esto sirve para visualizar la "altura" como superficie 3D.
        self.fx = float(rospy.get_param("~fx", 350.0))
        self.fy = float(rospy.get_param("~fy", 350.0))
        self.cx = float(rospy.get_param("~cx", 320.0))
        self.cy = float(rospy.get_param("~cy", 240.0))

        # Escalas de la nube
        self.xy_scale = float(rospy.get_param("~xy_scale", 0.0002))  # m/pixel (para hacer la nube "bien" de tamaño)
        self.z_scale  = float(rospy.get_param("~z_scale", 1e-5))     # m por unidad de diff (proxy)
        self.pc_step  = int(rospy.get_param("~pc_step", 2))          # downsample para velocidad

        # Publishers
        self.pub_mask = rospy.Publisher("mask", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("depth", Image, queue_size=1)
        self.pub_points = rospy.Publisher("points", PointCloud2, queue_size=1)

        # Subscriber
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("image_raw", Image, self.cb_image, queue_size=1)

        # Services
        self.srv_calib = rospy.Service("calibrate_baseline", Trigger, self.handle_calibrate_baseline)

        rospy.loginfo("GelSightPointCloudNode ready. Waiting for image_raw...")

    def handle_calibrate_baseline(self, _req):
        # Limpiamos y pedimos N frames desde callbacks
        self._baseline_buffer = []
        rospy.loginfo("Calibrating baseline: collecting %d frames...", self.baseline_frames)

        # Espera activa (máx 2s + N frames)
        t0 = rospy.Time.now()
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            if len(self._baseline_buffer) >= self.baseline_frames:
                break
            if (rospy.Time.now() - t0).to_sec() > 4.0:
                break
            rate.sleep()

        if len(self._baseline_buffer) < max(3, self.baseline_frames // 2):
            return TriggerResponse(success=False, message=f"Not enough frames ({len(self._baseline_buffer)})")

        avg = np.mean(self._baseline_buffer, axis=0)
        self.baseline_gray = np.clip(avg, 0, 255).astype(np.uint8)
        rospy.loginfo("Baseline calibrated with %d frames", len(self._baseline_buffer))
        return TriggerResponse(success=True, message=f"Baseline OK ({len(self._baseline_buffer)} frames)")

    def _to_gray_blur(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if self.blur_ksize >= 3 and self.blur_ksize % 2 == 1:
            gray = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
        return gray

    def compute_mask_and_depth_proxy(self, rgb):
        gray = self._to_gray_blur(rgb)

        if self.baseline_gray is None:
            mask = np.zeros_like(gray, dtype=np.uint8)
            depth = np.zeros_like(gray, dtype=np.float32)
            return mask, depth, gray, None

        diff = cv2.absdiff(gray, self.baseline_gray).astype(np.float32)

        # mask por umbral
        _, mask = cv2.threshold(diff, self.mask_threshold, 255, cv2.THRESH_BINARY)
        mask = mask.astype(np.uint8)

        # limpieza morfológica
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # depth proxy: diff * z_scale (solo dentro de máscara)
        depth = (diff * self.z_scale).astype(np.float32)
        depth[mask == 0] = 0.0

        # clip de profundidad para que RViz no explote
        if self.max_depth > 0:
            depth = np.clip(depth, 0.0, self.max_depth)

        return mask, depth, gray, diff

    def depth_to_pointcloud(self, depth_m, stamp):
        H, W = depth_m.shape

        step = max(1, self.pc_step)
        us = np.arange(0, W, step, dtype=np.int32)
        vs = np.arange(0, H, step, dtype=np.int32)
        uu, vv = np.meshgrid(us, vs)

        z = depth_m[vv, uu]

        valid = np.isfinite(z) & (z > self.min_depth)
        if not np.any(valid):
            header = Header(stamp=stamp, frame_id=self.frame_id)
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
            ]
            return pc2.create_cloud(header, fields, [])

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        # Opción A (más “plano táctil”): XY en metros por pixel
        x = (uu - self.cx) * self.xy_scale
        y = (vv - self.cy) * self.xy_scale

        points = np.stack([x, y, z], axis=1)

        header = Header(stamp=stamp, frame_id=self.frame_id)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        return pc2.create_cloud(header, fields, points)

    def cb_image(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Failed to convert image_raw to rgb8: %s", str(e))
            return

        # Si estamos calibrando baseline, añadimos frames
        if self._baseline_buffer is not None and len(self._baseline_buffer) < self.baseline_frames:
            gray = self._to_gray_blur(rgb).astype(np.float32)
            self._baseline_buffer.append(gray)

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

        mask, depth, _, _ = self.compute_mask_and_depth_proxy(rgb)

        # Publish mask
        m = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        self.pub_mask.publish(m)

        # Publish depth (32FC1)
        d = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        d.header.stamp = stamp
        d.header.frame_id = self.frame_id
        self.pub_depth.publish(d)

        # Publish pointcloud
        pc = self.depth_to_pointcloud(depth, stamp)
        self.pub_points.publish(pc)


def main():
    rospy.init_node("gelsight_pointcloud_node", anonymous=False)
    node = GelSightPointCloudNode()
    rospy.spin()


if __name__ == "__main__":
    main()
