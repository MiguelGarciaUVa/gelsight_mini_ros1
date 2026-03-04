#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge

from utilities.gelsightmini import GelSightMini

class GelSightWrapper:
    def __init__(self):
        # Params
        self.target_width = int(rospy.get_param("~target_width", 640))
        self.target_height = int(rospy.get_param("~target_height", 480))
        self.border_fraction = float(rospy.get_param("~border_fraction", 0.15))
        self.device_index = int(rospy.get_param("~device_index", 1))
        self.fps = float(rospy.get_param("~fps", 15.0))
        self.frame_id = rospy.get_param("~frame_id", "gelsight_frame")
        self.encoding = rospy.get_param("~encoding", "rgb8")  # "rgb8" o "bgr8"

        # Contact params
        self.mask_threshold = float(rospy.get_param("~mask_threshold", 25.0))  # intensidad de diferencia
        self.min_area = float(rospy.get_param("~min_area", 400.0))             # px
        self.blur_ksize = int(rospy.get_param("~blur_ksize", 7))               # impar
        self.baseline_frames = int(rospy.get_param("~baseline_frames", 20))

        # Publishers
        self.pub_img = rospy.Publisher("image_raw", Image, queue_size=1)
        self.pub_mask = rospy.Publisher("contact_mask", Image, queue_size=1)
        self.pub_contact = rospy.Publisher("contact", Bool, queue_size=1)
        self.pub_area = rospy.Publisher("contact_area", Float32, queue_size=1)
        self.pub_center = rospy.Publisher("contact_center", PointStamped, queue_size=1)

        # Service
        self.srv_baseline = rospy.Service("calibrate_baseline", Trigger, self.handle_calibrate_baseline)

        self.bridge = CvBridge()
        self.baseline_gray = None

        # Camera
        self.cam = GelSightMini(
            target_width=self.target_width,
            target_height=self.target_height,
            border_fraction=self.border_fraction
        )
        selected = self.cam.select_device(self.device_index)
        rospy.loginfo("GelSightMini selected device: %s", str(selected))
        self.cam.start()

        rospy.loginfo("Wrapper started. Publishing image_raw + contact_* @ %.1f Hz", self.fps)

    def get_frame_rgb(self):
        img = self.cam.update(1.0)
        if img is None or getattr(img, "shape", (0,))[0] == 0:
            return None
        # img viene en RGB según tu ejemplo
        return img

    def compute_contact(self, rgb):
        # a gris
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        if self.blur_ksize >= 3 and self.blur_ksize % 2 == 1:
            gray = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)

        if self.baseline_gray is None:
            # sin baseline no podemos estimar contacto
            mask = np.zeros_like(gray, dtype=np.uint8)
            return mask, False, 0.0, None

        diff = cv2.absdiff(gray, self.baseline_gray)

        # umbral
        _, mask = cv2.threshold(diff, self.mask_threshold, 255, cv2.THRESH_BINARY)

        # limpia ruido
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        area = float(cv2.countNonZero(mask))
        contact = area >= self.min_area

        center = None
        if contact:
            M = cv2.moments(mask, binaryImage=True)
            if M["m00"] > 1e-6:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                center = (cx, cy)

        return mask, contact, area, center

    def handle_calibrate_baseline(self, _req):
        frames = []
        for _ in range(self.baseline_frames):
            rgb = self.get_frame_rgb()
            if rgb is None:
                continue
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            if self.blur_ksize >= 3 and self.blur_ksize % 2 == 1:
                gray = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
            frames.append(gray.astype(np.float32))
            rospy.sleep(0.01)

        if len(frames) < max(3, self.baseline_frames // 2):
            return TriggerResponse(success=False, message="Not enough frames to calibrate baseline")

        avg = np.mean(frames, axis=0)
        self.baseline_gray = np.clip(avg, 0, 255).astype(np.uint8)
        return TriggerResponse(success=True, message=f"Baseline calibrated with {len(frames)} frames")

    def spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            rgb = self.get_frame_rgb()
            if rgb is None:
                rospy.logwarn_throttle(2.0, "Empty frame from GelSightMini")
                rate.sleep()
                continue

            stamp = rospy.Time.now()

            # Publica imagen
            if self.encoding == "bgr8":
                img_pub = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                img_pub = rgb

            msg_img = self.bridge.cv2_to_imgmsg(img_pub, encoding=self.encoding)
            msg_img.header.stamp = stamp
            msg_img.header.frame_id = self.frame_id
            self.pub_img.publish(msg_img)

            # Contacto
            mask, contact, area, center = self.compute_contact(rgb)

            msg_mask = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            msg_mask.header.stamp = stamp
            msg_mask.header.frame_id = self.frame_id
            self.pub_mask.publish(msg_mask)

            self.pub_contact.publish(Bool(data=bool(contact)))
            self.pub_area.publish(Float32(data=float(area)))

            if center is not None:
                p = PointStamped()
                p.header.stamp = stamp
                p.header.frame_id = self.frame_id
                p.point.x = float(center[0])  # px
                p.point.y = float(center[1])  # px
                p.point.z = 0.0
                self.pub_center.publish(p)

            rate.sleep()

def main():
    rospy.init_node("gelsight_mini_node", anonymous=False)
    node = GelSightWrapper()
    node.spin()

if __name__ == "__main__":
    main()



