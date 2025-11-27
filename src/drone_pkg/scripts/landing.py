#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class FlatRegionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1)
	rospy.loginfo("FlatRegionDetector node started with median filtering and green highlighting...")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (16UC1: depth in mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Apply median filter to reduce noise
            median_filtered = cv2.medianBlur(depth_image, 5)
            
            # Parameters
            window_size = 10
            var_threshold = 1000  # Adjust as needed for your sensor
            
            h, w = median_filtered.shape
            low_var_mask = np.zeros_like(median_filtered, dtype=np.uint8)
            
            # Slide window and mark low-variance (flat) regions
            for y in range(0, h - window_size + 1, window_size):
                for x in range(0, w - window_size + 1, window_size):
                    window = median_filtered[y:y+window_size, x:x+window_size]
                    # Only consider nonzero (valid) depths
                    window_valid = window[window > 0]
                    if window_valid.size == 0:
                        continue
                    if np.var(window_valid) < var_threshold:
                        low_var_mask[y:y+window_size, x:x+window_size] = 1
            
            # For display: convert to 8-bit and color
            depth_display = cv2.normalize(median_filtered, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            color_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)
            
            # Mark low-variance (flat) regions in green
            color_display[low_var_mask == 1] = [0, 255, 0]
            
            cv2.imshow("Flat Regions (Green)", color_display)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr("cv_bridge exception: {}".format(e))


if __name__ == '__main__':
    rospy.init_node('flat_region_detector')
    FlatRegionDetector()
    rospy.spin()

