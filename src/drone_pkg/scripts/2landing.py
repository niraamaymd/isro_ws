#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FlatRegionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(
            '/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1
        )
        rospy.loginfo("FlatRegionDetector node started with block-level vectorized processing...")

    def detect_flat_regions(self, depth_image, block_size=10, var_threshold=10000):
        # Apply median filter to reduce noise
        median_filtered = cv2.medianBlur(depth_image, 5)

        h, w = median_filtered.shape
        # Trim image so it's divisible by block_size
        h_trim = h - (h % block_size)
        w_trim = w - (w % block_size)
        trimmed = median_filtered[:h_trim, :w_trim]

        # Reshape into blocks
        blocks = trimmed.reshape(
            h_trim // block_size, block_size,
            w_trim // block_size, block_size
        ).swapaxes(1, 2)  # shape: (num_blocks_y, num_blocks_x, block_size, block_size)

        # Mask out invalid (zero) depths
        valid_mask = (blocks > 0)
        blocks_float = np.where(valid_mask, blocks, np.nan)

        # Compute variance for each block, ignoring NaNs
        block_var = np.nanvar(blocks_float, axis=(2, 3))

        # Find blocks with variance below threshold
        low_var = block_var < var_threshold

        # Create block-level mask and upsample to image size
        block_mask = low_var.astype(np.uint8)
        mask = np.kron(block_mask, np.ones((block_size, block_size), dtype=np.uint8))

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Pad mask to original image size if needed
        full_mask = np.zeros_like(median_filtered, dtype=np.uint8)
        full_mask[:h_trim, :w_trim] = mask

        return full_mask, median_filtered

    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (16UC1: depth in mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            rospy.loginfo("Depth at center: %f(mm)" ,  depth_image[msg.height/2, msg.width/2])
            mask, median_filtered = self.detect_flat_regions(depth_image)

            # For display: convert to 8-bit and color
            depth_display = cv2.normalize(
                median_filtered, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            color_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)

            # Mark low-variance (flat) regions in green
            color_display[mask == 1] = [0, 255, 0]
            """
            original_display = cv2.normalize(
                    depth_image, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            cv2.imshow("Original Depth", original_display)
            """
            cv2.imshow("Flat Regions (Green)", color_display)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr("cv_bridge exception: {}".format(e))

if __name__ == '__main__':
    rospy.init_node('flat_region_detector')
    FlatRegionDetector()
    rospy.spin()

