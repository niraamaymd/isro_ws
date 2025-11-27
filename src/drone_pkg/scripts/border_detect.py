#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from my_msgs.msg import GridDetection
from cv_bridge import CvBridge
import cv2
import numpy as np

class YellowGridDetector:
    def __init__(self):
        rospy.init_node('yellow_grid_detector', anonymous=True)
        
        # Parameters
        self.area_threshold_ratio = rospy.get_param('~area_threshold_ratio', 0.01)
        self.hsv_lower = rospy.get_param('~hsv_lower', [12, 80, 200])
        self.hsv_upper = rospy.get_param('~hsv_upper', [56, 255, 255])
        
        # ROS infrastructure
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(
            'camera/color/image_raw', 
            Image, 
            self.image_callback,
            queue_size=10
        )
        self.pub = rospy.Publisher('yellow_grid', GridDetection, queue_size=10)
        self.pubfeed = rospy.Publisher('yellow_border', Image, queue_size=10)
        # Initialize grid labels
        self.grid_size = 3
        self.grid_labels = [["{}{}".format(i+1, j+1) for j in range(self.grid_size)] for i in range(self.grid_size)]

        # OpenCV window
        cv2.namedWindow("Yellow Detection Feed", cv2.WINDOW_NORMAL)

    def get_yellow_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.hsv_lower, dtype=np.uint8)
        upper = np.array(self.hsv_upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def analyze_grid(self, mask):
        """
        Segments the mask into 3x3 based on 25% and 75% splits.
        Returns a 3x3 list of 0s and 1s.
        """
        height, width = mask.shape
        h1 = int(0.25 * height)
        h2 = int(0.75 * height)
        w1 = int(0.25 * width)
        w2 = int(0.75 * width)

        # Regions: [(y1, y2, x1, x2), ...] in row-major order
        regions = [
            (0, h1, 0, w1),     (0, h1, w1, w2),     (0, h1, w2, width),
            (h1, h2, 0, w1),    (h1, h2, w1, w2),    (h1, h2, w2, width),
            (h2, height, 0, w1), (h2, height, w1, w2), (h2, height, w2, width)
        ]

        # Calculate threshold per region
        region_area = (h1) * (w1)  # approximate based on top-left cell
        threshold = int(region_area * self.area_threshold_ratio)

        detection_matrix = []
        for i in range(3):
            row = []
            for j in range(3):
                idx = i * 3 + j
                y1, y2, x1, x2 = regions[idx]
                segment = mask[y1:y2, x1:x2]
                active_pixels = np.sum(segment > 0)
                row.append(1 if active_pixels > threshold else 0)
            detection_matrix.append(row)

        return detection_matrix  # 3x3 matrix

#        height, width = mask.shape
#        step_y = height // self.grid_size
#        step_x = width // self.grid_size
#        threshold = int(step_x * step_y * self.area_threshold_ratio)

#        detection_matrix = []
#        for i in range(self.grid_size):
#            row = []
#            for j in range(self.grid_size):
#                segment = mask[i*step_y:(i+1)*step_y, j*step_x:(j+1)*step_x]
#                row.append(1 if np.sum(segment > 0) > threshold else 0)
#            detection_matrix.append(row)
#        return detection_matrix

    def draw_grid_overlay(self, frame, detection_matrix):
        """
        Draws the 3x3 grid based on the same 0.25 / 0.75 splits,
        highlights detected cells in yellow.
        """
        h, w = frame.shape[:2]
        h1 = int(0.25 * h)
        h2 = int(0.75 * h)
        w1 = int(0.25 * w)
        w2 = int(0.75 * w)

        # Draw grid lines
        cv2.line(frame, (w1, 0), (w1, h), (150, 150, 150), 2)
        cv2.line(frame, (w2, 0), (w2, h), (150, 150, 150), 2)
        cv2.line(frame, (0, h1), (w, h1), (150, 150, 150), 2)
        cv2.line(frame, (0, h2), (w, h2), (150, 150, 150), 2)

        # Rectangle coordinates for each cell
        coords = [
            (0, 0, w1, h1),     (w1, 0, w2, h1),     (w2, 0, w, h1),
            (0, h1, w1, h2),    (w1, h1, w2, h2),    (w2, h1, w, h2),
            (0, h2, w1, h),     (w1, h2, w2, h),     (w2, h2, w, h)
        ]

        for i in range(3):
            for j in range(3):
                if detection_matrix[i][j] == 1:
                    idx = i * 3 + j
                    x1, y1, x2, y2 = coords[idx]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
        self.image_callback(frame)
        return frame

#        h, w = frame.shape[:2]
#        for i in range(1, self.grid_size):
#            cv2.line(frame, (i*w//self.grid_size, 0), (i*w//self.grid_size, h), (255, 0, 0), 2)
#            cv2.line(frame, (0, i*h//self.grid_size), (w, i*h//self.grid_size), (255, 0, 0), 2)
        
#        cell_h = h // self.grid_size
#        cell_w = w // self.grid_size
#        for i in range(self.grid_size):
#            for j in range(self.grid_size):
#                if detection_matrix[i][j] == 1:
#                    cv2.rectangle(frame, 
#                                (j*cell_w, i*cell_h),
#                                ((j+1)*cell_w, (i+1)*cell_h),
#                                (0, 255, 255), 3)
#        return frame

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
 
        except Exception as e:
            rospy.logerr("Image conversion failed: {}".format(e))
            return

        # Process image
        mask = self.get_yellow_mask(cv_image)
        detection_matrix = self.analyze_grid(mask)

        # Create and publish message
        grid_msg = GridDetection()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.row1 = detection_matrix[0]
        grid_msg.row2 = detection_matrix[1]
        grid_msg.row3 = detection_matrix[2]
        self.pub.publish(grid_msg)
        
        # Visualization
        output_frame = self.draw_grid_overlay(cv_image.copy(), detection_matrix)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.addWeighted(output_frame, 0.7, mask_bgr, 0.3, 0)
        self.pubfeed.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
        cv2.imshow("Yellow Detection Feed", overlay)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = YellowGridDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
