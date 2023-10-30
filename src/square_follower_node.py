#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import subprocess

class SquareFollower:

    # States for the state machine
    SEARCHING = "searching"
    FOLLOWING = "following"
    STOPPED = "stopped"

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        subprocess.Popen(["rqt_image_view", "/camera/rgb/image_raw"])

        # Initialize the robot's state
        self.state = SquareFollower.SEARCHING
        self.buffer_threshold = 20

        # Counter for frames where the square is not detected
        self.missed_frames = 0
        # Threshold for the number of frames to miss before switching state
        self.missed_frames_threshold = 5

        rospy.loginfo("Square follower node initialised.")

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        square_found, centroid = self.detect_square(cv_image)

        if square_found:
            self.missed_frames = 0  # Reset counter
            self.follow_square(cv_image, centroid)
        else:
            self.missed_frames += 1
            if self.missed_frames > self.missed_frames_threshold:
                self.search_for_square()

    def detect_square(self, cv_image):
        """Detect a square in the given image and return the centroid."""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    return True, (cX, cY)

        return False, (0, 0)

    def search_for_square(self):
        """Robot's behavior when it's searching for a square."""
        if self.state != SquareFollower.SEARCHING:
            self.state = SquareFollower.SEARCHING
            rospy.loginfo("Searching for square...")

        self.drive_robot('right')

    def follow_square(self, cv_image, centroid):
        """Robot's behavior when it's following a detected square."""
        if self.state != SquareFollower.FOLLOWING:
            self.state = SquareFollower.FOLLOWING
            rospy.loginfo("Following the square...")

        cX, cY = centroid
        image_center_x = cv_image.shape[1] // 2

        if cX > image_center_x + self.buffer_threshold:
            self.drive_robot('right')
        elif cX < image_center_x - self.buffer_threshold:
            self.drive_robot('left')
        else:
            self.drive_robot('forward')

    def drive_robot(self, direction):
        """Send command to drive the robot in a given direction."""
        twist = Twist()
        if direction == 'right':
            twist.angular.z = -0.3
        elif direction == 'left':
            twist.angular.z = 0.3
        else:  # forward
            twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('square_follower')
    sf = SquareFollower()
    rospy.spin()