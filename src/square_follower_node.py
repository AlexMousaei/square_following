#!/usr/bin/env python

# Importing required libraries
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import subprocess

# Class to implement a square-following robot
class SquareFollower:

    # Defining states for state machine logic
    SEARCHING = "searching"
    FOLLOWING = "following"

    def __init__(self):
        # Initialise CvBridge
        self.bridge = CvBridge()
        
        # Subscribing to camera image feed and setting up a publisher for velocity commands
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Starting an image view window using rqt_image_view
        subprocess.Popen(["rqt_image_view", "/camera/rgb/image_raw"])

        # Initialise the robot's state to SEARCHING
        self.state = SquareFollower.SEARCHING
        
        # Define a buffer threshold for object center alignment
        self.buffer_threshold = 20

        # Initialise counter for missed frames
        self.missed_frames = 0
        
        # Define a threshold for maximum missed frames before searching
        self.missed_frames_threshold = 5

        # Log initialisation complete
        rospy.loginfo("Square follower node initialised.")

    # Callback function for image data
    def image_callback(self, data):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Detect square and get its centroid
        square_found, centroid = self.detect_square(cv_image)

        # Logic for following or searching for square
        if square_found:
            self.missed_frames = 0  # Reset missed frames counter
            self.follow_square(cv_image, centroid)
        else:
            self.missed_frames += 1
            if self.missed_frames > self.missed_frames_threshold:
                self.search_for_square()

    # Function to detect square and return its centroid
    def detect_square(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through each contour to find a square
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

    # Function to define robot's behavior when searching for square
    def search_for_square(self):
        if self.state != SquareFollower.SEARCHING:
            self.state = SquareFollower.SEARCHING
            rospy.loginfo("Searching for square...")
        
        self.drive_robot('right')

    # Function to define robot's behavior when following the square
    def follow_square(self, cv_image, centroid):
        if self.state != SquareFollower.FOLLOWING:
            self.state = SquareFollower.FOLLOWING
            rospy.loginfo("Following the square...")

        cX, cY = centroid
        image_center_x = cv_image.shape[1] // 2

        # Aligning robot with the square
        if cX > image_center_x + self.buffer_threshold:
            self.drive_robot('right')
        elif cX < image_center_x - self.buffer_threshold:
            self.drive_robot('left')
        else:
            self.drive_robot('forward')

    # Function to send movement command to robot
    def drive_robot(self, direction):
        twist = Twist()
        
        # Assigning movement based on direction
        if direction == 'right':
            twist.angular.z = -0.3
        elif direction == 'left':
            twist.angular.z = 0.3
        else:  # forward
            twist.linear.x = 0.2
        
        # Publishing the command
        self.cmd_vel_pub.publish(twist)

# Main function
if __name__ == '__main__':
    rospy.init_node('square_follower')
    sf = SquareFollower()
    rospy.spin()
