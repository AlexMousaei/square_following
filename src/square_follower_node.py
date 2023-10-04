#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class SquareFollower:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.distance_to_square = float('inf')

        self.square_detected = False  # Initialize square detection flag

        # Define a buffer threshold
        self.buffer_threshold = 20  # Adjust this value as needed

        rospy.loginfo("Square follower node initialised.")

    def scan_callback(self, scan):
        self.distance_to_square = scan.ranges[0]
        #rospy.loginfo("Distance to obstacle: %s", self.distance_to_square)

    def image_callback(self, data):
        if self.square_detected == False:
            # Only process the image if a square has not been detected yet
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Convert the image to grayscale for processing
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # Use Canny edge detection
            edges = cv2.Canny(gray, 50, 150)
            # Find contours
            _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Approximate the contour to a polygon and check if it's a square
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if len(approx) == 4:
                    # Set the square detection flag to True
                    self.square_detected = True
                    rospy.loginfo("Square Found")
                    # Switch to the "following" mode
                    self.state = "following"
                    self.stop_robot()
                    break
                else:
                     self.drive_robot('right')
                     rospy.loginfo("No Square Found")
        elif self.state == "following":
            # Logic for following the square
            # Compute the centroid of the square
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0

                    # Assuming the center of the image is the center of the robot's view
                    image_center_x = cv_image.shape[1] // 2
                    

                    # If centroid is to the right with a buffer, turn right; otherwise, turn left
                    if cX > image_center_x + self.buffer_threshold:
                        self.drive_robot('right')
                    elif cX < image_center_x - self.buffer_threshold:
                        self.drive_robot('left')
                    else:
                        # If the centroid is within the buffer zone, go forward
                        self.drive_robot('forward')
        else:
            rospy.loginfo("CODE STUCK")            
        
    def drive_robot(self, direction):
        twist = Twist()
        if direction == 'right':
            twist.angular.z = -0.3
        elif direction == 'left':
            twist.angular.z = 0.3
        else:  # forward
            twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        rospy.loginfo("Stopping robot.")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('square_follower')
    sf = SquareFollower()
    rospy.spin()
