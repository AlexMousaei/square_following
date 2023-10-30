#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion
import actionlib
import math
import tf

class SquareFollower:

    # States for the state machine
    SEARCHING = "searching"
    FOLLOWING = "following"
    STOPPED = "stopped"
    RETURNING = "returning"

    def __init__(self):
        self.scan_data = None
        self.front_distance = 0
        self.distance_to_square = 0
        self.returned = True
        self.centroid_ = (0,0)
        self.cv_image_ = None

        self.bridge = CvBridge()


        # Counter for frames where the square is not detected
        self.missed_frames = 0
        # Threshold for the number of frames to miss before switching state
        self.missed_frames_threshold = 5

        # # Action client for move_base
        self.move_base_action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client_.wait_for_server()
        rospy.loginfo("move_base action available")

        # Subscribe to laser scan
        self.laser_sub_ = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.test_image_pub = rospy.Publisher("/image/test", Image, queue_size=1)

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)



        # Initialize the robot's state
        self.state = SquareFollower.SEARCHING
        self.buffer_threshold = 20


        rospy.loginfo("Square follower node initialised.")

    def laser_callback(self,scan_data):
        #Laser scan here
        self.scan_data = scan_data
        center_index = len(scan_data.ranges) // 2

        # Get the distance value for the front-facing direction
        self.front_distance = scan_data.ranges[center_index]
        rospy.loginfo("Front distance: " + str(self.front_distance))

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        square_found, centroid, square = self.detect_square(cv_image)

        if square_found:
            self.missed_frames = 0  # Reset counter
            cv2.drawContours(cv_image, [square], -1, (0, 255, 0), 3)
            # Convert the OpenCV image to a ROS image message
            try:
                msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
            self.centroid_ = centroid
            self.cv_image_ = cv_image
            # Publish the image
            self.test_image_pub.publish(msg)
            self.follow_square(cv_image, centroid)
        elif self.state == SquareFollower.FOLLOWING or self.state == SquareFollower.RETURNING:
            self.follow_square(self.cv_image_,self.centroid_)
        else:
            self.missed_frames += 1
            if self.missed_frames > self.missed_frames_threshold:
                self.search_for_square()

    def detect_square(self, cv_image):
        """Detect a square in the given image and return the centroid."""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) >= 4 and len(approx) <= 6:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    return True, (cX, cY), approx

        return False, (0, 0), None

    def search_for_square(self):
        """Robot's behavior when it's searching for a square."""
        if self.state != SquareFollower.SEARCHING:
            self.state = SquareFollower.SEARCHING
            rospy.loginfo("Searching for square...")

        self.drive_robot('right')

    def follow_square(self, cv_image, centroid):
        """Robot's behavior when it's following a detected square."""
        if self.state == SquareFollower.FOLLOWING and self.returned == True:
            self.returned = False
        elif self.state == SquareFollower.RETURNING and self.move_base_action_client_.get_state() == actionlib.GoalStatus.SUCCEEDED and self.returned == False:
            self.returned = True
            self.centroid_ = (0,0)
            self.cv_image_ = None
            self.search_for_square()


        if self.state != SquareFollower.FOLLOWING and self.state != SquareFollower.RETURNING:
            self.state = SquareFollower.FOLLOWING
            rospy.loginfo("Following the square...")

            cX, cY = centroid
            self.centroid_ = centroid
            self.cv_image_ = cv_image
            image_center_x = cv_image.shape[1] // 2
            angle = math.atan2(cX - image_center_x, self.front_distance)
            self.rotate_robot(angle)
            self.distance_to_square = self.front_distance - 0.3
            if self.distance_to_square != float('inf'):
                self.move_to_goal(self.distance_to_square, 0)

        elif self.front_distance == float('inf') and self.state == SquareFollower.FOLLOWING and self.move_base_action_client_.get_state() != actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Aligning with square...")
            cX, cY = centroid
            image_center_x = cv_image.shape[1] // 2
            angle = math.atan2(cX - image_center_x, self.front_distance)
            self.rotate_robot(angle)

        elif self.state == SquareFollower.FOLLOWING and self.front_distance != float('inf') and self.move_base_action_client_.get_state() != actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Moving to square...")
            self.distance_to_square = self.front_distance - 0.3
            self.move_to_goal(self.distance_to_square, 0)

        elif self.move_base_action_client_.get_state() == actionlib.GoalStatus.SUCCEEDED and self.state != SquareFollower.RETURNING and self.returned == False:
            self.state = SquareFollower.RETURNING
            rospy.loginfo("Rotating...")
            self.move_to_goal(0,0,math.pi)

        elif self.state == SquareFollower.RETURNING and self.move_base_action_client_.get_state() == actionlib.GoalStatus.SUCCEEDED and self.returned == False:
            self.move_to_goal(self.distance_to_square, 0)

        # if cX > image_center_x + self.buffer_threshold:
        #     self.drive_robot('right')
        # elif cX < image_center_x - self.buffer_threshold:
        #     self.drive_robot('left')
        # else:
        #     self.drive_robot('forward')

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

    def move_to_goal(self, x, y, angle=0.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"  # Replace with the frame your goal is defined in (often "map")
        goal.target_pose.pose.position.x = x # Replace with your desired coordinates
        goal.target_pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, angle)  # Replace with your desired orientation

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        self.move_base_action_client_.send_goal(goal)
        rospy.loginfo(f"Setting goal to ({x}, {y}) in frame {goal.target_pose.header.frame_id}")

    def rotate_robot(self, angle):
        """Rotate the robot to the specified angle (in radians)."""
        twist = Twist()
        twist.angular.z = 0.3 if angle > 0 else -0.3  # Adjust angular velocity as needed
        self.cmd_vel_pub.publish(twist)

        # Keep rotating until the desired angle is achieved
        while abs(angle) > 0.1 and self.front_distance == float('inf'):  # You can adjust the tolerance as needed
            rospy.sleep(0.1)  # Adjust the sleep duration as needed
            twist.angular.z = 0.2 if angle > 0 else -0.2  # Adjust angular velocity as needed
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Rotating...")

        # Stop rotating
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('square_follower')
    sf = SquareFollower()
    rospy.spin()