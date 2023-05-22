#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from task3.srv import FineApproachSrv, FineApproachSrvResponse


class FineApproachNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/arm_camera/rgb/image_raw", Image, self.image_callback
        )
        self.cmd_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )
        self.service = rospy.Service(
            "/fine_approach", FineApproachSrv, self.fine_approach
        )
        self.current_image = None
        self.target_color = None

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def fine_approach(self, req):
        self.target_color = req.color

        while not rospy.is_shutdown():
            twist = self.approach_algorithm(self.current_image, self.target_color)
            self.cmd_pub.publish(twist)

        return FineApproachSrvResponse(success=True)

    def approach_algorithm(self, image, target_color, target_distance=0.1):
        color_ranges = {
            "blue": (np.array([100, 100, 100]), np.array([130, 255, 255])),
            "green": (np.array([50, 100, 100]), np.array([70, 255, 255])),
            "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
            "red": (np.array([0, 100, 100]), np.array([10, 255, 255])),
        }

        if target_color not in color_ranges:
            rospy.logerr(f"Unsupported color: {target_color}")
            return Twist()

        # get the color range for the target color
        lower_color, upper_color = color_ranges[target_color]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour, assuming this is the cylinder
            c = max(contours, key=cv2.contourArea)

            # Get the radius of a minimum enclosing circle of the contour
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # Known radius of the cylinder in cm
            known_radius = 10.0  # cm

            # Focal length of the camera (calibration required for accurate results)
            focal_length = 500  # pixels

            # Calculate distance from camera to object using the formula
            # distance_to_object = (known_radius * focal_length) / pixel_radius
            distance_to_object = (known_radius * focal_length) / radius  # cm

            # Calculate the velocity needed to approach the cylinder
            twist = Twist()
            if distance_to_object > target_distance:
                twist.linear.x = 0.1
            elif distance_to_object < target_distance:
                twist.linear.x = -0.1
            else:
                # Stop if the object is at the target_distance
                twist.linear.x = 0.0

            return twist


if __name__ == "__main__":
    rospy.init_node("fine_approach_node")
    fine_approach_node = FineApproachNode()
    rospy.loginfo("Fine approach node started")
    rospy.spin()
