#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from task3.srv import ArmParkingSrv, ArmParkingSrvRequest, ArmParkingSrvResponse


class FineParkingNode:
    def __init__(self):
        self.bridge = CvBridge()

        self.cmd_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )
        self.service = rospy.Service("/fine_parking", ArmParkingSrv, self.park)

    def park(self, req: ArmParkingSrvRequest):
        r = rospy.Rate(4)

        if req.search:
            i = 0
            while i < 50:
                image = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
                image = self.bridge.imgmsg_to_cv2(image, "bgr8")
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                image = np.where(image > 20, 0, 1)

                # image[image <= 20] = 0
                # image[image > 20] = 1
                # image = image / 255
                height, width = image.shape

                image_l = image[:, : width // 2]
                image_r = image[:, width // 2 :]

                count_left = cv2.countNonZero(image_l)
                count_right = cv2.countNonZero(image_r)

                if count_left + count_right < 8000 and i > 19:
                    return ArmParkingSrvResponse(finished=True)

                ratio = count_left / count_right

                twist_msg = Twist()
                if ratio < 0.8:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = -0.1
                elif ratio > 1.2:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.1
                else:
                    twist_msg.linear.x = 0.1
                    twist_msg.angular.z = 0.0

                self.cmd_pub.publish(twist_msg)

                i += 1
                r.sleep()

        return ArmParkingSrvResponse(finished=True)


if __name__ == "__main__":
    rospy.init_node("fine_parking_node")
    fine_parking_node = FineParkingNode()
    rospy.loginfo("Fine parking node started")
    rospy.spin()
