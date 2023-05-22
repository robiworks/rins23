#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
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

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def fine_approach(self, req):
        action = req.action
        twist = Twist()

        if action == "approach":
            rospy.loginfo("Approaching")
            for i in range(5):
                twist.linear.x = 0.05
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                rospy.sleep(0.5)

        elif action == "retreat":
            twist.linear.x -= 0.4
            twist.angular.z = 0.0
            rospy.loginfo("Retreating")
        else:
            rospy.logerr("Invalid action")
            return FineApproachSrvResponse(success=False)

        return FineApproachSrvResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("fine_approach_node")
    fine_approach_node = FineApproachNode()
    rospy.loginfo("Fine approach node started")
    rospy.spin()
