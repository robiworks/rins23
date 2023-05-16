#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import time

from os.path import dirname, join
import os
import datetime

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from PIL import Image as PILImage

from dataclasses import dataclass
from actionlib_msgs.msg import GoalID
import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from torchvision.transforms import ToTensor
from task1.msg import FacePositionMessage

import easyocr

### PARAMETERS ###
MARKER_DURATION = 360
FACE_DIFF_THRESHOLD = 0.5
NUM_POINTS = 8
RADIUS = 3
FACE_HEIGHT = 120
FACE_WIDTH = 90

POSTER_WORDS = ["WAN", "TED", "WANT", "WA", "NT"]

### RUN  ###
# roslaunch task1 combined.launch
# rosrun task1 map_goals
# rosrun task1 face_localizer_dnn

@dataclass
class FacePositionArrayDto:
    def __init__(self, fpm: FacePositionMessage, ros_time: rospy.Time):
        self.fpm: FacePositionMessage = fpm
        self.ros_time: rospy.Time = ros_time

@dataclass
class FacePositionsHolder:
    def __init__(self):
        self.face_positions: FacePositionArrayDto = []
        self.tol = 0.2
        self.time_tol = 5

    def add_face_position(self, fpm: FacePositionMessage, ros_time: rospy.Time) -> bool:
        def check_fpm(fpm1, fpm2) -> bool:
            return (
                abs(fpm1.x - fpm2.x) < self.tol
                and abs(fpm1.y - fpm2.y) < self.tol
                and abs(fpm1.z - fpm2.z) < self.tol
            )

        def check_time(time1, time2) -> bool:
            time_diff = time1 - time2
            time_diff_sec = time_diff.to_sec()
            return abs(time_diff_sec) < self.time_tol

        for face_position in self.face_positions:
            if check_time(face_position.ros_time, ros_time):
                return False

        self.face_positions.append(FacePositionArrayDto(fpm, ros_time))
        return True

@dataclass
class Face:
    def __init__(self, box, face_distance, depth_time, pose):
        self.box = box
        self.face_distance = face_distance
        self.depth_time = depth_time
        self.pose = pose
        self.descriptor = None
        self.marker = None
        self.poses = []

    def describe(self, descriptor):
        self.descriptor = descriptor

    def add_marker(self, marker):
        self.marker = marker

    def add_pose(self, pose):
        self.poses.append(pose)
        poses_m = [
            np.array([p.position.x, p.position.y, p.position.z]) for p in self.poses
        ]
        self.pose.position.x, self.pose.position.y, self.pose.position.z = np.mean(
            poses_m, axis=0
        )
        self.marker.pose = self.pose


class FaceDescriptors:
    def __init__(self):
        self.faces_with_descriptors = []
        self.cancel_id = 0
        self.marker_id = 0
        self.markers_pub = rospy.Publisher("face_markers", MarkerArray, queue_size=1000)
        self.goal_cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
        self.marker_array = MarkerArray()

    def hellinger_distance(self, a, b):
        return np.sqrt(
            np.sum((np.sqrt(np.abs(a)) - np.sqrt(np.abs(b))) ** 2)
        ) / np.sqrt(2)

    def add_descriptor(self, fdf: Face) -> bool:
        # Check if the description is already in the list or something similar
        for face in self.faces_with_descriptors:
            norm = self.hellinger_distance(face.descriptor, fdf.descriptor)
            if norm < FACE_DIFF_THRESHOLD:
                face.add_pose(fdf.pose)
                print(f"[-] Face already known, norm: {norm}")
                return False
            else:
                if np.abs(face.pose.position.x - fdf.pose.position.x) < 0.11 or np.abs(
                    face.pose.position.y - fdf.pose.position.y
                ) < 0.11:
                    print(f"[-] Face already known, norm: {norm}: Too close point")
                    return False


        print("[+] New face detected!")
        self.faces_with_descriptors.append(fdf)
        marker = self.report_new_face_and_stop_the_robot()
        fdf.add_marker(marker)
        return True

    def report_new_face_and_stop_the_robot(self) -> Marker:
        new_face = self.faces_with_descriptors[-1]

        # Report the new Face
        self.marker_id += 1
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.pose = new_face.pose
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(MARKER_DURATION)
        marker.id = self.marker_id
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 1, 0, 1)
        self.marker_array.markers.append(marker)
        # Publish the marker
        self.markers_pub.publish(self.marker_array)

        return marker


class face_localizer:
    def __init__(self):
        rospy.init_node("face_localizer", anonymous=True)

        # Initialize the models
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.use_gpu = True if torch.cuda.is_available() else False
        self.mtcnn = MTCNN(
            keep_all=True, device=self.device, post_process=False, margin=20
        )
        self.resnet = InceptionResnetV1(pretrained="vggface2").eval().to(self.device)
        self.transform = ToTensor()

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        self.face_coords_pub = rospy.Publisher(
            "/msg/coords/", PointStamped, queue_size=10
        )

        self.custom_msgs_face_position_publisher = rospy.Publisher(
                "/custom_msgs/face_position_message",
                FacePositionMessage,
                queue_size=10,
        )

        self.dims = (0, 0, 0)
        self.log_dir = "/tmp/face_localizer_log"
        self.timestamp = datetime.datetime.now().isoformat()

        ## Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.face_descriptors = FaceDescriptors()

        self.face_positions_holder = FacePositionsHolder()

    def get_pose(self, coords, dist, stamp, return_angle=False):
        """Calculate the position of the detected face"""

        k_f = 554  # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1 + x2) / 2.0
        face_y = self.dims[0] / 2 - (y1 + y2) / 2.0

        angle_to_target = np.arctan2(face_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        if return_angle:
            return pose, angle_to_target
        else:
            return pose

    def ocr_image(self, image):
        """OCR the image and return the text"""
        reader = easyocr.Reader(["en"], gpu=self.use_gpu)
        result = reader.readtext(image)

        # Convert result into array of strings
        result = [r[1] for r in result]

        return result

    def check_if_face_is_on_poster(self, rgb_image):
        rgb_image = np.array(rgb_image)

        ocr_res = self.ocr_image(rgb_image)

        for word in ocr_res:
            for poster_word in POSTER_WORDS:
                if poster_word in word:
                    return True

        return False


    def find_faces(self):
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw/", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message(
                "/camera/depth/image_raw/", Image
            )
        except Exception as e:
            print(e)
            return 0

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        self.dims = rgb_image.shape
        #print(self.dims)

        #rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        #upper_limit = 40 
        #rgb_image = rgb_image[upper_limit:, :, :]
        #rgb_image = PILImage.fromarray(rgb_image)

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Equalize the V channel of HSV image to normalize brightness
        hsv_image[:, :, 2] = cv2.equalizeHist(hsv_image[:, :, 2])

        # Convert image to rgb
        rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)

        # Convert the image to PIL format
        rgb_image = PILImage.fromarray(rgb_image)
        #import matplotlib.pyplot as plt
        #plt.imshow(rgb_image)
        #plt.show()

        # Detect faces and extract descriptors
        with torch.no_grad():
            faces_cropped = self.mtcnn(rgb_image, save_path=None)
            # Get the bounding boxes
            batch_boxes, _ = self.mtcnn.detect(rgb_image, landmarks=False)

        if faces_cropped is None or batch_boxes is None:
            return

        for face, box in zip(faces_cropped, batch_boxes):
            x1, y1, x2, y2 = box
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            # Get the pose of the face
            depth_time = depth_image_message.header.stamp

            # Prevent computing the distance to a face that is not in the image
            if np.isnan(depth_image[y1:y2, x1:x2]).any():
                continue

            # Get the distance to the face
            try:
                face_distance = np.nanmedian(depth_image[y1:y2, x1:x2])
                # Ignore nan warnging
            except RuntimeWarning:
                continue

            # Face is either to close or too far away
            if face_distance < 0.5 or face_distance > 1.2 or face_distance == 0.0 or np.isnan(face_distance):
                print(f"Face with distance {face_distance} is too close or too far")
                continue

            print("[~] face distance:", face_distance)

            pose = self.get_pose((x1, x2, y1, y2), face_distance, depth_time)

            if pose is None:
                continue

            face_descriptor = (
                self.resnet(face.to(self.device).unsqueeze(0)).cpu().detach().numpy()
            )

            # Save the detected_face
            #import time
            #print("[~] Saving face")
            #print(f"/tmp/{time.time()}.jpg")
            #cv2.imwrite(f"/tmp/{time.time()}.jpg", np.array(rgb_image))

            fdf = Face(box, face_distance, depth_time, pose)
            fdf.describe(face_descriptor)

            if self.face_descriptors.add_descriptor(fdf):
                self.report_close_face(face_distance, box, depth_time)
                result = self.check_if_face_is_on_poster(rgb_image)
                if result:
                    print("[~] Face is on the poster")
                else:
                    print("[~] Face is not on the poster")
                print("[~] Face added to the database")
                cv2.imwrite(f"/tmp/rins_{time.time()}.jpg", np.array(rgb_image))
                print("[+] Face saved")

    def report_close_face(self, face_distance, box, depth_time):
        x1, y1, x2, y2 = box
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # Get the pose of the face
        pose, angle = self.get_pose((x1, x2, y1, y2), face_distance * 0.95, depth_time, return_angle=True)
        if pose is None:
            return
        fpm = FacePositionMessage()
        fpm.x = pose.position.x
        fpm.y = pose.position.y
        fpm.z = pose.position.z
        fpm.angle = angle

        print(fpm)

        self.custom_msgs_face_position_publisher.publish(fpm)


def main():
    face_finder = face_localizer()

    rate = rospy.Rate(5)
    print("Face localizer initialized")
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
