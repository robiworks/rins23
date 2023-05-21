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
from std_msgs.msg import ColorRGBA, Bool
import numpy as np
from PIL import Image as PILImage

from dataclasses import dataclass
from actionlib_msgs.msg import GoalID
import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from torchvision.transforms import ToTensor
from task3.msg import FacePositionMsg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations as tf_transformations
import geometry_msgs.msg
from task3.srv import (
    PosterExplorationSrv,
    PosterExplorationSrvResponse,
    FaceDialogueSrv,
    FaceDialogueSrvResponse,
)

import easyocr

### PARAMETERS ###
MARKER_DURATION = 360
FACE_DIFF_THRESHOLD = 0.5
NUM_POINTS = 8
RADIUS = 3
FACE_HEIGHT = 120
FACE_WIDTH = 90

POSTER_WORDS = ["WAN", "TED", "WA", "AN", "NT", "TE", "ED", "10", "1", "0", "1o"]
RING_INDICATORS = {
    "blue": ["BL", "UE", "LU"],
    "green": ["GR", "EE", "EN", "RE"],
}

### RUN  ###
# roslaunch task1 combined.launch
# rosrun task1 map_goals
# rosrun task1 face_localizer_dnn


@dataclass
class FacePositionArrayDto:
    def __init__(self, fpm: FacePositionMsg, ros_time: rospy.Time):
        self.fpm: FacePositionMsg = fpm
        self.ros_time: rospy.Time = ros_time


@dataclass
class FacePositionsHolder:
    def __init__(self):
        self.face_positions: FacePositionArrayDto = []
        self.tol = 0.2
        self.time_tol = 5

    def add_face_position(self, fpm: FacePositionMsg, ros_time: rospy.Time) -> bool:
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

    def add_pose(self, pose):
        self.poses.append(pose)
        poses_m = [
            np.array([p.position.x, p.position.y, p.position.z]) for p in self.poses
        ]
        self.pose.position.x, self.pose.position.y, self.pose.position.z = np.mean(
            poses_m, axis=0
        )


class FaceDescriptors:
    def __init__(self):
        self.faces_with_descriptors = []
        self.cancel_id = 0
        self.marker_id = 0

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
                if (
                    np.abs(face.pose.position.x - fdf.pose.position.x) < 0.11
                    or np.abs(face.pose.position.y - fdf.pose.position.y) < 0.11
                ):
                    print(f"[-] Face already known, norm: {norm}: Too close point")
                    return False

        self.faces_with_descriptors.append(fdf)
        return True


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

        self.dims = (0, 0, 0)
        self.log_dir = "/tmp/face_localizer_log"
        self.timestamp = datetime.datetime.now().isoformat()

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.face_descriptors = FaceDescriptors()
        self.poster_descriptors = FaceDescriptors()
        self.arm_face_descriptors = FaceDescriptors()

        ###            ###
        ### PUBLISHERS ###
        ###            ###

        self.face_coords_pub = rospy.Publisher(
            "/msg/coords/", PointStamped, queue_size=10
        )

        self.custom_msgs_face_detected = rospy.Publisher(
            "/custom_msgs/face_detected",
            FacePositionMsg,
            queue_size=10,
        )

        self.custom_msgs_poster_detected = rospy.Publisher(
            "/custom_msgs/poster_detected",
            FacePositionMsg,
            queue_size=10,
        )

        self.face_marker_pub = rospy.Publisher("/faces", MarkerArray, queue_size=10)

        self.poster_marker_pub = rospy.Publisher("/posters", MarkerArray, queue_size=10)

        self.face_marker_array = MarkerArray()
        self.poster_marker_array = MarkerArray()
        self.face_marker_id = 0
        self.poster_marker_id = 0
        self.marker_duration = rospy.Duration(3600)

        ###             ###
        ### SUBSCRIBERS ###
        ###             ###

        # self.custom_msgs_face_approached = rospy.Subscriber(
        #    "/custom_msgs/face_approached",
        #    Bool,
        #    self.face_approached_callback,
        #    queue_size=10,
        # )

        ###          ###
        ### SERVICES ###
        ###          ###

        self.poster_exploration_service = rospy.Service(
            "/poster_exploration",
            PosterExplorationSrv,
            self.poster_exploration_callback,
        )

        self.face_dialogue_service = rospy.Service(
            "/face_dialogue", FaceDialogueSrv, self.face_dialogue_callback
        )

    def face_dialogue_callback(self, msg):
        return FaceDialogueSrvResponse(useful=False, color1="blue", color2="green")

    def poster_exploration_callback(self, req):
        prize, ring_color = self.analyze_poster()
        return PosterExplorationSrvResponse(ring_color=ring_color, prize=prize)

    def analyze_poster(self):
        prizes = []
        ring_votes = {"blue": 0, "green": 0}

        for i in range(3):
            try:
                rgb_image_message = rospy.wait_for_message(
                    "/camera/rgb/image_raw/", Image
                )
            except Exception as e:
                print(e)
                return 0

            try:
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
            except CvBridgeError as e:
                print(e)

            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            hsv_image[:, :, 2] = cv2.equalizeHist(hsv_image[:, :, 2])
            rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
            rgb_image = PILImage.fromarray(rgb_image)
            rgb_image = np.array(rgb_image)

            ocr_result = self.ocr_image(rgb_image)

            print(ocr_result)

            if ocr_result is None:
                print("[-] No text detected")

            for word in ocr_result:
                if "BTC" in word:
                    print("[+] BTC detected")
                    word = word.replace("BTC", "")
                    word = word.replace(" ", "")
                    word = word.replace("o", "0")
                    word = word.replace("O", "0")
                    word = word.strip()
                    # Try to convert it into integer
                    print(word)
                    try:
                        wint = int(word)
                    except:
                        print("[-] Failed to convert to int")
                        continue
                    prizes.append(wint)

                br = RING_INDICATORS["blue"]
                for w in br:
                    if w in word:
                        ring_votes["blue"] += 1

                gr = RING_INDICATORS["green"]

                for w in gr:
                    if w in word:
                        ring_votes["green"] += 1

        if len(prizes) == 0:
            print("[-] No prizes detected")
            most_common_prize = 0
        else:
            most_common_prize = max(set(prizes), key=prizes.count)

            print("[+] Most common prize: {}".format(most_common_prize))

        votes_blue = ring_votes["blue"]
        votes_green = ring_votes["green"]

        ring = ""

        if votes_blue > votes_green:
            print("[+] Blue ring detected")
            ring = "blue"
        elif votes_green > votes_blue:
            print("[+] Green ring detected")
            ring = "green"
        else:
            print("[-] No ring detected")

        if ring == "" or most_common_prize == 0:
            print("[-] Poster analysis failed")
            return 11

        # descriptors = self.detect_faces()
        return most_common_prize, ring
        # for (
        #    face_descriptor,
        #    fdf,
        #    rgb_image,
        #    face_distance,
        #    box,
        #    depth_time,
        # ) in descriptors:
        #    fdf.poster_content = msg
        #    if self.poster_descriptors.add_descriptor(fdf):
        #        print("[+] Poster recognition is done.")
        #        self.custom_msgs_poster_analyzed.publish(msg)
        #    else:
        #        print("[-] Poster already detected.")

    def get_pose(self, coords, dist, stamp, return_angle=False):
        k_f = 554

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1 + x2) / 2.0
        face_y = self.dims[0] / 2 - (y1 + y2) / 2.0

        angle_to_target = np.arctan2(face_x, k_f)

        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Get the orientation of camera_rgb_optical_frame with respect to map frame
            transform = self.tf_buf.lookup_transform(
                "map", "camera_rgb_optical_frame", stamp
            )
            orientation = transform.transform.rotation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]

            angle_to_target_map = angle_to_target + yaw
            angle_to_target_map += np.pi / 2

            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z

        except Exception as e:
            print(e)
            pose = None

        if return_angle:
            return pose, angle_to_target_map
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

        print("OCR _RRES", ocr_res)

        if ocr_res is None:
            return False

        if len(ocr_res) == 0:
            return False

        if len(ocr_res) >= 1:
            if len(ocr_res[0]) == 0:
                return False
            if len(ocr_res[0]) >= 1:
                return True

        return False

    def detect_faces(self):
        descriptors = []
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

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Equalize the V channel of HSV image to normalize brightness
        hsv_image[:, :, 2] = cv2.equalizeHist(hsv_image[:, :, 2])

        # Convert image to rgb
        rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)

        rgb_image = PILImage.fromarray(rgb_image)

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
            if (
                face_distance < 0.5
                or face_distance > 1.2
                or face_distance == 0.0
                or np.isnan(face_distance)
            ):
                print(f"Face with distance {face_distance} is too close or too far")
                continue

            print("[~] face distance:", face_distance)

            pose = self.get_pose((x1, x2, y1, y2), face_distance, depth_time)

            if pose is None:
                continue

            face_descriptor = (
                self.resnet(face.to(self.device).unsqueeze(0)).cpu().detach().numpy()
            )

            fdf = Face(box, face_distance, depth_time, pose)
            fdf.describe(face_descriptor)
            if face_descriptor is None:
                continue

            if fdf is None:
                continue

            if face_distance is None or np.isnan(face_distance):
                continue

            if box is None:
                continue

            if depth_time is None:
                continue

            descriptors.append(
                (face_descriptor, fdf, rgb_image, face_distance, box, depth_time)
            )

        return descriptors

    def find_faces(self):
        descriptors = self.detect_faces()
        if (
            descriptors is None
            or len(descriptors) == 0
            or descriptors == 0
            or isinstance(descriptors, int)
        ):
            return
        for (
            face_descriptor,
            fdf,
            rgb_image,
            face_distance,
            box,
            depth_time,
        ) in descriptors:
            if self.face_descriptors.add_descriptor(fdf):
                print("[+] New Face added to the database")
                is_poster = self.check_if_face_is_on_poster(rgb_image)
                if is_poster:
                    self.poster_descriptors.add_descriptor(fdf)
                    print("[~] Face is on the poster")
                    self.send_marker(fdf, "poster")
                else:
                    print("[~] Face is not on the poster")
                    self.send_marker(fdf, "face")
                print("[~] Face added to the database")
                self.report_close_face(face_distance, box, depth_time, is_poster)

                # Save the image of detected face
                cv2.imwrite(f"/tmp/rins_{time.time()}.jpg", np.array(rgb_image))
                print("[+] Face saved")

    def send_marker(self, fdf, m_type):
        if m_type == "poster":
            self.poster_marker_id += 1
            marker_id = self.poster_marker_id
            marker_color = ColorRGBA(0, 255, 0, 1.0)
        elif m_type == "face":
            self.face_marker_id += 1
            marker_id = self.face_marker_id
            marker_color = ColorRGBA(127, 0, 255, 1.0)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = m_type
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = fdf.pose
        marker.lifetime = self.marker_duration
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color = marker_color

        if m_type == "poster":
            self.poster_marker_array.markers.append(marker)
            self.poster_marker_pub.publish(self.poster_marker_array)
        elif m_type == "face":
            self.face_marker_array.markers.append(marker)
            self.face_marker_pub.publish(self.face_marker_array)

    def report_close_face(self, face_distance, box, depth_time, is_poster):
        x1, y1, x2, y2 = box
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # Get the pose of the face
        pose, angle = self.get_pose(
            (x1, x2, y1, y2),
            max(face_distance - 0.45, 0.20),
            depth_time,
            return_angle=True,
        )

        print("ANGLE", angle)
        if pose is None:
            return

        quaterninon = tf_transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.x = quaterninon[0]
        pose.orientation.y = quaterninon[1]
        pose.orientation.z = quaterninon[2]
        pose.orientation.w = quaterninon[3]

        fpm = FacePositionMsg()
        fpm.pose = pose
        fpm.angle = angle

        if is_poster:
            self.custom_msgs_poster_detected.publish(fpm)
        else:
            self.custom_msgs_face_detected.publish(fpm)

        print(fpm)


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
