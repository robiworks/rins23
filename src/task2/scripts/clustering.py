#!/usr/bin/python3

import rospy
import numpy as np
import time

from sklearn.cluster import DBSCAN
from task2.msg import RingPoseMsg, ColorMsg
from collections import deque
from typing import List
from dataclasses import dataclass

RINGS_ON_POLYGON = {
    "Green": [0, 255, 0],
    "Red": [255, 0, 0],
    "Blue": [0, 0, 255],
    "Black": [0, 0, 0],
    "Brown": [175, 50, 0],
}

CYLINDERS_ON_POLYGON = {
    "Green": [0, 255, 0],
    "Red": [255, 0, 0],
    "Blue": [0, 0, 255],
    "Yellow": [255, 255, 0],
}


@dataclass
class RingColor:
    color: str
    ring: RingPoseMsg


class RingHolder:
    def __init__(self):
        self.rings: List[RingColor] = []

    def update_ring(self, ring: RingPoseMsg, color: str) -> bool:
        for rring in self.rings:
            if rring.color == color:
                rring.ring = ring
                return False
        self.rings.append(RingColor(color, ring))
        return True


class DBSCANClustering:
    def __init__(
        self,
        color,
        dbscan_eps=0.5,
        cluster_eps=0.5,
        min_samples=2,
        frame_duration=5,
        input_vector_size=5,
        num_features=2,
        do_reverse_lookup=True,
    ):
        self.frame_data = deque(maxlen=int(frame_duration * 1e3))
        self.color = color
        self.frame_duration = frame_duration
        self.dbscan_eps = dbscan_eps
        self.cluster_eps = cluster_eps
        self.min_samples = min_samples
        self.num_clusters = 0
        self.input_vector_size = input_vector_size
        self.num_features = num_features
        self.global_centroids = []
        self.do_reverse_lookup = do_reverse_lookup

    def add_point(self, point) -> RingPoseMsg:
        if len(point) != self.input_vector_size:
            raise ValueError(f"Point must be of length {self.input_vector_size}")

        timestamp = time.time()
        self.frame_data.append((timestamp, point))
        # Cleanup old data
        self.trim_frame_data()

        clusters = self.perform_dbscan()

        if clusters is None:
            return None

        for cluster in clusters:
            if not self.is_cluster_in_global_centroids(cluster):
                self.global_centroids.append(cluster)
                if self.do_reverse_lookup:
                    rpm = self.perform_reverse_lookup(cluster)
                else:
                    rpm = RingPoseMsg()
                    rpm.pose.position.x = cluster[0]
                    rpm.pose.position.y = cluster[1]
                    rpm.pose.position.z = cluster[2]
                return rpm
        return None

    def perform_dbscan(self) -> np.ndarray:
        data_array = np.array([point for _, point in self.frame_data])
        data_array = data_array[
            :, 0 : self.num_features
        ]  # Only use the first num_features

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=self.dbscan_eps, min_samples=self.min_samples)
        dbscan.fit(data_array)

        centroids = []
        for cluster_label in set(dbscan.labels_):
            if cluster_label == -1:  # This is the label for noise points
                continue
            cluster_points = data_array[dbscan.labels_ == cluster_label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)

        if len(centroids) == 0:
            return None

        return np.array(centroids)

    def is_cluster_in_global_centroids(self, cluster):
        cluster_threshold = self.cluster_eps
        for global_centroid in self.global_centroids:
            distance = np.linalg.norm(global_centroid - cluster)
            if distance < cluster_threshold:
                return True
        return False

    def trim_frame_data(self):
        current_time = time.time()
        while (
            self.frame_data
            and current_time - self.frame_data[0][0] > self.frame_duration
        ):
            self.frame_data.popleft()

    def perform_reverse_lookup(self, centroid) -> RingPoseMsg:
        # Find the closest point in frame data to the centroid
        similarities = []
        for v in self.frame_data:
            x = np.array(v[1][:2])
            sim = np.linalg.norm(x - centroid)
            similarities.append(sim)

        # find the smallest similraity
        min_sim = min(similarities)
        min_sim_idx = similarities.index(min_sim)
        ring_pose = self.frame_data[min_sim_idx][1]

        rpm = RingPoseMsg()
        rpm.pose.position.x = ring_pose[0]
        rpm.pose.position.z = ring_pose[1]
        rpm.color.r = ring_pose[2]
        rpm.color.g = ring_pose[3]
        rpm.color.b = ring_pose[4]

        return rpm


class Clustering:
    def __init__(self):
        rospy.init_node("ring_localizer", anonymous=True)

        # Setup publishers
        self.green_ring_pub = rospy.Publisher(
            "/custom_msgs/nav/green_ring_detected", RingPoseMsg, queue_size=10
        )
        self.ring_pub = rospy.Publisher(
            "/custom_msgs/nav/ring_detected", RingPoseMsg, queue_size=10
        )

        self.color_pub = rospy.Publisher(
            "/custom_msgs/sound/new_color", ColorMsg, queue_size=10
        )

        self.cylinder_pub = rospy.Publisher(
            "/custom_msgs/nav/cylinder_detected", RingPoseMsg, queue_size=10
        )

        self.cylinder_color_pub = rospy.Publisher(
            "/custom_msgs/sound/new_cylinder_color", ColorMsg, queue_size=10
        )

        # Setup subscribers
        self.ring_detected_sub = rospy.Subscriber(
            "/custom_msgs/ring_detection", RingPoseMsg, self.ring_detetected_callback
        )

        self.cylinder_detected_sub = rospy.Subscriber(
            "/custom_msgs/cylinder_detection",
            RingPoseMsg,
            self.cylinder_detected_callback,
        )

        # Setup differential localizer
        self.differnatial_localizers = []
        for ring in RINGS_ON_POLYGON:
            self.differnatial_localizers.append(DBSCANClustering(color=ring))
        # Setup ring holder
        self.ring_holder = RingHolder()

        # Setup differential localizer for cylinders
        self.cylinder_differnatial_localizer = DBSCANClustering(
            color="cylinder",
            input_vector_size=3,
            do_reverse_lookup=False,
            num_features=3,
        )
        # Setup cylinder holder
        self.cylinder_holder = RingHolder()

    def cylinder_detected_callback(self, msg: RingPoseMsg):
        x = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
        )

        if np.isnan(x).any():
            return

        rpm = self.cylinder_differnatial_localizer.add_point(x)
        if rpm is None:
            return

        rpm.color.r = msg.color.r
        rpm.color.g = msg.color.g
        rpm.color.b = msg.color.b

        rgb = np.array([msg.color.r, msg.color.g, msg.color.b])
        color = self.color_reverse_lookup(rgb, "cylinder")

        is_new = self.cylinder_holder.update_ring(rpm, color)
        if is_new:
            print(f"{color} cylinder detected at: ", rpm)
            self.cylinder_color_pub.publish(color)
            self.cylinder_pub.publish(rpm)

    def ring_detetected_callback(self, msg: RingPoseMsg):
        """
        Callback function for the ring detected topic.
        :param msg: RingPoseMsg
        :return: None
        """
        x = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.z,  # for some reason
                msg.color.r,
                msg.color.g,
                msg.color.b,
            ]
        )
        rgb = np.array([msg.color.r, msg.color.g, msg.color.b])
        color = self.color_reverse_lookup(rgb, "ring")
        idx = list(RINGS_ON_POLYGON.keys()).index(color)

        rpm = self.differnatial_localizers[idx].add_point(x)

        if rpm == None:
            return

        is_new = self.ring_holder.update_ring(rpm, color)

        if color == "Green":
            if is_new:
                print("Green ring detected at: ", rpm)
                self.green_ring_pub.publish(rpm)
                self.color_pub.publish(ColorMsg(color=color))
            print("Green ring detected at: ", rpm)
        else:
            if is_new:
                self.ring_pub.publish(rpm)
                self.color_pub.publish(ColorMsg(color=color))
            print(f"{color} ring detected at:", rpm)

    def color_reverse_lookup(self, rgb, type="ring"):
        def closest_color_ring(rgb):
            # If first value is biggest, then it is red
            if rgb[0] > rgb[1] and rgb[0] > rgb[2]:
                return "Red"
            # If second value is biggest, then it is green
            if rgb[1] > rgb[0] and rgb[1] > rgb[2]:
                return "Green"
            # If third value is biggest, then it is blue
            if rgb[2] > rgb[0] and rgb[2] > rgb[1]:
                return "Blue"
            # If all values are the same, with 5% tolerance, then it is black
            if (
                abs(rgb[0] - rgb[1]) < 0.05 * rgb[0]
                and abs(rgb[0] - rgb[2]) < 0.05 * rgb[0]
            ):
                return "Black"
            return "Unknown"  # Idk what here..

        def closest_color_cylinder(rgb, color_dict):
            min_distance = float("inf")
            closest_color_name = None
            for color_name, color_rgb in color_dict.items():
                distance = np.linalg.norm(np.array(rgb) - np.array(color_rgb))
                if distance < min_distance:
                    min_distance = distance
                    closest_color_name = color_name
            return closest_color_name

        if type == "cylinder":
            base_colors = CYLINDERS_ON_POLYGON
            return closest_color_cylinder(rgb, base_colors)
        elif type == "ring":
            base_colors = RINGS_ON_POLYGON
            return closest_color_ring(rgb)
        return None


def main():
    clustering = Clustering()
    print("Clustering node started")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
