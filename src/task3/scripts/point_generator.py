#!/usr/bin/python3
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan

import cv2
import numpy as np
import matplotlib.pyplot as plt

def generate_points(wall_margin: int, point_margin: int, n_clusters:int=8):
    map = cv2.imread('/home/nik/ros_repo/rins23/src/task3/config/map2_edited.pgm', cv2.IMREAD_GRAYSCALE)
    map_data = rospy.wait_for_message('map', OccupancyGrid)

    position = map_data.info.origin.position
    resolution = map_data.info.resolution
    # rotation = map_data.info.origin.orientation
    
    binary_map = map.copy()
    binary_map = np.where(map == 254, 1, 0).astype(np.uint8)


    # Erode
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    eroded_map = cv2.erode(binary_map, kernel)

    romaing_area = list(zip(*np.nonzero(map)))

    points = []

    while len(points) < n_clusters:
        add_point = True
        # Randomly select a point
        point = romaing_area[np.random.randint(0, len(romaing_area))]

        # Check if point is too close to wall using wall_margin
        min_dist_to_wall = np.min(eroded_map[max(0, point[0] - wall_margin):min(eroded_map.shape[0], point[0] + wall_margin+1),
                                       max(0, point[1] - wall_margin):min(eroded_map.shape[1], point[1] + wall_margin + 1)
                                        ])
        if min_dist_to_wall == 0:
            continue


        # Check if point is too close to other points
        if len(points) > 0:
            for p in points:
                dist = np.linalg.norm(np.array(point) - np.array(p))
                if dist < point_margin:
                    add_point = False
                    break


        if add_point:
            points.append(point)

    points = np.array(points)

    plt.imshow(map, cmap='gray')
    plt.scatter(points[:, 1], points[:, 0], c='r', s=50)
    plt.show()

    points[:, [1, 0]] = points[:, [0, 1]]
    points = points * resolution + np.array([position.x, position.y])

    # Rotate points 90 degrees in radians
    points = np.array([np.array([np.cos(np.pi/2) * p[0] - np.sin(np.pi/2) * p[1], np.sin(np.pi/2) * p[0] + np.cos(np.pi/2) * p[1]]) for p in points])

    # Translate points a bit down
    points = np.array([np.array([p[0] - 0.5, p[1]]) for p in points])

    return points


def main():
    rospy.init_node('point_generator', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    # Marker publisher
    marker_pub = rospy.Publisher('/markers/points', MarkerArray, queue_size=10)

    # Generate points
    points = generate_points(wall_margin=10, point_margin=20, n_clusters=9)
    print(points)

    # Create marker array
    marker_array = MarkerArray()

    # Create markers
    for i, point in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = i
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[1]
        marker.pose.position.y = point[0]
        marker.pose.position.z = 0.0

        marker_array.markers.append(marker)

    # Publish marker array
    marker_pub.publish(marker_array)

    # Spin
    rospy.spin()

if __name__ == '__main__':
    main()