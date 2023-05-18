import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

def main():
    # Read map
    map = cv2.imread('/home/nik/ros_repo/rins23/src/task3/config/map2_edited.pgm', cv2.IMREAD_GRAYSCALE)
    # cv2.imshow('map', map)
    # cv2.waitKey(0)

    scaled_map = cv2.resize(map, (0, 0), fx=1/4, fy=1/4)

    # Show scaled map
    # cv2.imshow('scaled_map', scaled_map)
    # cv2.waitKey(0)

    # Binary map
    binary_map = map.copy()
    binary_map = np.where(map == 254, 1, 0).astype(np.uint8)

    # Show binary map
    # cv2.imshow('binary_map', binary_map)
    # cv2.waitKey(0)

    plt.imshow(binary_map, cmap='gray')
    plt.show()

    # Erode
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    eroded_map = cv2.erode(binary_map, kernel)

    # Show eroded map
    plt.imshow(eroded_map, cmap='gray')
    plt.show()

    # Calculate centroids
    romaing_area = list(zip(*np.nonzero(eroded_map)))
    kmeans = KMeans(n_clusters=6).fit(romaing_area)

    goals = kmeans.cluster_centers_

    # Show goals
    plt.imshow(eroded_map, cmap='gray')
    plt.scatter(goals[:, 1], goals[:, 0], c='r', s=50)
    plt.show()

    return

if __name__ == '__main__':
    main()