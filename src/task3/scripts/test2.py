import cv2
import numpy as np
import matplotlib.pyplot as plt

def generate_points(map, wall_margin: int, point_margin: int, n_clusters:int=6):
    romaing_area = list(zip(*np.nonzero(map)))

    points = []

    while len(points) < n_clusters:
        add_point = True
        # Randomly select a point
        point = romaing_area[np.random.randint(0, len(romaing_area))]

        # Check if point is too close to wall using wall_margin
        min_dist_to_wall = np.min(map[max(0, point[0] - wall_margin):min(map.shape[0], point[0] + wall_margin+1),
                                       max(0, point[1] - wall_margin):min(map.shape[1], point[1] + wall_margin + 1)
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
            # plt.imshow(map, cmap='gray')
            # plt.scatter(point[1], point[0], c='r', s=50)
            # plt.show()

    return np.array(points)

def main():
    map = cv2.imread('/home/nik/ros_repo/rins23/src/task3/config/map2_edited.pgm', cv2.IMREAD_GRAYSCALE)
    
    binary_map = map.copy()
    binary_map = np.where(map == 254, 1, 0).astype(np.uint8)


    # Erode
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    eroded_map = cv2.erode(binary_map, kernel)

    # plt.imshow(eroded_map, cmap='gray')
    # plt.show()


    # Function for adding points

    points = generate_points(eroded_map, wall_margin=10, point_margin=30, n_clusters=8)

    plt.imshow(eroded_map, cmap='gray')
    plt.scatter(points[:, 1], points[:, 0], c='r', s=50)
    plt.show()

    return

if __name__ == '__main__':
    main()