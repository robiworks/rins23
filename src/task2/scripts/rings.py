import cv2
import numpy as np

def main():
    # Read in the image
    ring = cv2.imread('/home/nik/Pictures/ring1.png', cv2.IMREAD_COLOR)
    ring_depth = cv2.imread('/home/nik/Pictures/ring1_depth.png', cv2.IMREAD_COLOR)
    
    # ring = cv2.imread('/home/nik/Pictures/ring_correct.png', cv2.IMREAD_COLOR)
    # ring_depth = cv2.imread('/home/nik/Pictures/ring_correct_depth.png', cv2.IMREAD_COLOR)

    ring_gray = cv2.cvtColor(ring, cv2.COLOR_BGR2GRAY)
    ring_depth_gray = cv2.cvtColor(ring_depth, cv2.COLOR_BGR2GRAY)


      # Apply thresholding
    img_h, thresh = cv2.threshold(ring_gray, 127, 255, cv2.THRESH_BINARY)
  
    # print(contours)
    # print(hierarchy)
    
    # Draw contours
    # cv2.drawContours(ring, contours, -1, (0, 255, 0), 3)
    

    # Show the img
    # cv2.imshow('Image', ring)
    # cv2.waitKey(0)

    # TODO: Get help with angle


    circles = cv2.HoughCircles(ring_gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    circles = np.uint16(np.around(circles))
    
    mask = np.zeros(ring_gray.shape, dtype=np.uint8)
    cv2.circle(mask, (circles[0][0][0], circles[0][0][1]), circles[0][0][2], 255, 2)
    mask = cv2.inRange(mask, 254, 255)

    # cv2.imshow('Image', mask)
    # cv2.waitKey(0)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # cv2.circle(ring, (x, y), r, (0, 255, 0), 2)
            # cv2.rectangle(ring, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            # Draw the mask on the image
            image = cv2.bitwise_and(ring_gray, ring_gray, mask=mask)

            # Mean the values that are not black
            hhh = ring_depth[image != 0]
            ring_mean = np.mean(ring_depth[image != 0])
            # print(ring_mean)

            center_mean = np.mean(ring_depth[y, x])
            # print(center_mean)

            if ring_mean > center_mean:
                print('Ring is not on paper')
            else:
                print('Ring is on paper')

            # 
            # cv2.imshow('Image', image)
            # # Save image to file
            # cv2.waitKey(0)
                


    # cv2.imshow('Image', mask)
    # cv2.waitKey(0)

    # # Show the img
    # cv2.imshow('Image', ring_depth)
    # cv2.waitKey(0)

    

    cv2.imshow('Image', ring)
    # Save the image
    cv2.imwrite('/home/nik/Pictures/ring_correct.png', ring)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # On the depth image, find all of the pixels that are black and set them to blue
    ring_depth[ring_depth == 0] = 255
    ring_depth[ring_depth == 1] = 0

    # # Show the img
    # cv2.imshow('Image', ring_depth)
    # cv2.waitKey(0)

if __name__ == '__main__':
    main()