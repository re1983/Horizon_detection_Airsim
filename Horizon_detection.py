import numpy as np
import airsim
import keyboard
import cv2
import threading
import math
from math import cos, sin, pi, radians, degrees
# import time
# import sys
# import os
FULL_ROTATION = 360
FULL_ROTATION_RADIANS = 2 * pi
threads =[]

def _find_points(m: float, b: float, frame_shape: tuple) -> list:
    """"
    Given the slope (m), y intercept (b) and the frame shape (frame_shape),
    find the two points of the line that intersect with the border of the frame.
    """
    # special condition if slope is 0
    if m == 0:
        b = int(np.round(b))
        p1 = (0, b)
        p2 = (frame_shape[1], b)
        return [p1, p2]

    points_to_return = []
    # left
    if 0 < b <= frame_shape[0]:
        px = 0
        py = int(np.round(b))
        points_to_return.append((px, py))
    # top
    if 0 < -b / m <= frame_shape[1]:
        px = int(np.round(-b / m))
        py = 0
        points_to_return.append((px, py))
    # right
    if 0 < m * frame_shape[1] + b <= frame_shape[0]:
        px = frame_shape[1]
        py = int(np.round(m * frame_shape[1] + b))
        points_to_return.append((px, py))
    # bottom
    if 0 < (frame_shape[0] - b) / m <= frame_shape[1]:
        px = int(np.round((frame_shape[0] - b) / m))
        py = frame_shape[0]
        points_to_return.append((px, py))

    return points_to_return


def draw_horizon(frame: np.ndarray, roll: float , pitch: float, 
                    fov: float, color: tuple, draw_groundline: bool):

    # if no horizon data is provided, terminate function early and return
    if roll is None:
        return

    # take roll in degrees and express it in terms of radians
    # roll = radians(roll)
    
    # determine if the sky is up or down based on the roll
    sky_is_up = (roll >= FULL_ROTATION_RADIANS * .75  or (roll > 0 and roll <= FULL_ROTATION_RADIANS * .25))
    
    # find the distance 
    distance = degrees(pitch) / fov * frame.shape[0]

    # define the line perpendicular to horizon
    angle_perp = roll + pi / 2
    x_perp = distance * cos(angle_perp) + frame.shape[1]/2
    y_perp = distance * sin(angle_perp) + frame.shape[0]/2

    # define the horizon line
    run = cos(roll)
    rise = sin(roll)
    if run != 0:
        m = rise / run
        b = y_perp - m * x_perp
        points = _find_points(m, b, frame.shape)
        if not points:
            return
        else:
            p1, p2 = points
       
    else:
        p1 = (int(np.round(x_perp)), 0)
        p2 = (int(np.round(x_perp)), frame.shape[0])

    cv2.line(frame, p1, p2, color, 2)
    # print(frame.shape[1])
    cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), round(10), (0,255,0), 2)
    # if draw_groundline and m != 0:
    #     m_perp = -1/m
    #     b_perp = y_perp - m_perp * x_perp
    #     points = _find_points(-1/m, b_perp, frame.shape)
    #     above_line = m * points[0][0] + b < points[0][1]
    #     if (sky_is_up and above_line) or (not sky_is_up and not above_line):
    #         p2 = points[0]
    #     else:
    #         p2 = points[1]
    #     p1x = int(np.round(x_perp))
    #     p1y = int(np.round(y_perp))
    #     p1 = (p1x, p1y)
    #     cv2.line(frame, p1, p2, (0,191,255), 1)



def detect_horizon_line(image_grayscaled):
    """Detect the horizon's starting and ending points in the given image
    The horizon line is detected by applying Otsu's threshold method to
    separate the sky from the remainder of the image.
    :param image_grayscaled: grayscaled image to detect the horizon on, of
     shape (height, width)
    :type image_grayscale: np.ndarray of dtype uint8
    :return: the (x1, x2, y1, y2) coordinates for the starting and ending
     points of the detected horizon line
    :rtype: tuple(int)
    """

    msg = ('`image_grayscaled` should be a grayscale, 2-dimensional image '
           'of shape (height, width).')
    assert image_grayscaled.ndim == 2, msg
    image_blurred = cv2.GaussianBlur(image_grayscaled, ksize=(3, 3), sigmaX=0)

    _, image_thresholded = cv2.threshold(
        image_blurred, thresh=0, maxval=1,
        type=cv2.THRESH_BINARY+cv2.THRESH_OTSU
    )
    image_thresholded = image_thresholded - 1
    image_closed = cv2.morphologyEx(image_thresholded, cv2.MORPH_CLOSE,
                                    kernel=np.ones((9, 9), np.uint8))

    # horizon_x1 = 0
    # horizon_x2 = image_grayscaled.shape[1] - 1
    # horizon_y1 = max(np.where(image_closed[:, horizon_x1] == 0)[0])
    # horizon_y2 = max(np.where(image_closed[:, horizon_x2] == 0)[0])

    # return horizon_x1, horizon_x2, horizon_y1, horizon_y2
    return image_closed



def keyboard_input():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # client.enableApiControl(True, "Drone2")
    # client.armDisarm(True, "Drone2")

    # client.takeoffAsync().join()

    print('Multirotor Initialized')
    print('Fly with the following controls')
    print('w: forward')
    print('s: backward')
    print('a: left')
    print('d: right')
    print('left arrow: yaw left')
    print('right arrow: yaw right')
    print('up arraw: up')
    print('down arrow: down')
    print('hold left shift to go faster')
    print('press esc to cancel flight and land')
    running = True
    control_freq = 60 # control update frequency in hertz
    control_period = 1/control_freq
    base_speed = 15 
    base_rotation_rate = 75 
    velocity = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    yaw_rate = 0.0
    while running:

        state = client.getMultirotorState()
        velocity[0] = 0.0
        velocity[1] = 0.0
        velocity[2] = 0.0   
        yaw_rate = 0.0
        q = state.kinematics_estimated.orientation.to_numpy_array()
        heading = np.arctan2(2.0*(q[2]*q[3] + q[0]*q[1]), 
                            q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])

        if keyboard.is_pressed('esc'):
            running = False
            cv2.destroyAllWindows()
            continue
        else:
            if keyboard.is_pressed('left_arrow'):
                yaw_rate = -base_rotation_rate
                client.rotateByYawRateAsync(yaw_rate, control_period)
                continue
            elif keyboard.is_pressed('right_arrow'):
                yaw_rate = base_rotation_rate 
                client.rotateByYawRateAsync(yaw_rate, control_period)
                continue
            if keyboard.is_pressed('w'):
                velocity[0] += np.cos(heading)
                velocity[1] += np.sin(heading)
            if keyboard.is_pressed('s'):
                velocity[0] -= np.cos(heading)
                velocity[1] -= np.sin(heading)
            if keyboard.is_pressed('a'):
                velocity[0] += np.sin(heading)
                velocity[1] -= np.cos(heading)
            if keyboard.is_pressed('d'):
                velocity[0] -= np.sin(heading)
                velocity[1] += np.cos(heading)
            if keyboard.is_pressed('up_arrow'):
                velocity[2] -= 1
            if keyboard.is_pressed('down_arrow'):
                velocity[2] += 1

            norm = np.sqrt(np.sum(velocity**2))
            if norm != 0.0:
                velocity /= norm 
            velocity *= base_speed
            if keyboard.is_pressed('left_shift'):
                velocity *= 2
            velocity[0] = (velocity[0]+state.kinematics_estimated.linear_velocity.x_val)/2
            velocity[1] = (velocity[1]+state.kinematics_estimated.linear_velocity.y_val)/2

            client.moveByVelocityAsync(velocity[0], velocity[1], velocity[2], control_period)

        # imu_data = client.getImuData()    
        # print("imu_data: time_stamp: %d, position: %s" % (imu_data.time_stamp, str(imu_data.angular_velocity)))  #  https://microsoft.github.io/AirSim/api_docs/html/#airsim.types.ImuData
    client.armDisarm(False)
    client.reset()
    client.enableApiControl(False)

t = threading.Thread(target=keyboard_input)
threads.append(t)

def video_output():
    client = airsim.MultirotorClient()
    client.confirmConnection()


    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    inkey = 'a'
    while inkey != 27:
        # get IMU data
        imu_data = client.getImuData()
        orientation = imu_data.orientation
        pitch, roll, _ = airsim.to_eularian_angles(orientation)

        in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
        img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
        image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3

        # img_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # # img_blur = cv2.GaussianBlur(img_gray, (15,15), 0)
        # # ret3,th3 = cv2.threshold(img_blur,250,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # # Sobel Edge Detection
        # # sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
        # # sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
        # #sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
        # # edges = cv2.Canny(image=img_blur, threshold1= 86, threshold2=170, apertureSize = 3) # Canny Edge
        # # horizon_x1, horizon_x2, horizon_y1, horizon_y2 = detect_horizon_line(img_gray)

        # img = cv2.cvtColor(detect_horizon_line(img_gray), cv2.COLOR_GRAY2BGR)
        # # create artificial horizon
        draw_horizon(image, -roll, pitch, 120, (0,0,255), True)
        # show image
        cv2.imshow('image', image)

        # cv2.imshow('image', image)
        inkey = cv2.waitKey(1)

    cv2.destroyAllWindows()

t = threading.Thread(target=video_output)
threads.append(t)

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
# client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
# client.armDisarm(True, "Drone2")

client.takeoffAsync(vehicle_name="Drone1").join()
# client.takeoffAsync(vehicle_name="Drone2").join()

client.moveToPositionAsync(0, 0, -10, 10, vehicle_name="Drone1").join()
# client.moveToPositionAsync(8, 0, -50, 50, vehicle_name="Drone2").join()

# client.moveToZAsync(-50, 200, vehicle_name="Drone1").join()
# client.moveToZAsync(-50, 200, vehicle_name="Drone2").join()

for t in threads:
    t.start()

for t in threads:
    t.join()


client.armDisarm(False, "Drone1")
# client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
# client.enableApiControl(False, "Drone2")