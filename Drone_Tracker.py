import numpy as np
import airsim
import keyboard
import cv2
import threading
import time
import pprint
import sys
threads =[]
bottom_size = 256
Altitude = -40
# connect to the AirSim simulator

client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")

client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

# airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name="Drone1").join()
f2 = client.takeoffAsync(vehicle_name="Drone2").join()
# f1.join()
# f2.join()

# state1 = client.getMultirotorState(vehicle_name="Drone1")
# s = pprint.pformat(state1)
# print("state: %s" % s)
# state2 = client.getMultirotorState(vehicle_name="Drone2")
# s = pprint.pformat(state2)
# print("state: %s" % s)
# airsim.wait_key('Press any key to move vehicles')

f1 = client.moveToPositionAsync(0, 0, Altitude, 30, vehicle_name="Drone1")
f2 = client.moveToPositionAsync(1, 0, Altitude, 30, vehicle_name="Drone2")
f1.join()
f2.join()

def keyboard_input():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # client.enableApiControl(True, "Drone1")
    # client.armDisarm(True, "Drone1")
    # client.moveToPositionAsync(0, 0, Altitude-1, 10, vehicle_name="Drone1").join()
    # client.enableApiControl(True, "Drone2")
    # client.armDisarm(True, "Drone2")
    # client.takeoffAsync().join()

    # print('Multirotor Initialized')
    # print('Fly with the following controls')
    # print('w: forward')
    # print('s: backward')
    # print('a: left')
    # print('d: right')
    # print('left arrow: yaw left')
    # print('right arrow: yaw right')
    # print('up arraw: up')
    # print('down arrow: down')
    # print('hold left shift to go faster')
    # print('press esc to cancel flight and land')
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
            # sys.exit("EXIT_keyboard_input")
            # return
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
    client.armDisarm(False)
    client.reset()
    client.enableApiControl(False)
    print('Done!')
    return

def drone_tracker():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Read first frame.    
    in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
    img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
    frame = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3

    # Define an initial bounding box
    bbox = (480, 280, 60, 20)
 
    #Uncomment the line below to select a different bounding box
    bbox = cv2.selectROI(frame, False)
 
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    while True:
        # Read a new frame
        # ok, frame = video.read()
        # if not ok:
        #     break
        in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
        img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
        frame = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3

        # Start timer
        timer = cv2.getTickCount()
 
        # Update tracker
        ok, bbox = tracker.update(frame)
 
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
 
        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
 
        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
     
        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
 
        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
    cv2.destroyAllWindows()

def drone_fly_circle():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, "Drone2")
    client.armDisarm(True, "Drone2")
    running = True
    index = 1
    while running:
        if keyboard.is_pressed('esc'):
            running = False
            # cv2.destroyAllWindows()
            # sys.exit("EXIT_drone_fly_circle")
            # return
            continue
        else:
            # client.moveToPositionAsync(7, -7, Altitude-3, 4, vehicle_name="Drone2").join()
            # print("Drone2: 7, -7, Altitude-3")
            # time.sleep(2.5)
            # client.moveToPositionAsync(1, 0, Altitude, 4, vehicle_name="Drone2").join()
            # print("Drone2: 1, 0, Altitude")
            # # time.sleep(2.5)
            # client.moveToPositionAsync(7, 7, Altitude-3, 4, vehicle_name="Drone2").join()
            # print("Drone2: 7, 7, Altitude-3")
            # # time.sleep(12.5)
            client.moveToPositionAsync(1*index, 0, Altitude-3, 3, vehicle_name="Drone2").join()
            index += 1
            client.moveToPositionAsync(1*index, 0, Altitude+1.5, 3, vehicle_name="Drone2").join()
            index += 1
            print(index)

    
    client.armDisarm(False, "Drone2")
    client.reset()
    client.enableApiControl(False, "Drone2")
    print('drone_fly_circle_Done!')
    return

def video_Edge():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # cv2.namedWindow('video_Edge', cv2.WINDOW_NORMAL)
    inkey = 'a'
    while inkey != 27:
        in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
        img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
        image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3
        image = image[0:256, 0:1024]
        img_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 
        # Sobel Edge Detection
        #sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
        #sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
        #sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
        edges = cv2.Canny(image=img_gray, threshold1=100, threshold2=200) # Canny Edge
        # ret, thresh = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        hull = []
        for i in range(len(contours)):
               hull.append(cv2.convexHull(contours[i], False))
        # create an empty black image
        drawing = np.zeros((edges.shape[0], edges.shape[1], 3), np.uint8)
        
        # draw contours and hull points
        for i in range(len(contours)):
            color_contours = (0, 255, 0) # green - color for contours
            color = (255, 0, 0) # blue - color for convex hull
            # draw ith contour
            cv2.drawContours(drawing, contours, i, color_contours, 1, 8, hierarchy)
            # draw ith convex hull object
            cv2.drawContours(drawing, hull, i, color, 1, 8)

        # Show in a window
        cv2.imshow('Contours', drawing)
        # cv2.imshow('video_Edge', edges)
        inkey = cv2.waitKey(1)

    cv2.destroyAllWindows()

def video_OpticalFlow():
    # client = airsim.MultirotorClient()
    # client.confirmConnection()

    cv2.namedWindow('OpticalFlow', cv2.WINDOW_NORMAL)
    inkey = 'a'
    in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
    img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
    old_image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3
    dimensions = old_image.shape
    print(dimensions, "/n")
    prvs = cv2.cvtColor(old_image,cv2.COLOR_BGR2GRAY)
    hsv = np.zeros_like(old_image)
    hsv[...,1] = 255

    while inkey != 27:
        in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
        img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
        image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3

        next = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.75, 9, 9, 3, 5, 1.2, 2)
        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

        cv2.imshow('OpticalFlow', rgb)
        prvs = next
        inkey = cv2.waitKey(1)

    cv2.destroyAllWindows()
    client.armDisarm(False)
    client.reset()
    client.enableApiControl(False)

def video_Absdiff():
    # cv2.namedWindow('Absdiff', cv2.WINDOW_NORMAL)
    inkey = 'a'
    in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
    img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
    old_image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3
    old_image = old_image[0:256, 0:1024]
    prvs = cv2.cvtColor(old_image,cv2.COLOR_BGR2GRAY)

    while inkey != 27:
        in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
        img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
        image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3
        image = image[0:256, 0:1024]
        next = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        grey_diff = cv2.absdiff(prvs, next)
        cv2.imshow('Absdiff', grey_diff)
        prvs = next
        inkey = cv2.waitKey(1)

    cv2.destroyAllWindows()
    client.armDisarm(False)
    client.reset()
    client.enableApiControl(False)

# running = threading.Event()
# running.set()
# video_Edge()
# video_Absdiff()
#keyboard_input()
drone_fly_circle()
# t = threading.Thread(target=drone_fly_circle,)
# threads.append(t)

# # t = threading.Thread(target=video_Absdiff)
# # threads.append(t)

# # t = threading.Thread(target=video_OpticalFlow)
# # threads.append(t)

# # t = threading.Thread(target=video_Edge)
# # threads.append(t)

# # t = threading.Thread(target=drone_tracker)
# # threads.append(t)

# t = threading.Thread(target=keyboard_input, )
# threads.append(t)


# for t in threads:
#     t.start()

# # running.clear()

# for t in threads:
#     t.join()

# airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
sys.exit("EXIT")
quit()