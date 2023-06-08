import cv2
import numpy as np
import glob

flag = False

for filename in sorted(glob.glob('/home/jj/Documents/AirSim/2023-06-08-16-28-15/images/*.ppm')):
    print(filename)
    img = cv2.imread(filename)
    if not flag:
        height, width, layers = img.shape
        frameSize = (width,height)
        out = cv2.VideoWriter('/home/jj/Documents/AirSim/2023-06-08-16-28-15/output_video_15.mp4',
                                cv2.VideoWriter_fourcc(*'mp4v'), 15, frameSize)
        flag = True
        # continue
    out.write(img)

out.release()