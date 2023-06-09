import cv2
import numpy as np
import glob

flag = False

for filename in sorted(glob.glob('/home/jj/Documents/AirSim/2023-06-09-09-57-52/images/*.ppm')):
    print(filename)
    img = cv2.imread(filename)
    if not flag:
        height, width, layers = img.shape
        frameSize = (width,height)
        out = cv2.VideoWriter('/home/jj/Documents/AirSim/2023-06-09-09-57-52/output_video_30.mp4',
                                cv2.VideoWriter_fourcc(*'mp4v'), 30, frameSize)
        flag = True
        # continue
    out.write(img)

out.release()