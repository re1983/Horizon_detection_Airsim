import cv2
import numpy as np
import glob

fourcc = -1 #cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4',fourcc, 15, (200, 480))

# OpenCV: FFMPEG: format mp4 / MP4 (MPEG-4 Part 14)
# fourcc tag 0x7634706d/'mp4v' codec_id 000C
# fourcc tag 0x31637661/'avc1' codec_id 001B
# fourcc tag 0x33637661/'avc3' codec_id 001B
# fourcc tag 0x31766568/'hev1' codec_id 00AD
# fourcc tag 0x31637668/'hvc1' codec_id 00AD
# fourcc tag 0x7634706d/'mp4v' codec_id 0002
# fourcc tag 0x7634706d/'mp4v' codec_id 0001
# fourcc tag 0x7634706d/'mp4v' codec_id 0007
# fourcc tag 0x7634706d/'mp4v' codec_id 003D
# fourcc tag 0x7634706d/'mp4v' codec_id 0058
# fourcc tag 0x312d6376/'vc-1' codec_id 0046
# fourcc tag 0x63617264/'drac' codec_id 0074
# fourcc tag 0x7634706d/'mp4v' codec_id 00A3
# fourcc tag 0x39307076/'vp09' codec_id 00A7
# fourcc tag 0x31307661/'av01' codec_id 00E2
# fourcc tag 0x6134706d/'mp4a' codec_id 15002
# fourcc tag 0x63616c61/'alac' codec_id 15010
# fourcc tag 0x6134706d/'mp4a' codec_id 1502D
# fourcc tag 0x6134706d/'mp4a' codec_id 15001
# fourcc tag 0x6134706d/'mp4a' codec_id 15000
# fourcc tag 0x332d6361/'ac-3' codec_id 15003
# fourcc tag 0x332d6365/'ec-3' codec_id 15028
# fourcc tag 0x6134706d/'mp4a' codec_id 15004
# fourcc tag 0x61706c6d/'mlpa' codec_id 1502C
# fourcc tag 0x43614c66/'fLaC' codec_id 1500C
# fourcc tag 0x7375704f/'Opus' codec_id 1503C
# fourcc tag 0x6134706d/'mp4a' codec_id 15005
# fourcc tag 0x6134706d/'mp4a' codec_id 15018
# fourcc tag 0x6134706d/'mp4a' codec_id 15047
# fourcc tag 0x7334706d/'mp4s' codec_id 17000
# fourcc tag 0x67337874/'tx3g' codec_id 17005
# fourcc tag 0x646d7067/'gpmd' codec_id 1800A
# fourcc tag 0x316d686d/'mhm1' codec_id 1505B
# fourcc tag 0x70707473/'stpp' codec_id 17018
# fourcc tag 0x70786664/'dfxp' codec_id 17018
