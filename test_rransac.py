from se2 import SE2
from so2 import SO2

from ctypes import *

libc = cdll.LoadLibrary("/home/jj/Work/Horizon_detection_Airsim/librransac.so")

print(type(libc))
# libc.rransac = libc.rransac
