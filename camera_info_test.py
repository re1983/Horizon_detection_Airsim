import airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()

camera_info = client.simGetCameraInfo("0")

print("Camera Info:", camera_info)

proj_mat = camera_info.proj_mat

print("proj_mat:", proj_mat)

focal_length = client.simGetFocalLength("0")

print("Focal Length:", focal_length)

focal_Distance = client.simGetFocusDistance("0")

print("Focal Distance:", focal_Distance)

FOV = client.simGetCurrentFieldOfView("0")

print("FOV:", FOV)

simGetLensSettings = client.simGetLensSettings("0")

print("LensSettings:", simGetLensSettings)

simGetFilmbackSettings = client.simGetFilmbackSettings("0")

print("FilmbackSettings:", simGetFilmbackSettings)

simGetFocusAperture = client.simGetFocusAperture("0")

print("FocusAperture:", simGetFocusAperture)

DistortionParams = client.simGetDistortionParams("0")

print("DistortionParams:", DistortionParams)