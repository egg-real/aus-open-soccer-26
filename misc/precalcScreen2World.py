import numpy as np
from math import cos, sin, atan2, tan, radians, degrees, hypot
#raw camera specs
cameraResX = 640
cameraResY = 360
cameraZ = 158 
cameraY = 37.227
cameraAOD = radians(30)
hFOV = radians(81)
distortionPercent =5 / 100
#calculated values
rMax = hypot(((cameraResX-1)/2), ((cameraResY-1)/2))
distortionCoefficient = ((1 / (1 + distortionPercent)) - 1) / (rMax * rMax)
print(distortionCoefficient)
raise SystemExit
div = ((cameraResX-1)/2) / tan(hFOV/2)

screenToWorldCartesian = np.zeros((cameraResX,cameraResY,2))
screenToWorldPolar = np.zeros((cameraResX,cameraResY,2))


for yPixelRaw in range(360):
    # z up, y forward, x right
    for xPixelRaw in range(640):
        r = hypot(xPixelRaw-((cameraResX-1)/2),yPixelRaw - ((cameraResY-1)/2))
        xPixel = (xPixelRaw - ((cameraResX-1)/2)) / (1 + (r * r * distortionCoefficient))
        yPixel = (((cameraResY-1)/2) - yPixelRaw) / (1 + (r * r * distortionCoefficient))
        z = (yPixel / div) * cos(cameraAOD) - sin(cameraAOD)
        y = cos(cameraAOD) + (yPixel / div) * sin(cameraAOD) 
        x = (xPixel) / div
        pos = np.array([x,y,z])
        screenToWorldCartesian[xPixelRaw,yPixelRaw] = (pos * cameraZ / -pos[2])[:2]
        screenToWorldCartesian[xPixelRaw,yPixelRaw,1] += cameraY
        screenToWorldPolar[xPixelRaw,yPixelRaw,0] = degrees(atan2(*screenToWorldCartesian[xPixelRaw,yPixelRaw]))
        screenToWorldPolar[xPixelRaw,yPixelRaw,1] = np.linalg.vector_norm(screenToWorldCartesian[xPixelRaw,yPixelRaw])
#kebab case to p*ss everyone off
print(screenToWorldPolar - np.load("Github/maix/screen-2-world-polar-5pc-distortion.npy"))
# screenToWorldPolar = np.load("Github/maix/screen-2-world-polar.npy")
# print(screenToWorldPolar)
#screenToWorldPolar[screenToWorldPolar[:,:,1] > 3500] = np.inf
np.save("Github/maix/screen-2-world-polar.npy",screenToWorldPolar.astype(np.float16))
