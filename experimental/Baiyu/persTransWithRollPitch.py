import cv2
import numpy as np
import math
from numpy.linalg import inv

f = 0.0041					#focal length
d = 0.20					#height of camera
alpha = (math.pi*20)/180	#vertical fov/2
beta = (math.pi*27.5)/180	#horizontal fov/2
#rotation of camera
roll = 0
pitch = 49*math.pi/180
yaw = 0

img = cv2.imread('.jpg')
height, width, chanels = img.shape
img = cv2.resize(img, (width/2, height/2)) 
height, width, chanels = img.shape

p1 = np.matrix([[0],[0]])
p2 = np.matrix([[1632],[0]])
p3 = np.matrix([[0],[1224]])
p4 = np.matrix([[1632],[1224]])

#shift pixel coordinate
pAll = [p1,p2,p3,p4]
for i in range(0,4):
	pAll[i] = pAll[i]-np.matrix([[width/2],[height/2]])

#pixel coordinate to image plane coordinate
mat3dAll = []
for i in range(0,4):
	temp3d = np.matrix([\
		[pAll[i][1, 0]*2*f*math.tan(alpha)/height]\
		,[pAll[i][0, 0]*2*f*math.tan(beta)/width]\
		,[-f]\
		,[1]])
	mat3dAll.append(temp3d)

#project point in image plane to ground
for i in range(0,4):
	mat3dAll[i] = mat3dAll[i]*(d/f)
	mat3dAll[i][3, 0] = 1

#roatation matrix
Rx = np.matrix([[1, 0, 0, 0],\
				[0, math.cos(roll),-math.sin(roll), 0],\
				[0, math.sin(roll), math.cos(roll), 0],\
				[0, 0, 0, 1]])
Ry = np.matrix([[math.cos(pitch), 0, math.sin(pitch), 0],\
				[0, 1, 0, 0],\
				[-math.sin(pitch), 0, math.cos(pitch), 0],\
				[0, 0, 0, 1]])
Rall = Ry*Rx
translationMat = np.matrix([[1, 0, 0, d*math.tan(pitch)/math.cos(roll)],\
							[0, 1, 0, -d*math.tan(roll)],\
							[0, 0, 1, 0],\
							[0, 0, 0, 1]])
rollTransMat = translationMat*Rall
print rollTransMat

#points relative to new camera in 3d coordinate
for i in range(0,4):
	mat3dAll[i] = inv(rollTransMat)*mat3dAll[i]

#projection matrix
projectMat = np.matrix([[1, 0, 0, 0],\
						[0, 1, 0, 0],\
						[0, 0, 1, 0],\
						[0, 0, -1/f, 0]])
#project points to image plane 3d coordinate
for i in range(0, 4):
	mat3dAll[i] = projectMat*mat3dAll[i]
	mat3dAll[i] = mat3dAll[i]/mat3dAll[i][3]

#transform points in 3d image plane to pixel coordinate
p1New = np.matrix([[mat3dAll[0][1, 0]*width/(2*f*math.tan(beta))+width/2],\
					[mat3dAll[0][0,0]*height/(2*f*math.tan(alpha))+height/2]])
p2New = np.matrix([[mat3dAll[1][1, 0]*width/(2*f*math.tan(beta))+width/2],\
					[mat3dAll[1][0,0]*height/(2*f*math.tan(alpha))+height/2]])
p3New = np.matrix([[mat3dAll[2][1, 0]*width/(2*f*math.tan(beta))+width/2],\
					[mat3dAll[2][0,0]*height/(2*f*math.tan(alpha))+height/2]])
p4New = np.matrix([[mat3dAll[3][1, 0]*width/(2*f*math.tan(beta))+width/2],\
					[mat3dAll[3][0,0]*height/(2*f*math.tan(alpha))+height/2]])

#apply perspective transform
pts1 = np.float32([[300+p1[0, 0],300+p1[1, 0]],[300+p2[0, 0],300+p2[1, 0]],[300+p3[0, 0],300+p3[1, 0]],[300+p4[0, 0],300+p4[1, 0]]])
pts2 = np.float32([[p1New[0, 0],p1New[1, 0]],[p2New[0, 0],p2New[1, 0]],[p3New[0, 0],p3New[1, 0]],[p4New[0, 0],p4New[1, 0]]])

M = cv2.getPerspectiveTransform(pts2,pts1)
print M
dst = cv2.warpPerspective(img,M,(width*2,height*2))
cv2.imwrite('transformed.jpg',dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
