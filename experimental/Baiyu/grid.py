import cv2
import numpy as np
from operator import itemgetter, attrgetter, methodcaller
from numpy.linalg import inv
import math
import cPickle as pickle


def PointsToPolar(x1,y1,x2,y2):
  if x1 == x2:
    return 0, x1
  x12Mat = np.matrix([[x1,1],[x2,1]])
  x12Mat = inv(x12Mat)
  y12Mat = np.matrix([[y1],[y2]])
  mbMat = np.dot(x12Mat,y12Mat)
  m = mbMat[0,0]
  b = mbMat[1,0]
  distance = abs(b)/math.sqrt(math.pow(m,2)+1)
  if(b < 0):#intersect above
    distance =  distance
    theta = - np.pi/2 + math.atan(m)
  else:
    theta = np.pi/2 + math.atan(m)
  return theta, distance


def intersect(p1,p2):
        mat = np.matrix(((math.cos(p1[1]),-math.sin(p1[1])),(math.cos(p2[1]),-math.sin(p2[1]))))
        mat = inv(mat)
        dis = np.matrix((p1[0],p2[0]))
        dis = dis.transpose()
        result = np.dot(mat,dis)
        return result

#parameters
minLineLength = 80
maxGapLength = 15
#
#get the image and preprocessing
img = cv2.imread("top 2.png")
img = cv2.resize(img, (1280, 800))
#cv2.imwrite('arena 3.jpg', img)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_white = np.array([0, 0, 180])
upper_white = np.array([180, 100, 255])
mask = cv2.inRange(hsv, lower_white, upper_white)
cross = cv2.bitwise_and(img, img, mask=mask)
cv2.imshow('img', img)
#cv2.imshow('mask', mask)
#cv2.imshow('cross', cross)
#img = cv2.resize(img,(60,60))
cross = cv2.medianBlur(cross,5)
gray = cv2.cvtColor(cross,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)
cv2.imshow("edge",edges)
#cv2.imwrite('edgeCross2.jpg',edges)
lines = cv2.HoughLinesP(edges,1,np.pi/180,80,minLineLength,maxGapLength)
#lines = cv2.HoughLines(edges,1,np.pi/180,80)


if lines != None:
  temp = []

  for x1,y1,x2,y2 in lines[0]:
      cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
      angle,dis = PointsToPolar(x1,y1,x2,y2)
      temp.append((dis,angle))



  #directly process if lines is polar
  good = []
  #temp = []
  arr1 = []
  arr2 = []
  hight, width, channel = img.shape


  '''
  for m,n in lines[0]:
      temp.append((m,n))
  '''
# sort with distance
  temp.sort(key=lambda elem: elem[1])
  lens = len(temp)
  goodlen = 0
  anglethres = 0.3
  prevangle = temp[0][1]
# delete multiple overlapping lines
  #for thing in temp:
  #  good.append(thing)

  cluster = [temp[0]]
  distancethres = 30
  for x in range(1, len(temp)):
    if (temp[x][1] - prevangle > anglethres):
      if len(cluster) > 0:
        cluster.sort(key=lambda elem: elem[0])
       # print cluster
        prevdist= cluster[0][0]
        previndex = 0
        for y in xrange(1, len(cluster)):
          if cluster[y][0] - prevdist > distancethres:
            index = int((y-1-previndex)/2)+previndex
            #print cluster[index][0], cluster[index][1]
            good.append([cluster[index][0], cluster[index][1]])
            goodlen += 1
            prevdist = cluster[y][0]
            previndex = y
        index = int((len(cluster)-1-previndex)/2)+previndex
        #print cluster[index][0], cluster[index][1]
        good.append([cluster[index][0], cluster[index][1]])
        goodlen += 1
      prevangle = temp[x][1]
      cluster = []
    else:
      cluster.append(temp[x])  
  if len(cluster) > 0:
      cluster.sort(key=lambda elem: elem[0])
      #print cluster
      prevdist= cluster[0][0]
      previndex = 0
      for y in xrange(1, len(cluster)):
        if cluster[y][0] - prevdist > distancethres:
          index = int((y-1-previndex)/2)+previndex
          #print cluster[index][0], cluster[index][1]
          good.append([cluster[index][0], cluster[index][1]])
          goodlen += 1
          prevdist = cluster[y][0]
          previndex = y
      index = int((len(cluster)-1-previndex)/2)+previndex
      #print cluster[index][0], cluster[index][1]
      good.append([cluster[index][0], cluster[index][1]])
      goodlen += 1
  
  #calculate angle
  print goodlen

  for x in range(0,goodlen):
    for y in range(x+1,goodlen):
        if abs(good[x][1]-good[y][1])>0.7 and abs(good[x][1]-good[y][1])<0.9:
              A = good[x]
              B = good[y]

  '''   
  if A is not None:
    intesec = intersect(A,B)
    print intesec[0,0]
    cv2.circle(img,(int(intesec[0,0]),-int(intesec[1,0])),10,255,-1)
  '''
  #print good
  for rho,theta in good:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
      
        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)


# write lines to pickle file for line tracking later
# may be replaced by ROS
pickle.dump(good, open("lines2_top2.pickle", 'w'))

cv2.imshow('grid',img)
cv2.imwrite('top 2 grid.png', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
