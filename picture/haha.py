import cv2 
import numpy as np
import sys 
from matplotlib import pyplot as plt

img1 = cv2.imread(sys.argv[1])
gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

img2 = cv2.imread(sys.argv[2])
gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(gray1,None)
kp2, des2 = sift.detectAndCompute(gray2,None)

bf = cv2.BFMatcher()
matches = bf.knnMatch(des1,des2, k=2)
# Apply ratio test
good = []
index = []
for m,n in matches:
    if m.distance < 0.75*n.distance:
        good.append(m)
        index.append(m.queryIdx)
# cv.drawMatchesKnn expects list of lists as matches.

print good 
print index 

display_kp1 = [kp1[i] for i in index]
print display_kp1
print display_kp1[0].pt
print display_kp1[0].pt[0]
print display_kp1[0].pt[1]


cv2.drawKeypoints(gray1, kp1, img1, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('img1',img1)
cv2.drawKeypoints(gray2, kp2, img2, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('img2',img2)

cv2.waitKey(0)
