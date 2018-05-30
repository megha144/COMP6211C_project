#!/usr/bin/env python
##### Acknowledgement: Part of code at the courtesy of flytxtds

#### The function in this file deal with one single image at a time

### import required libraries ###
### file/folder manipulation ###
#from urllib.request import urlretrieve
import roslib
roslib.load_manifest("scene_recognition")

import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from geometry_msgs.msg import Point32, PointStamped
from sensor_msgs.msg import PointCloud
import tf

from os import listdir
from os.path import isfile, join, exists
#from zipfile import ZipFile
import pickle
##for random split of dataset into a training set and a test set ###
from sklearn.model_selection import train_test_split
## image processing routines for feature extraction/transformation##
from skimage.feature import daisy,hog
from skimage import io
from skimage.color import rgb2gray
import skimage
import numpy as np
import pandas as pd
from tqdm import tqdm
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans
from sklearn.cluster import MiniBatchKMeans
from sklearn import svm

from sklearn import metrics
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
import datetime

import matplotlib.pyplot as plt
#%matplotlib inline

class scene_recognition:
    def __init__(self):
        rospy.loginfo("Initializing Scene Recognition")
        ##Load pretrained model
        self.daisy_cluster_model = pickle.load(open('daisy_cluster.sav','rb'))
        self.hybrid_classifier = pickle.load(open('svm_model.sav','rb'))
        self.bridge = CvBridge()
        self.image_sub = self.image_sub = rospy.Subscriber("/vrep/image", Image, self.callback);
        self.region = ("Region A","Region B","Region C","Region D")
        self.count = -1

    def callback(self, data):
        try:
           self.count+=1
           frame = self.bridge.imgmsg_to_cv2(data,"bgr8")
           frame = cv2.flip(frame,1)
           filename = "/home/steve/tmp/{}.jpeg".format(self.count)
           cv2.imwrite(filename,frame)
           self.predict_scene(filename)	    
        except CvBridgeError as e:
            rospy.loginfo(e)
       	

    ##Need to modify these functions
    def extract_daisy_and_hog_features_from_image_nf(self,file_path,daisy_step_size=32,daisy_radius=32,hog_pixels_per_cell=16,hog_cells_per_block=1):
        img = io.imread(file_path)
        img_gray = rgb2gray(img)
        img=skimage.transform.resize(img_gray,(512,512)) ##resize to a suitable dimension, avg size of images in the dataset
        #original, histograms=6
        descs = daisy(img, step=daisy_step_size, radius=daisy_radius, rings=2, histograms=8,orientations=8, visualize=False)
        #calculate daisy feature descriptors
        descs_num = descs.shape[0] * descs.shape[1]
        daisy_desriptors=descs.reshape(descs_num,descs.shape[2])
        hog_desriptor=hog(img, orientations=8, pixels_per_cell=(hog_pixels_per_cell, hog_pixels_per_cell),cells_per_block=(hog_cells_per_block, hog_cells_per_block), visualise=False,feature_vector=True)
        return daisy_desriptors,hog_desriptor

    def extract_daisy_hog_hybrid_feature_from_image_nf(self,img,daisy_cluster_model):
        #incase if we have encountered the file during training, the daisy and hog features would already have been computed
        daisy_features,hog_feature=self.extract_daisy_and_hog_features_from_image_nf(img,daisy_step_size=8,daisy_radius=8)
        
        ##find to which clusters each daisy feature belongs
        img_clusters=daisy_cluster_model.predict(daisy_features) 
        cluster_freq_counts=pd.DataFrame(img_clusters,columns=['cnt'])['cnt'].value_counts()
        bovw_vector=np.zeros(daisy_cluster_model.n_clusters) ##feature vector of size as the total number of clusters
        for key in cluster_freq_counts.keys():
            bovw_vector[key]=cluster_freq_counts[key]

        bovw_feature=bovw_vector/np.linalg.norm(bovw_vector)
        hog_feature=hog_feature/np.linalg.norm(hog_feature)
        return list(bovw_feature)+list(hog_feature)

    def predict_scene(self,img):
        XTEST = np.nan_to_num(self.extract_daisy_hog_hybrid_feature_from_image_nf(img,self.daisy_cluster_model))
        XTEST = XTEST.reshape(1,-1)
        hybridpred = self.hybrid_classifier.predict(XTEST)
        hybridpred = hybridpred.astype(np.int64)
        print('Predicted Region at time is '+self.region[hybridpred[0]])

### Need to be done, find images from vrep

### Prediction 
#hybridpred=hybrid_classifier.predict(XTEST)

def main(args):
    sr = scene_recognition();
    rospy.init_node('scene_recognition', anonymous=True)
    rospy.loginfo('Initialize Node Scene Recognition')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
