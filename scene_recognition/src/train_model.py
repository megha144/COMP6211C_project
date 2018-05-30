##### Acknowledgement: Part of code at the courtesy of flytxtds
### import required libraries ###
### file/folder manipulation ###
#from urllib.request import urlretrieve
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

import matplotlib.pyplot as plt
#%matplotlib inline

def get_filenames(path):
    onlyfiles = [path+f for f in listdir(path) if isfile(join(path, f))]
    return onlyfiles

def extract_daisy_and_hog_features_from_image(file_path,daisy_step_size=32,daisy_radius=32,hog_pixels_per_cell=16,hog_cells_per_block=1):
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

def cluster_daisy_features(daisy_feature_list,number_of_clusters):
    #km=KMeans(n_clusters=number_of_clusters)
    km=MiniBatchKMeans(n_clusters=number_of_clusters,batch_size=number_of_clusters*10)
    km.fit(daisy_feature_list)
    return km

def extract_daisy_hog_hybrid_feature_from_image(fname,daisy_cluster_model):
    #incase if we have encountered the file during training, the daisy and hog features would already have been computed
    if fname in training_data_feature_map:
        daisy_features=training_data_feature_map[fname][0]
        hog_feature=training_data_feature_map[fname][1]
    else:
        daisy_features,hog_feature=extract_daisy_and_hog_features_from_image(fname,daisy_step_size=8,daisy_radius=8)
        
    ##find to which clusters each daisy feature belongs
    img_clusters=daisy_cluster_model.predict(daisy_features) 
    cluster_freq_counts=pd.DataFrame(img_clusters,columns=['cnt'])['cnt'].value_counts()
    bovw_vector=np.zeros(daisy_cluster_model.n_clusters) ##feature vector of size as the total number of clusters
    for key in cluster_freq_counts.keys():
        bovw_vector[key]=cluster_freq_counts[key]

    bovw_feature=bovw_vector/np.linalg.norm(bovw_vector)
    hog_feature=hog_feature/np.linalg.norm(hog_feature)
    return list(bovw_feature)+list(hog_feature)

base_path="output/" #############Need to be changed

############### Load file names corresponding to each scene category into lists
category_names=listdir(base_path) ##
for i in range(len(category_names)):
    print(category_names[i],'=',i)
print('total categories:',len(category_names))
dataset_filenames=[] ##list to keep path of all files in the database
dataset_labels=[]
##category_names.index('store')  list the numeric representation of the category
##category_names[0] list the text representation of the category id
for category in category_names:
    category_filenames=get_filenames(base_path+category+"/")##get all the filenames in that category
    category_labels=np.ones(len(category_filenames))*category_names.index(category) ##label the category with its index position
    dataset_filenames=dataset_filenames+category_filenames
    dataset_labels=dataset_labels+list(category_labels)

print('total dataset size:',len(dataset_filenames)) ###Print total dataset size

#### Extract features from training datasplit for downstream processing, takes aprox 12 mins for a standard laptop
training_data_feature_map={} ##map to store daisy feature as well as hog feature for all training datapoints
daisy_descriptor_list=[] ##list to store all daisy descriptors to form our visual vocabulary by clustering
counter=0
#for fname in tqdm(train_filenames):
for fname in tqdm(dataset_filenames):
    daisy_features,hog_feature=extract_daisy_and_hog_features_from_image(fname,daisy_step_size=8,daisy_radius=8)
    ###extract DAISY features and HOG features from the image and save in a map###
    training_data_feature_map[fname]=[daisy_features,hog_feature]
    daisy_descriptor_list=daisy_descriptor_list+list(daisy_features)

### hide warnings ##
import warnings
warnings.filterwarnings('ignore')

daisy_cluster_model=cluster_daisy_features(daisy_descriptor_list,200) 
#daisy_cluster_model.n_clusters
filename1 = "daisy_cluster.sav"
pickle.dump(daisy_cluster_model, open(filename1, 'wb'),protocol=2)
daisy_cluster_model = pickle.load(open(filename1, 'rb'))
#loaded_model.n_clusters

XTRAIN=[]
YTRAIN=[]
for i in tqdm(range(len(dataset_filenames))):
    XTRAIN.append(np.nan_to_num(extract_daisy_hog_hybrid_feature_from_image(dataset_filenames[i],daisy_cluster_model)))
    YTRAIN.append(dataset_labels[i])

#### TRAIN A linear SVM CLASSIFIER and GET ACCURACY REPORT ###
hybrid_classifier=svm.LinearSVC()
hybrid_classifier.fit(XTRAIN,YTRAIN)
hybridpred = hybrid_classifier.predict(XTRAIN)
print('Overall accuracy:',accuracy_score(YTRAIN,hybridpred))

filename2 = "svm_model.sav"
pickle.dump(hybrid_classifier, open(filename2, 'wb'),protocol =2)
#pickle.dump(your_object, your_file, protocol=2)
#loaded_model = pickle.load(open(filename2, 'rb'))
