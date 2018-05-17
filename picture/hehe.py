import numpy as np
import cv2
import os 
import sys

subjects = ["Barack Obama", "Avril Lavigne", "Zhang Guorong", "Legolas", "Levi Rivaille"]
def detect_face(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier('lbpcascade_frontalface.xml')
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
    if (len(faces)==0):
        return None, None
    (x,y,w,h) = faces[0]
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    cv2.imshow('img',img)
    cv2.imshow('face resize', cv2.resize(gray[y:y+w, x:x+h],(50,50)))
    cv2.waitKey(0)
    return cv2.resize(gray[y:y+w, x:x+h],(50,50)),faces[0]

def prepare_training_data(data_folder_path):
    
    #------STEP-1--------
    #get the directories (one directory for each subject) in data folder
        dirs = os.listdir(data_folder_path)
    
    #list to hold all subject faces
        faces = []
    #list to hold labels for all subjects
        labels = []
    
    #let's go through each directory and read images within it
        for label in range(5):
            subject_dir_path = data_folder_path + "/" + str(label)
    
            subject_images_names = os.listdir(subject_dir_path)
        
            for image_name in subject_images_names:

                image_path = subject_dir_path + "/" + image_name

            #read image
                image = cv2.imread(image_path)
            #display an image window to show the image 
            	cv2.imshow("Training on image...", image)
                cv2.waitKey(100)

            
            #detect face
                face, rect = detect_face(image)
                if face is not None:
                #add face to list of faces
                    faces.append(face)
                #add label for this face
                    labels.append(label)
    
        return faces, labels

def draw_rectangle(img, rect):
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
 
#function to draw text on give image starting from
#passed (x, y) coordinates. 
def draw_text(img, text, x, y):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)


def predict(test_img):
#make a copy of the image as we don't want to change original image
    img = test_img.copy()
#detect face from the image
    face, rect = detect_face(img)
 
#predict the image using our face recognizer 
    label= face_recognizer.predict(face)
    print label 
#get name of respective label returned by face recognizer
    label_text = subjects[label[0]]
    print label_text
#draw a rectangle around face detected
    draw_rectangle(img, rect)
#draw name of predicted person
    draw_text(img, label_text, rect[0], rect[1]-5)
 
    return img


data_folder_path = sys.argv[1]
faces, labels = prepare_training_data(data_folder_path)
print len(faces)
print len(labels)
face_recognizer = cv2.face.LBPHFaceRecognizer_create()
face_recognizer.train(faces, np.array(labels))

test_image = cv2.imread(sys.argv[2])
lala = predict(test_image)
cv2.imshow("lala",lala)
cv2.waitKey(0
)


