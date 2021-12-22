import os
import csv
import cv2
import numpy as np
import sklearn
import matplotlib.pyplot as plt
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers.core import Dense, Flatten, Activation, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers import Lambda, Cropping2D

samples = []
csv_path = './data/driving_log.csv'
img_path = './data/IMG/'
#load data from csv file
with open(csv_path) as csvfile: 
    file = csv.reader(csvfile)
    next(file, None)
    for line in file: 
        samples.append(line)
#split collected data as trining and, validtion set
train_samples, validation_samples = train_test_split(samples, test_size=0.2)
#generator function that preprocess data (color space conversion), augment data (add flipped version) - output is (image and angle) array
def generator(samples, batch_size=32):
    num_samples = len(samples)
   
    while 1: 
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):            
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
                    for i in range(0,3):                        
                        name = img_path+batch_sample[i].split('/')[-1]
                        center_image = cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB) #color space conversion
                        
                        center_angle = float(batch_sample[3])
                        images.append(center_image)                                               
                        if(i==0):
                            angles.append(center_angle)
                        elif(i==1):
                            angles.append(center_angle+0.2)#left cam
                        elif(i==2):
                            angles.append(center_angle-0.2)#right cam
                        #augment by flipping
                        images.append(cv2.flip(center_image,1))#data augment by flipping
                        if(i==0):
                            angles.append(center_angle*-1)
                        elif(i==1):
                            angles.append((center_angle+0.2)*-1)
                        elif(i==2):
                            angles.append((center_angle-0.2)*-1)
        
            X_train = np.array(images)
            y_train = np.array(angles)            
            yield sklearn.utils.shuffle(X_train, y_train)
            
def get_nvidia_model():
    model = Sequential()

    model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))#normlize image pixels
    model.add(Cropping2D(cropping=((70,25),(0,0))))#preprocess by cropping image

    model.add(Convolution2D(24,5,5,subsample=(2,2),activation="relu"))
    model.add(Dropout(0.1))
    model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
    model.add(Dropout(0.1))
    model.add(Convolution2D(48,5,5,subsample=(2,2),activation="relu"))
    model.add(Dropout(0.1))
    model.add(Convolution2D(64,3,3,activation="relu"))
    model.add(Dropout(0.1))
    model.add(Convolution2D(64,3,3,activation="relu"))
    model.add(Dropout(0.1))

    model.add(Flatten())
    
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))
    return model

 
train_generator      = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)

nvidia_model = get_nvidia_model()

#mean square error as loss function, adam optimizer as optimizer
nvidia_model.compile(loss='mse',optimizer='adam')

nvidia_model.fit_generator(train_generator, samples_per_epoch= len(train_samples), validation_data=validation_generator,   nb_val_samples=len(validation_samples), nb_epoch=5, verbose=1)

nvidia_model.save('model2.h5')
nvidia_model.summary()
print('model saved!')

