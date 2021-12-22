# **Behavioral Cloning** 

## Writeup 

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

I added following files:

* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup.md summarizing the results

#### 2. Submission includes functional code
Using the Udacity simulator and my drive.py file, the one can see how my model drives autonomously around the track by executing 

$python drive.py model.h5


#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. 
The file includes the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

pipeline basiclly consists of 4 steps:
  1- color space conversion
  2- data augmentation  
  3- normalization
  4- cropping images
  5- neural network model (explined below fields in detail)    

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed
Both color conversion and data augmentation were made in generator function in line 61
Preprocessing:
I converted images from BGR to RGB color space in line 38
Data augmenttion:
I augmented data adding flipped images from all 3 cameras between lines in 49-55

Model:
used nvidia's model in get_nvidia_model function with some additional differences between in line 61-84
Normalized each pixel in input images then cropped images in line 64 and 65.
I used lambda for ease to operate process on each image for normalization and Cropping2D for cropping.

#### 2. Attempts to reduce overfitting in the model
I augmented data adding flipped images from all 3 cameras.
I implemented 10% dropout after all convolution layers 

#### 3. Model parameter tuning
I used an adam optimizer, so the learning rate was not tuned manually in line 93

#### 4. Appropriate training data
I collected data using Udacity simulator.
Then stopped recording and got closer to road borders then started recording and moved towards to the middle of the road.

I augmented data adding flipped images from all 3 cameras as I stated above.
Examples folder includes 3 samples from each camera.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach
The overall strategy is deriving a model that is similar to nvidia's explained network. I applied all dditional %10 dropout fter ech conv layer to avoid overfitting.


#### 2. Final Model Architecture
The final model architecture is in get_nvidia_model function that invoking get_nvidia_model in line 90.

Model architecture:

Layer 	Size
cropped input 	65 x 320 x 3
normalization 	65 x 320 x 3
1st Convolution with relu activation 	5 x 5 x 24 with 2x2 filters
Dropout %10	
2nd Convolution with relu activation 	5 x 5 x 36 with 2x2 filters
Dropout %10	
3rd Convolution with relu activation 	5 x 5 x 48 with 2x2 filters
Dropout %10	
4th Convolution with relu activation 	3 x 3 x 64
Dropout %10	
5th Convolution with relu activation 	3 x 3 x 64
Dropout %10	
Flatten
1st fully connected layer	100
2nd fully connected layer	50
3rd fully connected layer	10
output layer 				1

#### 3. Creation of the Training Set & Training Process
in gpu mode, i trained model (hyperprmeters: 5 epochs, 32 batch size) executing:
$python model.py

and got model.h5 file.

#### 4. Getting output video
then i collected data in run folder executing simulator and
$python drive.py model.h5 run
command together.

then i passed to cpu mode and created video file executing
$python video.py run



