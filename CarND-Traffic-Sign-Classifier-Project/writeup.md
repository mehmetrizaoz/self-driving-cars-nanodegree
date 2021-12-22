# **Traffic Sign Recognition** 

## Writeup

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/visualization.jpg "Visualization"
[image2]: ./examples/grayscale.jpg "Grayscaling"
[image3]: ./examples/random_noise.jpg "Random Noise"
[image4]: ./examples/placeholder.png "Traffic Sign 1"
[image5]: ./examples/placeholder.png "Traffic Sign 2"
[image6]: ./examples/placeholder.png "Traffic Sign 3"
[image7]: ./examples/placeholder.png "Traffic Sign 4"
[image8]: ./examples/placeholder.png "Traffic Sign 5"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the pandas library to calculate summary statistics of the traffic
signs data set:

Dataset:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is 32x32x3
* The number of unique classes/labels in the data set is 43

#### 2. Include an exploratory visualization of the dataset.

I used numpy library shape and unique functions to get statistics. 
"barchart.png" in the files displays training data visualization.

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As a first step, I decided to convert the images to grayscale to get 1 channel instead of 3. Then I normalized the dataset to have zero mean. Then ı trained and achieved 93.7% accuracy wwith test data.



![alt text][image3]

The difference between the original data set and the augmented data set is the following ... 


#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:

| Layer         		|     Description	        					|
|:---------------------:|:---------------------------------------------:|
| Input         		| 32x32x1 gray image  							|
| Convolution 5x5     	| 1x1 stride, valid padding, outputs 28x28x6 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride, valid padding, outputs 14x14x6 	|
| Convolution 5x5	    | 1x1 stride, valid padding, outputs 10x10x16   |
| RELU					|												|
| Max pooling	      	| 2x2 stride,  valid padding, outputs 5x5x16 	|
| Flatten	      	    | 											 	|
| Fully connected		| input 400, output 120        					|
| RELU					|												|
| Dropout				| 50% keep        								|
| Fully connected		| input 120, output 84        					|
| RELU					|												|
| Dropout				| 50% keep        								|
| Fully connected		| input 84, output 43        					|
 


#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

To train the model, I used an Adam optimizer to minimize loss of cross entropy. ı used following hyperparameters.

epochs = 50
batch_size = 256
dropout = 0.5
learning_rate = 0.001
mu = 0
sigma = 0.1

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy of 0.996
* validation set accuracy of 0.937 
* test set accuracy of 0.931

If an iterative approach was chosen:
* What was the first architecture that was tried and why was it chosen?
LeNet was my first chosen architecture.

* What were some problems with the initial architecture?
No problem with LeNet.

* How was the architecture adjusted and why was it adjusted? Typical adjustments could include choosing a different model architecture, adding or taking away layers (pooling, dropout, convolution, etc), using an activation function or changing the activation function. One common justification for adjusting an architecture would be due to overfitting or underfitting. A high accuracy on the training set but low accuracy on the validation set indicates over fitting; a low accuracy on both sets indicates under fitting.

* Which parameters were tuned? How were they adjusted and why?
I tuned number of epochs and learning rate .

* What are some of the important design choices and why were they chosen? For example, why might a convolution layer work well with this problem? How might a dropout layer help with creating a successful model?
I calculated layer matrix results. ı adhered to LeNet structure and i chose all padding, convolution, stride etc values to get exactly values that are in LeNet architecture.
50% dropout eleminated overfitting. Used more dropout could cause underfitting so I stayed at 50%.

If a well known architecture was chosen:
* What architecture was chosen?
LeNet

* Why did you believe it would be relevant to the traffic sign application?


* How does the final model's accuracy on the training, validation and test set provide evidence that the model is working well?
 The model works well. I got more than 93% accuracy which is expected.

### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

13_yield.jpeg
17_no_entry.jpeg
22_bumpy_road.jpg
25_road_work.jpeg
31_wild_animal_crossing.jpeg

The first image might be difficult to classify because there is some background colors in some of them. their size and formats are dfferent.

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

Image 0 prediction: 22 , the true label is 22 .
Image 1 prediction: 25 , the true label is 25 .
Image 2 prediction: 31 , the true label is 31 .
Image 3 prediction: 17 , the true label is 17 .
Image 4 prediction: 13 , the true label is 13 .


The model was able to correctly guess 5 of the 5 traffic signs, which gives an accuracy of 100%. 

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 11th cell of the Ipython notebook.

For the all the images, the model generates following top 5 predictions. All the first predictians is corrects for chosen images.

Image 0 probabilities: [  9.99759018e-01   2.31045269e-04   7.59536624e-06   1.24485859e-06
   1.02670526e-06] and predicted classes: [22 26 39 29 33]

Image 1 probabilities: [  9.75023985e-01   2.21333727e-02   1.36412412e-03   8.32468388e-04
   3.59818601e-04] and predicted classes: [25 20 35 13 36]

Image 2 probabilities: [  9.44168329e-01   5.57984710e-02   2.59735170e-05   3.49604375e-06
   3.45103581e-06] and predicted classes: [31 21 29 23 19]

Image 3 probabilities: [  9.99914527e-01   4.61458621e-05   3.05246467e-05   5.84514191e-06
   1.54600752e-06] and predicted classes: [17 33 13 12 10]

Image 4 probabilities: [  1.00000000e+00   2.32474799e-32   7.62282262e-38   1.88809862e-38
   0.00000000e+00] and predicted classes: [13 15  9 39  0]

### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
#### 1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?


