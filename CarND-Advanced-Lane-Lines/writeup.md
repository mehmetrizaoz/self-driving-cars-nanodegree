**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.
"calibrate" function returns camera parameters and distortion matrixes. 
I calibrated camera using test images in camera_cal folder.

I used cv2.findChessboardCorners function to get corners.
I used cv2.calibrateCamera function to get calibration parameters.
Using two outputs of above function i undistorted any image using cv2.undistort function.

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

pipeline steps are
1- undistortion
2- blurring
3- getting binary thresholded image
4- getting region of interest
5- warping
6- fitting polynomial
7- unwarping polimomyal (back projection)
8- calculating curvature radius

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
i created "thresholding" function to get binary thresholded image.
i used;
  s and l channels of hls color space
  v channel of hsv color space
  absolute of sobelx 
for thresholding

to elaminate shadow effects saturation used with both hls and hsv used  

Then i ignored pixels out of ROI.

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

i warped pixels;
  [560,460],[180,690],[1130,690],[750,460] to
  [320,0],  [320,720],[960,720], [960,0]
  
i used cv2.getPerspectiveTransform function to get matrixes for both 
   from source to destination and 
   destintion to src.

Then i used cv2.warpPerspective function to get the perspective transformed image.

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

i used warped image as input in fit_polynomial function.
i found max two histogram in x axis at the bottom half first. 
Then i divided image to 9 in y axis and i found all line pixels using margin, histogram etc. 
Then i calculated 2d polynomial using np.polyfit function and lane pixels.

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.. 

i used given radius curvature formula to get radius and polyomial found step 5.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.
i projected the polynomial back to the image to unwarp it using perspective matrix (destination to soruces) found in step 3.

output file is pipeline.png. it is in "output_images" folder.

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

output video file is "lane_detection_video.mp"4 and in the "output_video" folder.

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?
Finding threshold values is really difficult. i modified it after my first sumbissions.

Even i chose saturtion for both hsv and hls color spaces to eleminate shadow effects, there are still small problems in shadowy images. Some more binary thresholding adjustment is required.

Pipeline will may fail if videos taken in very dark environments, at night etc.

Other cars on the way my close the lines. This will be problem to detect lane polynomial.

Lane lines are not always parallel after warping. There are sometimes small differences.



 


