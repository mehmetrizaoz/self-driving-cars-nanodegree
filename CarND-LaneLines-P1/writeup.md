# **Finding Lane Lines on the Road** 

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.
Pipeline Steps:
1- RGB Image converted to gray scale.
2- Gray scale image blurred for better edge detection (with 5x5 kernel, gaussian) 
3- Blurred image inserted canny edge detection (with low_threshold=50 and high_threshold=100 values)
4- Defined region of interest and ignored pixels out of this area.
5- Hough transform (with rho=1, theta=pi/180, threshold=10, min_line_len=20, max_line_gap=20 values) implemented 

draw_lines funtion:
Slope is used to distinguish left and right lines.
   if slope is negative the line is on the left side. 
   Otherwise on the right side.
Parallel lines are ignored.

Formula of the left and right lines are generated using curve fitting.
OpenCV polyfit function generated average slope and intersect parameters for both lines. 
Lines are drawn using top and bottom coordinates, slope/intersect found above and line formula (y = mx + b)

### 2. Identify potential shortcomings with your current pipeline
Pipeline does not generate accurate results with curved lines. 
Result may be better after changing;
   parameters for canny edge detection and hough transform
   and field defined for Region of interest is defined.
Result has some wrong lines in video streams. 

### 3. Suggest possible improvements to your pipeline
Detecting curvature will be definitely an improvement for the current pipeline. 
Therfore method for drawing lines needs modifications to draw polynomial lines.

