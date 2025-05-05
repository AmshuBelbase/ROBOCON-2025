# Ball Tracking Project

The idea behind this project was to create a Python script that could recognize, track, and predict a thrown ball's trajectory. It was written in Python and relies heavily on the OpenCV library. Threshold color masking is used to differentiate the ball from the background; contours can then be applied to this mask, and the center point of the ball can be found.

Iterating through the video frame by frame, the difference in center point position can be used to calculate first the velocity of the ball, and then the acceleration of the ball. After three frames of object detection, the object's trajectory can be predicted and visualized.

It was found that the prediction algorithm was relatively accurate, and future iterations of the project could include object depth detection for non-planar motion.

The main file for ball tracking is Balltracker.py and addtional details and documentation can be found in the Report file

