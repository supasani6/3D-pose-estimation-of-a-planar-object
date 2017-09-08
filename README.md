3D Pose estimation of a planar object using OpenCV
===================================================

This uses the Perspective and Point algorithm with RANSAC to estimate the 3D pose of the planar object (In this case a chessboard). As a result the 3D world co-ordinates of any of the chessboard corners are displayed.

Camera Calibration is necessary to get the intrisic and extrinsic parametres of the camera
Camera Calibration
--------------------
For calibrating the camera, run the code calibration.py. This creates and saves the camera parameters in a file 'BBB.npz'

For estimating the pose, run the program PoseEstimation.py

Reference 
-----------------

Camera Calibration and 3D Reconstruction - OpenCV-Python Tutorials
(Modification to original code: Calculation of Camera Pose and 3D pose)
