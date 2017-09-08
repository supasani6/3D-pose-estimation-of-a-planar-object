3D Pose estimation of a planar object using OpenCV
===================================================

This uses the Perspective and Point algorithm with RANSAC to estimate the 3D pose of the planar object (In this case a chessboard).
Camera Calibration is necessary to get the intrisic and extrinsic parametres of the camera

Camera Calibration
--------------------
For calibrating the camera, run the code calibration.py. This creates and saves the camera parameters in a file 'BBB.npz'

For estimating the pose, run the program PoseEstimation.py
