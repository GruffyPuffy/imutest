# imutest
Small test program of integrating Intel Realsense D435i with PCL and using the IMU to rotate the cloud.

This is a small sample I made for the D435i. I found no other sample where the camera was integrated with PCL (to do point cloud maths) as well as using the IMU to rotate the cloud based on camera rotation. I have done a very simple filter for ACCEL + GYRO using the ACCEL to drift-compensate the GYRO. Note that I do not care about YAW in this sample (as the ACCEL cannot drift compensate that and I did not need it for my project).

The code uses the PCL visualizer to view the cloud as well as I added a second coordinate system for the camera rotation in order to visualize when you turn/rotate the camera. The idea with the sample is to rotate the cloud based on camera rotation in order to keep the "floor" (if indoor) aligned with the point cloud coordinate system...but see it as a test of the IMU-data.

If you want more background material on how I calculate the accelerometer angles please see this text:
https://www.instructables.com/id/Accelerometer-Gyro-Tutorial/

I am using a "complementary filter" instead of (for example) Kalman in this example. 
Try googling this and you will get lots of comparisons/explanations on this. 
One sample link to read could be this: http://www.pieter-jan.com/node/11

Feel free to use any of this code as you like. If you found it useful it would be great to know that from you.

All the best

Stefan Grufman
