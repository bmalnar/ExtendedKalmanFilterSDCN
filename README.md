# ExtendedKalmanFilterSDCN
Extended Kalman Filter (EKF) implementation for Udacity Self Driving Car Nanodegree

### Overview
This repository contains implementation of a Kalman filter for estimating the location of a moving vehicle with noisy lidar and radar measurements. The measurements are provided as a text file, and the performance of the Kalman filter can be visualized by using the simulator provided by Udacity. 

### Data file 
The data file is provided by Udacity and contains the lidar and radar measurements in a text format. The screenshot of the portion of the file can be seen in the following picture:

<img src="images/data_file_screenshot.png" width="700" alt="Data File Screenshot" />

As explained on the Udacity's original project github, each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L). For a row containing radar data, the columns are: **sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth**. For a row containing lidar data, the columns are: **sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth**.

The Kalman filter that is programmed in this project estimates the state vector of the moving vehicle, which has the format (px, py, vx, vy). For each data point, we store the ground truth and the Kalman filter estimation, so that we can calculate the root mean square error (RMSE) for the entire trajectory of the vehicle. The goal is to have RMSE less or equal to (.11, .11, 0.52, 0.52) for each of the values in the state vector (px, py, vx, vy).

### C++ source files
We have the following C++ source files in the _src_ directory of the repository:

- main.cpp - This file is used without modifications from the original Udacity repositry. The file reads in sensor data line by line and invokes the Kalman filter for estiamtions of the vehicle's position. It also stores the data for the ground truth, and makes sure that the RMSE values are displayed in the simulator in each step.  
- FusionEKF.cpp - This file contains the code that processes an individual measurements. For the first measurement, it initializes the Kalman filter with the matrices needed for computation (covariance matrix, state transition, measurement function, noise). For the subsequent measurements (after the first measurement), it calls the predict and update steps of the Kalman filter and displays the current state vector and the uncertainty after that measurement is processed.
- kalman_filter.cpp - This file contains the functions of the Kalman filter. The function _Init()_ initializes the Kalman filter at the first measurement, the function _Predict()_ runs the prediction step, and the functions _Update()_ and _UpdateEKF()_ run the update step for lidar and radar measurements, respectively. 
- tools.cpp - This file contains helper functions for printing debug messages, for calculating the Jacobian matrix, the radar function, and the RMSE value in each step. 

### Output from the program
The output of the program in the text format is stored in the file _outputlog.txt_. It shows the values of the state vector and the uncertainty covariance in each step, and can be used for debugging. The following image shows the screenshot of the simulator at the end of the run. We observe the RMSE values to be **(0.0971, 0.0857, 0.4521, 0.4458)**, meaning that the goal of the project, as described above, is met. 

<img src="images/simulator_screenshot.png" width="700" alt="Data File Screenshot" />
 
### How to run the program
After cloning this repository, simply execute the following commands to build and run the program:
```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```
At the same time, the Udacity simulator needs to be running, so that the simulator and the EKF program connect and exchange information. 

### Setting up the environment 
- The project is configured to compile with cmake and make. Please make sure that the following dependencies are met:
   - cmake version 3.5
   - make version 4.1 for Linux and Mac and 3.81 for Windows
   - gcc/g++ version 5.4
- Download the Udacity simulator from [here](https://github.com/udacity/self-driving-car-sim/releases/)
- Additional libraries need to be installed by running:
   - On Ubuntu, install-ubuntu.sh 
   - On Mac, install-mac.sh
   - On Windows, the recommended way is to run a virual machine and use the install-ubuntu.sh script
   
### More information
For even more information on the project structure, dependencies etc. please check original Udacity project [repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

