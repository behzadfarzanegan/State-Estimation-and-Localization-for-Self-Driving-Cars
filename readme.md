
## Vehicle Localization Using Extended Kalman Filter

This repository contains an implementation of vehicle localization along a trajectory using an Extended Kalman Filter (EKF). The goal is to estimate the vehicle's position in real time, utilizing sensor measurements and a motion model.

### Overview
In this project, we use a simple LIDAR sensor that provides range and bearing measurements to known landmarks in the environment. The EKF combines these measurements with odometry data to estimate the vehicle's 2D pose (position and orientation).

### Models Used

- **Motion Model**: The motion model receives odometry data (linear and angular velocity) as input and calculates the updated vehicle pose, considering process noise with a zero-mean Gaussian distribution.

- **Measurement Model**: The measurement model relates the current vehicle pose to LIDAR measurements, accounting for the known positions of landmarks and measurement noise, also modeled as a zero-mean Gaussian distribution.

### Implementation Steps
The repository includes implementations of the key steps required for EKF:

1. **Prediction Step**: Uses odometry measurements to predict the next state and uncertainty.
2. **Correction Step**: Uses LIDAR range and bearing measurements to update the predicted state and refine the pose estimate.

### Getting Started
- First, unpack the provided dataset containing timestamps, odometry data, LIDAR measurements, and landmark information.
- Run the provided script to initialize the EKF and perform the estimation of the vehicle's position along its trajectory.

### Dependencies
- Python 3
- NumPy
- Matplotlib
- Pandas

### Usage
Clone the repository, install the required packages, and run the main script to see the results of the vehicle localization using EKF. The results include visualizations of the estimated trajectory and orientation compared to the ground truth.

### Acknowledgments
This project is a part of a localization assignment for a robotics class, with a focus on using nonlinear filtering techniques to estimate the state of a moving vehicle.
