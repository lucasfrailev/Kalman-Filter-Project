# Extended Kalman Filter in C++ for Object Tracking

## Project Overview

This repository contains a C++ implementation of the Extended Kalman Filter (EKF) for sensor fusion, designed to track the position and velocity of a vehicle moving around a stationary sensor suite using simulated LIDAR and RADAR measurements. The project is part of Udacity's Self-Driving Car Engineer Nanodegree Program and serves as a practical application of Kalman Filters in the field of autonomous driving. 

Why is the standard Kalman Filter, not enough in this case? The RADAR measurements consist of $\rho$, $\theta$, and $\dot{\rho}$ representing the distance, angle, and speed, respectively as measured from the origin. As these are polar coordinates, and the map between polar and cartesian coordinates is non-linear, we are forced to use an Extended Kalman Filter, resorting to Jacobians for the RADAR measurement update.

The simulation environment provided along with this project visualizes the tracking process, where:
- LIDAR measurements are shown as red circles.
- RADAR measurements are represented by blue circles with arrows pointing in the direction of the observed angle.
- The Kalman Filter's estimations are depicted as green triangles.

## Demo Videos

The videos demonstrate the filter's ability to accurately track the vehicles's movement, despite the inherent noise in LIDAR and RADAR measurements.

- **Video 1 - Zoomed In View:** 

https://github.com/lucasfrailev/Kalman-Filter-Project/assets/47170229/b32bc7c6-7f42-4e69-a22c-aacda4706a7c


- **Video 2 - Zoomed Out View:**

https://github.com/lucasfrailev/Kalman-Filter-Project/assets/47170229/94dda6af-acf7-4d52-abd7-069e051306af


## Getting Started

### Dependencies
- CMake >= 3.5
- Make >= 4.1 (Linux, Mac), 3.81 (Windows)
- GCC/G++ >= 5.4
- [uWebSocketIO](https://github.com/uNetworking/uWebSockets) for interfacing with the simulator.

### Simulator
The Term 2 Simulator, which contains this and other projects for the Self-Driving Car Nanodegree, can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

### Setup and Build Instructions
1. **Clone this repository** to your local machine.
2. **Navigate to the project directory** and create a build directory:
mkdir build && cd build
3. **Compile the project**:
cmake .. && make
4. **Run the Extended Kalman Filter**:
./ExtendedKF
5. **Launch the Simulator** and select the Extended Kalman Filter project to see the filter in action.

## Code Structure

The main components of the code include:
- `src/FusionEKF.cpp`: Initializes the filter, calls the predict function, updates the filter for LIDAR and RADAR measurements.
- `src/kalman_filter.cpp`: Defines the predict and update equations.
- `src/tools.cpp`: Contains functions to calculate root mean squared error (RMSE) and the Jacobian matrix.

## Contributing

Contributions to improve the project are welcome. Please follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for your contributions.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

