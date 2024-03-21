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

### Quick EKF review

The Extended Kalman Filter (EKF) operates on a model and observation framework, where:

- The model is given by $x_k = f(x_{k-1}) + w_{k-1}$ and
- The observation is $z_k = h(x_k) + v_k$.

#### Initialization

- Initial state estimate: $x_{a0} = \mu_0$ with error covariance $P_0$.

#### Model Forecast Step/Predictor

1. The predicted state estimate $x_{fk} \approx f(x_{a{k-1}})$.
2. The predicted error covariance  $P_{fk} = J_f(x_{a{k-1}})P_{k-1}J_f^T(x_{a{k-1}}) + Q_{k-1}$..

#### Data Assimilation Step/Corrector

1. The updated state estimate $x_{ak} \approx x_{fk} + K_k(z_k - h(x_{fk}))$.
2. The Kalman Gain $K_k = P_{fk} J_h^T(x_{fk})(J_h(x_{fk})P_{fk} J_h^T(x_{fk}) + R_k)^{-1}$.
3. The updated error covariance $P_k = (I - K_k J_h(x_{fk})) P_{fk}$.

The EKF employs a linear approximation to handle nonlinear models and observations by utilizing Taylor Series expansions and Jacobians ($J_f$ and $J_h$ for the model and observation functions, respectively). This summary encapsulates the core mathematical framework and steps involved in executing the Extended Kalman Filter process.


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

```mkdir build && cd build```

3. **Compile the project**:

```cmake .. && make```

4. **Run the Extended Kalman Filter**:

```./ExtendedKF```

5. **Launch the Simulator** and select the Extended Kalman Filter project to see the filter in action.

## Code Structure

The main components of the code include:
- `src/FusionEKF.cpp`: Initializes the filter, calls the predict function, updates the filter for LIDAR and RADAR measurements.
- `src/kalman_filter.cpp`: Defines the predict and update equations.
- `src/tools.cpp`: Contains functions to calculate root mean squared error (RMSE) and the Jacobian matrix.


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

