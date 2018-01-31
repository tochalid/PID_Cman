# PID_Cman
# Submission for Term2 Project 4: PID Controler

Objective: Implementing a PID controler algorithm in C++ to solve a steering angle problem from a simulator of a car driving on a predefined circle track (reference trajectory).

> Environment setup, algorithm docu and boilerplate code provided by Udacity 
under https://github.com/udacity/CarND-PID-Control-Project. 
Improvements where made to:

- src/main.cpp
- src/PID.cpp

In main.cpp the PID controler is initialized with a set of 3 params (Kp, Ki, Kd) calling the __Init__ function of the pid instance. The parameters scale the effects of proportional, integral and differential Cross Track Error (__CTE__) to determine the new steering value. That equals the total error, which in case of violating the vehcile spec is limited to the min/max steering ability (-1 to 1) of the vehicle. PID.cpp implements the __Init, UpdateError__, and __TotalError()__ functions.

> "Algrithm Flowchart"

![Image1](./flowchart.png)

Besides the algorithm desribed in "Pseudocode" this requires implementing sampling from Gaussion distribution, Nearest Neighbor, Root Mean Squared Error, Homogenous Transformation of coordinates and Multivariate-Gaussian probability density.
> "Pseudodcode"

![Image2](./pseudocode.png)

Passing the project requires obtaining RMSE values that are lower than the tolerance tested by the simulator, pls see project rubric https://review.udacity.com/#!/rubrics/824/view.

This project involves the Term 2 Simulator which can be downloaded here: https://github.com/udacity/self-driving-car-sim/releases. A server package uWebSocketIO is setting up a connection from the C++ program to the simulator, which acts as the host.

## Running the project
> Basic Build Instructions: run shell-commands from the project directory
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
5. Run and start the simulator (resolution: 600x800, Mode: fastest)

## Results
> Vehicle stays on track for multiple rounds with top speed of 60mph. 

For this speed, in certain situations the car is oszillating, but staying in the limits of the road. Supposedly, for a autonomous passenger car this would not be acceptable, but could eg. improved by 

- drive slower, eg 30mph
- implementing a throttle PID controler that reduces speed for curves and accelerates accordingly
