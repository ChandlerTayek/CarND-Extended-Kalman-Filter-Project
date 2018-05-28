# Extended Kalman Filter Project
[*Self-Driving Car Engineer Nanodegree Program*](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)
<a href="https://imgflip.com/gif/2b4yfg"><img src="https://i.imgflip.com/2b4yfg.gif" title="made at imgflip.com"/></a>

## About This Project
This project is an exploration of using a kalman filter to track a car driving in a figure 8 using a fusion of both radar and sensor data.
The red dots are lidar and the blue dots with the directional arrows in them are radar.

## Getting Started
To get this project up and running please follow these steps:
1. Clonse this repo
2. Install the simulator which you can find [here](https://github.com/udacity/self-driving-car-sim/releases)
3. Make sure that you have the 3 Dependencies listed below.
4. Make a build directory: `mkdir build && cd build`
5. Compile: `cmake .. && make`
5. Run it: ./ExtendedKF`


## Dependencies
The minimum project dependency versions are:

- cmake: 3.5
  - All OSes: [click here for installation instructions](https://cmake.org/install/)
- make: 4.1 (Linux and Mac), 3.81 (Windows)
  - Linux: make is installed by default on most Linux distros
  - Mac: install [Xcode command line tools to get make] (https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++: 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line        tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)



## License
[MIT License](https://opensource.org/licenses/MIT)<br/>
Note: This project was from a Udacity course.




This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.


Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Known Issue: Rapid Expansion of Log Files
* Some people have reported rapid expansion of log files when using the simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.
    + create an empty log file
    + remove write permissions so that the simulator can't write to log

## Related information on EKF's
- [Tutioral](http://home.wlu.edu/~levys/kalman_tutorial/)
- [Slide show](http://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated3/kleeman_kalman_basics.pdf)

