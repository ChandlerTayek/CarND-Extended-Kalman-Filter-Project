# Extended Kalman Filter Project
[*Self-Driving Car Engineer Nanodegree Program*](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

<a href="https://imgflip.com/gif/2b4yfg"><img src="https://i.imgflip.com/2b4yfg.gif" title="made at imgflip.com"/></a> <br/>
[Click here for the full youtube video](www.youtube.com)

## About This Project
This project is an exploration of using a kalman filter to track a car driving in a figure 8 using a fusion of both radar and sensor data.
The red dots are lidar and the blue dots with the directional arrows in them are radar.

## Getting Started
To get this project up and running please follow these steps:
1. Clone this repo
2. Install the simulator which you can find [here](https://github.com/udacity/self-driving-car-sim/releases)
3. Make sure that you have the Dependencies listed below.
4. Once everything is installed make a build directory: `mkdir build && cd build`
5. Compile: `cmake .. && make`
6. Run it: ./ExtendedKF`
7. Open the simulator and select the first option "Project 1/2: EKF and UKF"
8. Select which dataset to use then click start to watch the car go. <br/>
*Note: Start from option 6 if you want to program again as the restart button doesn't clear out the memory..*


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
- uWebSockets (see below for installation instructions)


### uWebSockets
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.


Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


- #### Windows
The tips below may be helpful in setting up your environment and getting term 2 projects up and running. Windows 10 Users please note that Ubuntu BASH is the recommended option.
Ubuntu BASH on Windows
Steps
Ensure all dependencies are present per project resources
Follow these the instructions in the uWebSocketIO starter guide which includes instructions for setting up Ubuntu BASH.
open Ubuntu Bash and clone the project repository
on the command line execute ./install-ubuntu.sh
build and run according to the instructions in the project repository README
Trouble Shooting
.sh files not recognized on run: Try chmod a+x <filename.sh> for example chmod a+x install-ubuntu.sh
Messages regarding 404s, missing resources, and a variety of other error messages can be addressed by updates and other means, per this post and this post, and this post. The general steps are:
Not all steps will be necessary, for example, installing git and cloning the project repository, if this has already been done.
sudo apt-get update
sudo apt-get install git
sudo apt-get install cmake
sudo apt-get install openssl
sudo apt-get install libssl-dev
git clone https://github.com/udacity/CarND-Kidnapped-Vehicle-Project or whatever CarND project
sudo rm /usr/lib/libuWS.so
navigate to CarND-Kidnapped-Vehicle-Project/
./install-ubuntu.sh
at the top level of the project repository mkdir build && cd build
from /build cmake .. && make
Launch the simulator from Windows and execute the run command for the project, for example ./ExtendedKF or ./particle_filter (Make sure you also run the simulator on the Windows host machine) If you see this message, it is working Listening to port 4567 Connected!!!
After following these steps there may be some messages regarding makefile not found or can't create symbolic link to websockets. There is likely nothing wrong with the installation. Before doing any other troubleshooting make sure that steps 10 and 11 have been executed from the top level of the project directory, then test the installation using step 12.
Step 9 may fail for number of reasons as listed below:
install-ubuntu.sh has only rw but no x permission. Run chmod a+x install-ubuntu.sh to give execution permission
Cannot find the package libuv1-dev
To install the package run sudo apt-get install libuv1.dev
If you still cannot install the package run the following to get the package and install it:
sudo add-apt-repository ppa:acooks/libwebsockets6
sudo apt-get update
sudo apt-get install libuv1.dev
May complain about the version of cmake you have. You need a version greater than 3.0. Here is a link which describes how to get version 3.8. Look at Teocci's response in this link
Installing cmake requires g++ compiler. Install a g++ version 4.9 or greater. Here are the steps:
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install g++-4.9
A Note Regarding Step 11 This step can fail if the bash shell is still referring to an older g++ version. To address this, please refer to this Ask Ubuntu post.

Docker on Windows
The best place to start is to follow the instructions here. A common pitfall is to not log in to the docker container. The first time you run docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest the controls_kit may download, but the system may not log you in to the container. If you are logged in instead of a $ prompt, you should see something like this: root@27b126542a51:/work#. If you are not logged into the container commands such as apt-get and make will not be recognized, so be sure to execute docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest again, if you do not see the correct prompt.

A thoughtful student has created a docker specific starter guide for the EKF project. The following is an abridged version.

Tip regarding port forwarding when running code on vm and simulator on host
When using a virtual machine and running the simulator on the host machine, it is critical to set up port forwarding, as described here.

Transferring Files Between Native and Virtual Environments
Many prefer to use text editors in Windows rather than those that ship with Ubuntu BASH or Docker (vim, nano, etc.)

Options for addressing this include:

All Systems: setup a git repo, edit files in Windows, push to the repo from Windows, pull the repo from the virtual environment
Ubuntu BASH: edit files in windows, mount the c drive in Ubuntu BASH (cd /mnt /c), navigate to the files, copy to the desired location in Ubuntu BASH, navigated to the appropriate Ubuntu BASH folder
Docker on Windows: See this starter guide for suggestions.
Note Regarding Ubuntu Bash on Windows
The Ubuntu Bash system can be accessed from Windows, any files altered in this way may no longer be recognizable by Ubuntu BASH. This often manifest itself in the file disappearing from Ubuntu BASH.

IDE Profile to Develop Natively in Windows with Visual Studio
A student contributed IDE profile can be found here. More detail can be found here
-#### Linux
Steps
Ensure all dependencies are present per project resources
Clone the project repository
Follow these the instructions in the uWebSocketIO starter guide (From the project repository directory run the script: install-ubuntu.sh.
on the command line execute ./install-ubuntu.sh
build and run according to the instructions in the project repository README
Trouble Shooting
These steps are similar to those with Ubuntu BASH on Windows 10, above.

.sh files not recognized on run: Try chmod a+x <filename.sh> for example chmod a+x install-ubuntu.sh
Messages regarding 404s, missing resources, and a variety of other error messages can be addressed by updates and other means, per this post and this post, and this post. The general steps are:
Not all steps will be necessary, for example, installing git and cloning the project repository, if this has already been done.

sudo apt-get update
sudo apt-get install git
sudo apt-get install cmake
sudo apt-get install openssl
sudo apt-get install libssl-dev
git clone https://github.com/udacity/CarND-Kidnapped-Vehicle-Project or whatever CarND project
sudo rm /usr/lib/libuWS.so
navigate to CarND-Kidnapped-Vehicle-Project/
./install-ubuntu.sh
at the top level of the project repository mkdir build && cd build
from /build cmake .. && make
Launch the simulator from Windows and execute the run command for the project, for example ./ExtendedKF or ./particle_filter (Make sure you also run the simulator) If you see this message, it is working Listening to port 4567 Connected!!!
After following these steps there may be some messages regarding makefile not found or can't create symbolic link to websockets. There is likely nothing wrong with the installation. Before doing any other troubleshooting make sure that steps 10 and 11 have been executed from the top level of the project directory, then test the installation using step 12.

Step 9 may fail for number of reasons as listed below:

install-ubuntu.sh has only rw but no x permission. Run chmod a+x install-ubuntu.sh to give execution permission
Cannot find the package libuv1-dev
To install the package run sudo apt-get install libuv1.dev
If you still cannot install the package run the following to get the package and install it:
sudo add-apt-repository ppa:acooks/libwebsockets6
sudo apt-get update
sudo apt-get install libuv1.dev
May complain about the version of cmake you have. You need a version greater than 3.0. Here is a link which describes how to get version 3.8. Look at Teocci's response in this link
Installing cmake requires g++ compiler. Install a g++ version 4.9 or greater. Here are the steps:
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install g++-4.9
A Note Regarding Step 11 This step can fail if the bash shell is still referring to an older g++ version. To address this, please refer to this Ask Ubuntu post.


- #### Mac
For most instances of missing packages and messages regarding uWebsockets, refer to Linux and Windows troubleshooting. Below are some common issues and their solutions.
  - **.sh files not recognized on run**: Try chmod a+x for example `chmod a+x install-mac.sh`
  - **missing** `openssl`, `libuv`, or `cmake`: install-mac.sh contains the line `brew install openssl libuv cmake`, which will not execute properly if `homebrew` is not installed. To determine if `homebrew` is installed, execute `which brew` in a terminal. If a path returns it is installed, otherwise you see `brew not found`. Follow the guidance [here](https://brew.sh/) to install homebrew, then try running `install-mac.sh` again.
  - If the step above does not resolve issues regarding openssl, please try the guidance provided [here], here and (https://github.com/udacity/CarND-PID-Control-Project/issues/2) and here
  - **Issues with rootless mode in recent versions of OSx**: Some recent versions of OSx have a rootless mode by default that cause some install script commands to fail, even when running as root or sudo. To disable this reboot in recovery mode (command+R), and execute csrutil disable in a terminal. After this is complete, try running the install script.
After following these steps there may be some messages regarding makefile not found or can't create symbolic link to websockets. There is likely nothing wrong with the installation. Before doing any other troubleshooting make sure that build steps (10 and 11 from Windows and Linux instructions) have been executed from the top level of the project directory, then test the installation using running the code (step 12 from Windows and Linux instructions).

Tip regarding port forwarding when running code on vm and simulator on host
When using a virtual machine and running the simulator on the host machine, it is critical to set up port forwarding, as described here.
Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h



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

## License
[MIT License](https://opensource.org/licenses/MIT)<br/>
Note: This project was from a Udacity course.

## Known Issue: Rapid Expansion of Log Files
* Some people have reported rapid expansion of log files when using the simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.
    + create an empty log file
    + remove write permissions so that the simulator can't write to log

## Related information on EKF's
- [Tutioral](http://home.wlu.edu/~levys/kalman_tutorial/)
- [Slide show](http://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated3/kleeman_kalman_basics.pdf)

