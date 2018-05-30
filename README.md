# Extended Kalman Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<a href="https://youtu.be/G_uOwfKlPio"><img src="https://i.imgflip.com/2b4yfg.gif" title="Extended Kalman Filter Giff"/></a> <br/>
[Click here for the full youtube video](https://youtu.be/G_uOwfKlPio)

## About This Project
This project uses an Extended Kalman Filter to track a car in 2D. The filter fuses both radar and sensor data in order to track the (x,y) position and (Vx,Vy) velocity.
The red dots are lidar and the blue dots with the directional arrows in them are radar. The green triangles are the predicted positions.

## Getting Started
To get this project up and running please follow these steps:
1. Clone this repo
2. Install the simulator which you can find [here](https://github.com/udacity/self-driving-car-sim/releases)
3. Make sure that you have the Dependencies listed below.
4. Once everything is installed make a build directory: `mkdir build && cd build`
5. Compile: `cmake .. && make`
6. Run it: ./ExtendedKF`
7. Open the simulator and select the first option "Project 1/2: EKF and UKF"
8. Select which dataset to use then click start. <br/>
*Note: Start from option 6 if you want run the simulation again.*

### Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.<br/>
INPUT: values provided by the simulator to the c++ program<br/>
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)<br/>
OUTPUT: values provided by the c++ program to the simulator<br/>
["estimate_x"] <= kalman filter estimated position x<br/>
["estimate_y"] <= kalman filter estimated position y<br/>
["rmse_x"]<br/>
["rmse_y"]<br/>
["rmse_vx"]<br/>
["rmse_vy"]<br/>

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
- [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) (jump to the operating system below uWebSockets)

### uWebSocketIO Installation Guide

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Note: Only uWebSocketIO branch e94b6e1, which the scripts reference, is compatible with the package installation.

#### Linux Installation:
From the project repository directory run the script: `install-ubuntu.sh`

#### Mac Installation:
From the project repository directory run the script: `install-mac.sh`

Some users report needing to use cmakepatch.txt which is automatically referenced and is also located in the project repository directory.

#### Windows Installation
Although it is possible to install uWebSocketIO to native Windows, the process is quite involved. Instead, you can use one of several Linux-like environments on Windows to install and run the package.

##### Bash on Windows
One of the newest features to Windows 10 users is an Ubuntu Bash environment that works great and is easy to setup and use. Here is a nice [step by step guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up the utility.

We recommend using the newest version of Ubunut Bash 16.04, which is able to run the `install-ubuntu.sh` script without complications. The link [here](https://www.howtogeek.com/278152/how-to-update-the-windows-bash-shell/) can help you check which version of Ubuntu Bash you are running, and also help you upgrade if you need to. Once you have installed bash nagivgate to the project repository directory and run the script: `install-ubuntu.sh`

##### Docker
If you don't want to use Bash on Windows, or you don't have Windows 10, then you can use a virtual machine to run a Docker image that already contains all the project dependencies.

First [install Docker Toolbox for Windows.](https://docs.docker.com/toolbox/toolbox_install_windows/)

Next, launch the Docker Quickstart Terminal. The default Linux virtual environment should load up. You can test that Docker is setup correctly by running `docker version` and `docker ps`.

You can enter a Docker image that has all the Term 2 project dependencies by running:

docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest

Once inside Docker you can clone over the GitHub project repositories and run the project from there.

**Port forwarding is required when running code on VM and simulator on host**<br/>
For security reasons, the VM does not automatically open port forwarding, so you need to manually [enable port 4567](https://www.howtogeek.com/122641/how-to-forward-ports-to-a-virtual-machine-and-use-it-as-a-server/). This is needed for the C++ program to successfully connect to the host simulator.

**Port Forwarding Instructions**
1. First open up Oracle VM VirtualBox
2. Click on the default session and select settings.
3. Click on Network, and then Advanced.
4. Click on Port Forwarding
5. Click on the green plus, adds new port forwarding rule.
6. Add a rule that assigns 4567 as both the host port and guest Port, as in the screenshot.

![alt text](port-forward-img.png)
**Setting up port forwarding for 4567 in a Virtual Machine**

---
### Troubleshooting
The tips below may be helpful in setting up your environment and getting term 2 projects up and running. **Windows 10 Users please note that Ubuntu BASH is the recommended option.**

#### Ubuntu BASH on Windows & Linux Trouble Shooting

**Not all steps will be necessary, for example, installing git and cloning the project repository, if this has already been done.**

1. `sudo apt-get update`
2. `sudo apt-get install git`
3. `sudo apt-get install cmake`
4. `sudo apt-get install openssl`
5. `sudo apt-get install libssl-dev`
6. `git clone https://github.com/udacity/CarND-Kidnapped-Vehicle-Project` or whatever CarND project
7. `sudo rm /usr/lib/libuWS.so`
8. navigate to the top level directory of the project
9. `./install-ubuntu.sh`
10. at the top level of the project repository `mkdir build && cd build`
11. from /build `cmake .. && make`
12. Launch the simulator from Windows and execute the run command for the project, for example `./ExtendedKF` or `./particle_filter` (Make sure you also run the simulator on the Windows host machine) If you see this message, it is working `Listening to port 4567 Connected!!!`

After following these steps there may be some messages regarding makefile not found or can't create symbolic link to websockets. There is likely nothing wrong with the installation. Before doing any other troubleshooting make sure that steps 10 and 11 have been executed from the top level of the project directory, then test the installation using step 12.

**Step 9 may fail for number of reasons as listed below:**
- `install-ubuntu.sh` has only rw but no x permission. Run `chmod a+x install-ubuntu.sh` to give execution permission
- Cannot find the package `libuv1-dev`
  - To install the package run `sudo apt-get install libuv1.dev`
  - If you still cannot install the package run the following to get the package and install it:
    - `sudo add-apt-repository ppa:acooks/libwebsockets6`
    - `sudo apt-get update`
    - `sudo apt-get install libuv1.dev`


- May complain about the version of cmake you have. You need a version greater than 3.0. [Here is a link](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line) which describes how to get version 3.8. Look at Teocci's response in this link
- Installing cmake requires g++ compiler. Install a g++ version 4.9 or greater. Here are the steps:
  - `sudo add-apt-repository ppa:ubuntu-toolchain-r/test`
  - `sudo apt-get update`
  - `sudo apt-get install g++-4.9`
**A Note Regarding Step 11** This step can fail if the bash shell is still referring to an older g++ version. To address this, please refer to [this Ask Ubuntu post](https://askubuntu.com/questions/428198/getting-installing-gcc-g-4-9-on-ubuntu/507068#507068).

#### Docker on Windows
A common pitfall is to not log in to the docker container. The first time you run `docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest` the controls_kit may download, but the system may not log you in to the container. If you are logged in instead of a `$` prompt, you should see something like this: `root@27b126542a51:/work#`. If you are not logged into the container commands such as `apt-get` and `make` will not be recognized, so be sure to execute `docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest` again, if you do not see the correct prompt.


**Tip regarding port forwarding when running code on vm and simulator on host**<br/>
When using a virtual machine and running the simulator on the host machine, it is critical to set up port forwarding, as described above.

**Note Regarding Ubuntu Bash on Windows**<br/>
The Ubuntu Bash system can be accessed from Windows, any files altered in this way may no longer be recognizable by Ubuntu BASH. This often manifest itself in the file disappearing from Ubuntu BASH.

##### IDE Profile to Develop Natively in Windows with Visual Studio
A student contributed IDE profile can be found [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio).

#### Mac
For most instances of missing packages and messages regarding uWebsockets, refer to Linux and Windows troubleshooting. Below are some common issues and their solutions.
  - **.sh files not recognized on run**: Try chmod a+x for example `chmod a+x install-mac.sh`
  - **missing** `openssl`, `libuv`, or `cmake`: install-mac.sh contains the line `brew install openssl libuv cmake`, which will not execute properly if `homebrew` is not installed. To determine if `homebrew` is installed, execute `which brew` in a terminal. If a path returns it is installed, otherwise you see `brew not found`. Follow the guidance [here](https://brew.sh/) to install homebrew, then try running `install-mac.sh` again.
  - If the step above does not resolve issues regarding openssl, please try the guidance provided [here](https://github.com/udacity/CarND-PID-Control-Project/issues/2)
  - **Issues with rootless mode in recent versions of OSx**: Some recent versions of OSx have a rootless mode by default that cause some install script commands to fail, even when running as root or sudo. To disable this reboot in recovery mode (`command+R`), and execute `csrutil disable` in a terminal. After this is complete, try running the install script.
  
After following these steps there may be some messages regarding makefile not found or can't create symbolic link to websockets. There is likely nothing wrong with the installation. Before doing any other troubleshooting make sure that build steps (10 and 11 from Windows and Linux instructions) have been executed from the top level of the project directory, then test the installation using running the code (step 12 from Windows and Linux instructions).




### Known Issue: Rapid Expansion of Log Files
Some people have reported rapid expansion of log files when using the simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.
+ create an empty log file
+ remove write permissions so that the simulator can't write to log

---
## Related information on EKF's
- [Tutioral](http://home.wlu.edu/~levys/kalman_tutorial/)
- [Slide show](http://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated3/kleeman_kalman_basics.pdf)

## License
[MIT License](https://opensource.org/licenses/MIT)<br/>
