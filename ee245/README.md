# ee245
A repository for all material relevant to EE245 labs.

## Installation
- `sudo apt install git swig libpython-dev python-numpy python-yaml python-matplotlib gcc-arm-none-eabi libpcl-dev libusb-1.0-0-dev sdcc` install dependency
- `cd ~/catkin_ws/src` please switch to your own ROS workspace
- `git clone https://github.com/UCR-Robotics/ee245.git` (Or git clone your own "forked" repo)
- `cd ee245`  go into this folder
- `./build.sh`   build simulation firmware
- `cd ../..`  go back to ROS workspace
- `catkin_make`  compile ROS package

## Usage
### Running on a real robot
- `roslaunch ee245 robot.launch`
- `roscd ee245`
- `cd scripts`
- `./some_script.py` or `python some_script.py`

### Running on simulation
- `roscd ee245`
- `cd scripts`
- `./some_script.py --sim` or `python some_script.py --sim`

## Checklist
**To avoid unnecessary crash, please make sure you have checked everything listed below before running your script**
- Each robot should have its own unique and **asymmetric** marker configuration.
(Otherwise the mocap system cannot uniquely identify the robot.)
- When creating a new object in OptiTrack Motive software, make sure the object ID
is 1, 2, 3 corresponding to cf1, cf2, cf3, respectively.
(Otherwise your computer cannot receive the correct pose information.)
- Modify two parameters in your `robot.launch` file:
robot name (cf1, cf2, cf3) and corresponding uri channel (90, 100, 110).
- Modify two parameters in your `waypoint_navigation.py` script:
robot index (1, 2, 3) and correct initial position ([0, 1.5, 0], [0, 0, 0], or [0, -1.5, 0]).
- Put the robot onto the correct/corresponding takeoff point as you specified in the script.
(Otherwise state estimation will fail and robot will go crazy.)
- Make sure the robot is heading towards x axis (yaw = 0).
(Otherwise state estimation will fail and robot will go crazy.)
- **First** put the robot on the ground and **then** turn on the power.
(Otherwise it may have some issue with IMU calibration.)
- Finally check if the robot has enough battery voltage by the following command:
`rosrun crazyflie_tools battery --uri radio://0/100/2M/E7E7E7E701`
(use your correct radio address)

Some recommended additional checks:
- After `roslaunch ee245 robot.launch`, check if your computer can receive correct
pose information from mocap by `rostopic echo /optitrack/cf1/pose`
(this can avoid unknown network issues.)
- After `roslaunch ee245 robot.launch`, it's better to start your script soon.
This launch file will enable onboard state estimation.
Kalman Filter may diverge or have large noise if the robot stays stationary for a long time.
