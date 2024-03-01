# osr_ws

This repository contains the code that runs on the Raspberry Pi to control the rover.

The rover runs on ROS2 (tested on Foxy), and uses Python3.

For more detail information, refer to the following link.  
(https://github.com/nasa-jpl/osr-rover-code/tree/master)

# 1. Install ROS2 Foxy on Raspberry Pi
(https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

# 2. Open Source Rover code build
## Setup ROS build environment
First we'll create a ROS workspace for the rover code.
```
# Create a colcon workspace directory, which will contain all ROS compilation and 
# source code files, and navigate into it
mkdir -p ~/osr_ws/src && cd ~/osr_ws

# Source your newly created ROS environment. If you get "No such file or directory", either you have not installed ROS2 properly, or the environment variables aren't set correctly. Ask for help on Slack on the troubleshooting channel.
source /opt/ros/${ROS_DISTRO}/setup.bash
```
## Clone and build the rover code
For this section, you'll be working with the version control software git. Now's a good time to read up on how that works if you're new to it and make a GitHub account! In the newly created colcon workspace you just made, clone (download) this repo:
```
sudo apt install git
cd ~/osr_ws/src
git clone https://github.com/nasa-jpl/osr-rover-code.git
```
Now we will install the dependencies using rosdep
```
sudo apt install python3-rosdep
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
pip3 install adafruit-circuitpython-servokit
# build the ROS packages
colcon build --symlink-install
```
Now let's add the generated files to the path so ROS can find them
```
source install/setup.bash
```
The rover has some customizable settings that will overwrite the default values. Whether you have any changes compared to the defaults or not, you have to manually create these files:
```
cd ~/osr_ws/src/osr-rover-code/ROS/osr_bringup/config
touch osr_params_mod.yaml roboclaw_params_mod.yaml
```
## Add ROS config scripts to .bashrc
The source ....bash lines you typed out earlier are used to manually configure your ROS environment. We can do this automatically in the future by doing:
```
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc 
echo "source ~/osr_ws/install/setup.bash" >> ~/.bashrc
```
## Disable serial-getty@ttyS0.service
```
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
```
## Copy udev rules
```
# copy udev file from the repo to your system
cd ~/osr_ws/src/osr-rover-code/config
sudo cp serial_udev_ubuntu.rules /etc/udev/rules.d/10-local.rules
```
Reload the udev rules so that the rules go into effect and the devices files are set up correctly.
```
sudo udevadm control --reload-rules && sudo udevadm trigger
```
## Add user to tty and dialout groups
Finally, add the user to the tty and dialout groups:
```
sudo adduser $USER tty
sudo adduser $USER dialout
```
# Testing serial comm with the Roboclaw motors controllers
Run the roboclawtest.py script with all of the motor addresses:
'''
cd ~/osr_ws/src/osr-rover-code/scripts
python3 roboclawtest.py 128
python3 roboclawtest.py 129
python3 roboclawtest.py 130
'''
Each of these should output something like the following, within a very short execution time:
```
(1, 'USB Roboclaw 2x7a v4.1.34\n')
(1, 853, 130)
```




