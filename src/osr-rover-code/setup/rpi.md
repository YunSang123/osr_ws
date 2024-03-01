# Setting up the Raspberry Pi (RPi)

In this section, we’ll go over setting up the Raspberry Pi (RPi) and setting up all the code that will run the rover. Our rover uses ROS (Robotic Operating System); we will set these up below.

These instructions should work for both the RPi 3 and 4. You are free to use other versions of RPi, ROS, or OS, but setting these up is not covered here and it is not guaranteed that those will work.

## Installing an Operating System

The first step is to install Raspberry Pi OS on your RPi. Other OS'es like Ubuntu will also work but might require deviation from the instructions. We recommend using the [Raspberry Pi Imager](https://www.raspberrypi.com/documentation/computers/getting-started.html#using-raspberry-pi-imager) to load the OS onto an SD card. If done right, you won't need an external monitor, mouse, or keyboard to set up your RPi!

Make sure you set the following settings while configuring the image to be flashed onto the SD card (ctrl/cmd+shift+x):

* Select `Enable SSH` to you can connect to the rover over your local WiFi connection rather than having to attach a screen, keyboard, and mouse to the RPi each time. We recommend using a password.
* `Set username and password`. You will need these later to log into the RPi over SSH.
* `Configure wifi` so the RPi knows how to connect to your network when it boots up. 
* `Set locale settings` to your time zone

When done flashing, insert the SD card into the Raspberry Pi. When the Raspberry Pi is powered (make sure to use a proper power supply or connect it to the rover), it should connect to the WiFi connection and be visible from your router's 'devices' page. If possible, we recommend assigning a static IP address to your RPi so you can reach it using that IP address. Otherwise you should also be able to reach it using its hostname (which you selected in the Imager process), potentially with `.local` appended to the end.

From another computer, try connecting to the Raspberry Pi using SSH. You should be able to do so from a terminal in Windows, Linux, or MacOS using the following syntax: `ssh user@machine`, where the user is the username you set during the SD card flashing process earlier. For example:

```
ssh osr_user@raspberrypi.local
ssh alfred@192.168.1.17
```

It should prompt for your password and then form a connection to a terminal window on the RPi!

## Installing ROS

We'll install ROS2 (Robot Operating System) on the RPi. If you're new to ROS, we recommend learning it as it is a crucial part in the code base.

You'll need to be logged in to the RPi via ssh, or open a terminal in the desktop GUI if you're connected via a monitor and mouse/keyboard.

Follow the [instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) for installing ROS2. 

**NOTE**: Depending on which version of Raspberry Pi OS you installed, you might need to install a [different version of ROS2](https://www.ros.org/reps/rep-2000.html). In what follows, we assume ROS2 `Iron`.

You can choose to either install the 'full version' (`sudo apt install ros-iron-desktop`) which comes with graphical packages like RViz and QT or install just the barebones version (`sudo apt install ros-iron-ros-base`). The latter allows you to install packages in the full version whenever you need them and so we recommend following this approach. You can install ROS2 and related graphical packages on a different computer on the same network and you will be able to receive all messages out of the box.

## Setting up ROS environment and building the rover code

### Setup ROS build environment

First we'll create a ROS workspace for the rover code. 

```
# Create a colcon workspace directory, which will contain all ROS compilation and 
# source code files, and navigate into it
mkdir -p ~/osr_ws/src && cd ~/osr_ws

# Source your newly created ROS environment. If you get "No such file or directory", either you have not installed ROS2 properly, or the environment variables aren't set correctly. Ask for help on Slack on the troubleshooting channel.
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### Clone and build the rover code

For this section, you'll be working with the version control software `git`. Now's a good time to [read up](https://try.github.io/) on how that works if you're new to it and make a GitHub account!
In the newly created colcon workspace you just made, clone (download) this repo:

```commandline
sudo apt install git
cd ~/osr_ws/src
git clone https://github.com/nasa-jpl/osr-rover-code.git
cd osr-rover-code
git fetch origin
git checkout v2-humble
```

Now we will install the dependencies using rosdep

```commandline
sudo apt install python3-rosdep
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=iron
pip3 install adafruit-circuitpython-servokit
# build the ROS packages
colcon build --symlink-install
```

It should run successfully. If it doesn't, please ask on Slack or [submit an issue](https://github.com/nasa-jpl/osr-rover-code/issues/new) if you believe there's a problem with the instructions.

Now let's add the generated files to the path so ROS can find them

```commandline
source install/setup.bash
```

The rover has some customizable settings that will overwrite the default values. 
Whether you have any changes compared to the defaults or not, you have to manually create these files:

```commandline
cd ~/osr_ws/src/osr-rover-code/ROS/osr_bringup/config
touch osr_params_mod.yaml roboclaw_params_mod.yaml
```

In the [rover bringup instructions](rover_bringup.md) we will edit these files to make any changes.

### Add ROS config scripts to .bashrc

The `source ....bash` lines you typed out earlier are used to manually configure your ROS environment. We can do this automatically in the future by doing:

```
cd ~
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc 
echo "source ~/osr_ws/install/setup.bash" >> ~/.bashrc
```

This adds the `source` lines to `~/.bashrc`, which runs whenever a new shell is opened on the RPi - by logging in via ssh, for example. So, from now on, when you log into the RPi your new command line environment will have the appropriate configuration for ROS and the rover code.

## Setting up serial communication on the RPi

The RPi talks to the motor controllers over serial bus.

### Enabling Serial and I2C

```
sudo raspi-config
```

Then use the menu to enable **I2C** and **Serial** under `Interfaces`. More on raspi-config [here](https://www.raspberrypi.com/documentation/computers/configuration.html).

### Disable serial-getty@ttyS0.service

*NOTE*: this section may no longer be necessary with newer versions of debian. We suggest skipping it and revisiting if serial does not work.

Because we are using the serial port for communicating with the roboclaw motor controllers, we have to disable the serial-getty@ttyS0.service service. This service has some level of control over serial devices that we use, so if we leave it on it we'll get weird errors ([source](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/)). Note that the masking step was suggested [here](https://stackoverflow.com/a/43633467/4292910). It seems to be necessary for some setups of the rpi4 - just using `systemctl disable` won't cut it for disabling the service.

**Note that the following will stop you from being able to communicate with the RPi over the serial, wired connection. However, it won't affect communication with the rpi with SSH over wifi.**

```
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
```

### Copy udev rules

Now we'll need to copy over a udev rules file, which is used to automatically configure needed linux device files in `/dev` namely, `ttyS0 and ttyAMA0`. These are used to connect to the Roboclaws using the RPi's GPIO pins. Here's a [good primer](http://reactivated.net/writing_udev_rules.html) on udev. 

```commandline
# copy udev file from the repo to your system
cd ~/osr_ws/src/osr-rover-code/config
sudo cp serial_udev_ubuntu.rules /etc/udev/rules.d/10-local.rules
```

and enter your password if requested.

Reload the udev rules so that the rules go into effect and the devices files are set up correctly.

```commandline
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This configuration will persist across RPi reboots.

### Add user to tty and dialout groups

Finally, add the user to the `tty` and `dialout` groups:

```
sudo adduser $USER tty
sudo adduser $USER dialout
```

You might have to create the dialout group if it doesn't already exist.

**note**: You'll need to log out of your ssh session and log back in for this to take effect. Or you can reboot with `sudo reboot`.

## Testing serial comm with the Roboclaw motors controllers

Run the [roboclawtest.py](../scripts/roboclawtest.py) script with all of the motor addresses:

```
cd ~/osr_ws/src/osr-rover-code/scripts
python3 roboclawtest.py 128
python3 roboclawtest.py 129
python3 roboclawtest.py 130
```

Each of these should output something like the following, within a very short execution time:

```
(1, 'USB Roboclaw 2x7a v4.1.34\n')
(1, 853, 130)
```

The version number may be later.

If the script seems to hang, or returns only zeros inside the parantheses (0,0), then you have a problem communicating with the given roboclaw for that address. Some troubleshooting steps in this cases:

- Make sure you followed the instructions in the [#Setting up serial communication] section above, and the serial devices are configured correctly on the RPi.
- Also make sure you went through the calibration instructions from the [main repo](https://github.com/nasa-jpl/open-source-rover/blob/master/Electrical/Calibration.pdf) and set the proper address, serial comm baud rate, and "Enable Multi-Unit Mode" option for every roboclaw controller (if multi-unit mode isn't enabled on every controller, there will be serial bus contention.). If you update anything on a controller, you'll need to fully power cycle it by turning the rover off.
- If you're still having trouble after the above steps, try unplugging every motor controller except for one, and debug exclusively with that one.
