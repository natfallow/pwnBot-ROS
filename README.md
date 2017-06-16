# pwnBot-ROS
The ROS code and launch files needed to run pwnBot for ECEN430 in 2017, included in the 
hardware directory are the two custom PCBS used for power management and the Arduino interface. 

pwnBot uses HECTOR SLAM and the navigation stack to navigate and create a map of an unknown environment. The joystick can be used to override the navigation stack to prevent crashes or move the robot manually.  

## Usage 
1. Install dependences
2. Setup ROS IPs
3. SHH into the NUC with username techies
4. For mapping and navigation on the NUC run: `rosluanch pwn_bot_navagation nav.launch`
5. For just locomotion run with the wired Xbox controller plugged into NUC : `roslaunch pwn_bot_hardware` standard_joystick_local_config.launch
6. Or if the joystick is run remotely: `roslaunch pwn_bot_hardware standard_config.launch`
7. On the local machine run `rosrun joy joy_node` and to view mapping/navigation output in another terminal run `rosrun rviz rviz` and add necessary topic views
8. To navigate using the robot either set a navgoal in rviz or use the joystick  

## Joystick 
The joystick can be used to override the navigation stack to prevent crashes or move the robot manually. The code will only work with the wired Xbox controller. The Xbox controller can either be plugged directly into the NUC or run via an external machine.  

### Joystick mapping
* <b>RB</b> is the joystick enable, press this down to enable the joystick control or override the navigation stack
* The left joystick provides acaman style control. 
* The right joystick allows holonomic style control
* By pressing RIGHT, X, RIGHT, LEFT, RIGHT, R1, RIGHT, LEFT, X, TRIANGLE the robot becomes invincible and will never fail. 

Both joysticks can be used simultaneously to allow movement in all 3 axis of freedom. 

## Hardware 
There is two PCBS on board pwnBot one providing power dissolution and one providing control for the motor drivers. Altium files and schematics for these PCBS can be found in the `Hardware` file of this repo. 

The power distribution PCB can take 10 to 30V these limits are set by the regulator and the motor drivers. The power supply must be able to supply a steady current of 4.5 amps in order to run all components. For testing a 3 cell (11.1V) 8000mAh LiPo was used. 

There are 3 LEDs on the power PCB that indicate the presence on supply rails.
* RED: 5V
* Green: Motor Driver Power
* Blue: 12V

The motor drivers are connected though an emergency stop relay. This can be actuated using the toggle switch on the side of the robot. For this relay to function there must be regulated 5V on the 5V rails on this PCB. The NUC and the LiDAR are powered off the 12V rail from the regulator. 

The Arduino PCB provides PWM and PTG(Pull to Ground) outputs for the motor drivers. The PWM control provides speed control while the PTG outputs provide direction and breaking control. The PTG controls are completed with external MOSFETs to protect the Arduino from voltage transients.  

The Arduino shield also provides a header to link to the power PCB for voltage monitoring and button control with the other external button. A header is also included so the mouse sensor can be connected to the SPI and interrupt pins. 

### Hardware list
* MAXON motors [230572](http://www.maxonmotor.com/maxon/view/product/230572) motor drivers
* MAXON motors [438107](http://www.maxonmotor.com/maxon/view/service_search;JSESSIONID=DF403172332B135C67D37C4328F00898.node1?query=438107) motor and gearbox combo
* SICK [LMS100](https://www.sick.com/us/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms100-10000/p/p109841) LiDAR
* Intel NUC 
* 3 cell [11.1V 8000 mAh](https://hobbyking.com/en_us/zippy-flightmax-8000mah-3s1p-30c-lipo-pack.html?___store=en_us) LiPo Battery
* [ADNS-9800](https://www.tindie.com/products/jkicklighter/adns-9800-laser-motion-sensor/) mouse sensor for odometry (not used)
* [Arduino Mega](https://www.arduino.cc/en/Main/ArduinoBoardMega2560)
* [M4-ATX](http://www.mini-box.com/M4-ATX?sc=8&category=981) Automotive DC-DC Power Supply

## Dependices
1. All the dependices should be on the NUC allready, if not copy this repo into the src folder of your catkin_ws

...If adding files to new NUC you will need to run `cd ~/catkin_ws` and `catkin_make` then `source devel/setup.bash` 
2. If the arduino does not have the correct code on it, you will have to upload it (Code is in `arduino` folder)

...You make need to make the libraies for the upload to work, find out how [HERE](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
3. On another Ubuntu 16 machine you will need to [install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) desktop-full
4. Install [joy](http://wiki.ros.org/joy) on the local machine with `sudo apt-get install ros-kinetic-joy`

## ROS IP setup
To run ROS with an external machine you need to setup the `ROS_MASTER_URI` and `ROS_IP` variables. If using the pwnBot network the NUC will have an IP of `192.168.1.100`. If using another network you will have to use `ifconfig` to find the IP of the robot.   

1. On the NUC `nano ~/.bashrc` and scroll to the bottom of the file
2. Add/change the line `export ROS_MASTER_URI=http://*NUC IP Address*:11311`
3. Add/change the line `export ROS_IP=*NUC IP Address` and save/close the file
4. On the local machine `nano ~/.bashrc` and scroll to the bottom of the file
5. Add/change the line `export ROS_MASTER_URI=http://*NUC IP Address*:11311`
6. Add/change the line `export ROS_IP=*Local IP Address*` and save/close the file
7. Restart both terminal windows for the changes to take effect


## Known Bugs
* Either ROS serial or the Arduino are crashing, this means to robot will not respond to new velocity commands - sometimes causing it to crash. 
* The navigation stack needs further tuning as it still fails to complete simple goals - the parameters in `pwn_bot_navagation/base_local_params.yaml` will need to be changed
* The navigation stack needs a modified version of hector slam (found in this repo) - discussion [HERE](http://answers.ros.org/question/40541/extrapolation-error-of-local-cost-map-in-navigation/)
* The battery voltage monitoring is on the wrong side of the relay
* The connector pinout between the Arduino PCB and the power PCB is not the same.  

## Work to be done
* Integrate the odom into ROS
* Tune navigation stack
* Add destructor to stop robot when hwd_init_joy_override is terminated
* Add battery voltage monitoring to maintain safe battery limits
* Notify ROS when motor emergency stop is activated.
