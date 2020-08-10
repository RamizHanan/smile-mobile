# smile-mobile
Welcome! Smile-mobile is an small autonomous ground vehicle seeking to develop its capabilities to be an intelligent autonomous system capable of navigating various environments, from nice paved rounds to bumpy, rocky terrian. The Smile-mobile software platform is built on the robotic operating system (ROS).

### Project Structure
The ROS environment for smile-mobile is broken up into two seperate workspaces (catkin workspaces). The first workspace is the robot workspace which is contained in the directory **smile_mobile_robot_ws**. The robot workspace contains all of the software that runs on the robot (both physical and simulated); the software in this package should be agnostic of whether the robot is the physical or simulated robot. 

The second workspace is the simulation workspace which is contained in the **smile_mobile_sim_ws** directory. The simulation workspace contains all the contents required for building the simulated world in gazebo and glueing together the robots control software with the simulated robot model.

Each of the workspaces contain ROS packages for various functions.

### Building the Project
First navigate into the **smile_mobile_robot_ws** directory. Run the following
```cmd
catkin_init_workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
Now navigate into the **smile_mobile_sim_ws** directory and run the following. **NOTE**: Notice the sourcing order 
```cmd
catkin_init_workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

Now, you should added the sourcing lines to the **.bashrc** so that the correct ROS_PACKAGE_PATH is set each time a new terminal is opened. Open the files with **vim ~/.bashrc**. At the end of the file, add

```cmd
source /opt/ros/melodic/setup.bash
source ~/smile-mobile/smile_mobile_robot_ws/devel/setup.bash
source ~/smile-mobile/smile_mobile_sim_ws/devel/setup.bash
```

### Adding the Gazebo World Models to Environment Variables
In order to launch the world correctly, the model paths for the smile mobile robot worlds need to be added to the environment variable path **GAZEBO_MODEL_PATH**. The best way to do this is to add the sourcing to the **.bashrc**. Note the directories shown below may differ for users.

```cmd
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_gazebo/models/
```

### Launching the Gazebo Simulation

Launching the Smile Mobile Robot in Gazebo simulation can all be done by running the following launch file within the package **smile_mobile_robot**. This simulation launchs the necessary simulation artifacts for gazebo alongside the robots movement controllers and raw odometry estimation.

Launch robot in empty world (will in future launch a custom world.)
```cmd
roslaunch smile_mobile_robot main_sim.launch
```

The robot can be controlled by interfacing with the movement controller that executes velocity and steering controls. The following helper node allows to a desired velocity and steering angle. The first parameter is the velocity (current range ~[-0.5, 0.5]m/s). The second parameter is the steering angle (range [-pi, pi]).

```cmd
rosrun smile_mobile_robot send_desired_movement --movement 0.4 1.579
```

The position estimated by the raw odometry node can viewed with the **Position_Plotter** Qt plugin.

```cmd
rosrun smile_mobile_gui Position_Plotter
```

### Physcial Vehicle
Most of the same software that was used to run the simulated vehicle will also be used for the physical vehicle. The physcial vehicle does not require any of the Gazebo related software. However, the physical vehicle requires running various hardware drivers to communicate with the onboard sensors and actuators.

### Flashing the Hardware Code

If necessary, flash the encoder_imu_driver and motor_driver Arduino software to their respective boards on the physical vehicle. These arduino drivers are located within the package **smile_mobile_hardward**.

### Launching the code on the physcial vehicle.
To launch the code on the physical vehicle, after all the batteries are plugged in and the hardware is powered up, SSH into the onboard computer and run the launch scripts. Note, it is important to have static IP. Once SSHed in, run one of the following

```cmd
//Use movement controller with PID's to hold velocity and steering angle
roslaunch smile_mobile_robot main_physical.launch 

OR

//Simply except PWM (typically from web gui) to manually control the vehicle
roslaunch smile_mobile_robot keyboard_teleop_physical.launch
```

### Launching the Web GUI
On the host computer (that you are using to SSH into the vehicle), you can interface with the vehicle via the Web GUI built using the Vue framework with ROSLIBjs.
Within the **smile_mobile_web** directory run.

```
//The comman web package manager is used to start a server to run the web app.
npm serve run
```

To via the Web GUI in the browser, copy the localhost url shown in the output of running the previous command.


Note: If any issues are discovered, please set an issue inquiry in the github **issues** tab for this repo. Thanks!
