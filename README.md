# AIRo_trajectory

A trajectory publish server for quadrotor applications. This package is dependent on [AIRo_control_interface](https://github.com/HKPolyU-UAV/airo_control_interface).

## Installation

It is recommanded to run the package in our docker following instructions [here](https://github.com/HKPolyU-UAV/docker_practice). By doing so, you can skip the following steps.

Create a catkin workspace and clone this repository to src folder (ex. ~/airo_trajectory_ws/src)
```
mkdir -p ~/airo_trajectory_ws/src
cd ~/airo_trajectory_ws/src
git clone https://github.com/HKPolyU-UAV/airo_trajectory.git
```

Source airo_control_interface setup file
```
source ~/airo_control_interface_ws/devel/setup.bash
```

Build the package
```
cd ~/airo_trajectory_ws
catkin_make
source ~/airo_trajectory_ws/setup.bash
```

## Usage

Launch airo_control_interface first, and run launch file in airo_trajectory package.

The ```example_mission.launch``` publishes several setpoints with or without yaw commands
```
roslaunch airo_trajectory example_mission.launch
```

The ```file_trajectory.launch``` will run a python script to generate ```.txt``` file that contains reference trajectory and publish it.
```
roslaunch airo_trajectory file_trajectory.launch
```

The ```position_setpoint.launch``` is similar to ```file_trajectory.launch``` but will directly publish trajectory to rostopic ```/mavros/setpoint_position/local``` so that the trajectory is tracked with the embedded flight controller.
```
roslaunch airo_trajectory position_setpoint.launch
```

Note that the user can publish either position, position&twist, or position&twist&acceleration commands. Besides, if no yaw angle is published, the server will publish zero yaw angle ```psi = 0```. In the .txt file, make sure that the first three columns are position reference, and you can add 3 more columns of twist reference, or 6 more columns of twist&acceleration reference. You can choose whether to add yaw reference or not, if yes, always add the reference yaw angle in rad at the last column. Therefore, the .txt file can have either 3 column (position), 6 column (position,twist), 9 column (position,twist,acceleration), 4 column (position,yaw), 7 column (position,twist,yaw), and 10 column (position,twist,acceleration,yaw).

For ```file_trajectory.launch```, if parameter ```result_save``` is set to true, the trajectory tracking results will be saved in the ```results``` folder. To post-process the results, check this [repository](https://github.com/HKPolyU-UAV/matlab_plot_master.git).