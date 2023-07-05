# airo_trajectory

Note that the user can publish either position, position&twist, or position&twist&acceleration commands. Besides, if no yaw angle is published, the controller will track yaw angle ```psi = 0```.

The airo_trajectory package can also publish commands predefined in .txt file. Several trajectories are defined in ```airo_trajectory/traj```, and the python scripts used to generate .txt files are also included. Note that sample time should be set to be the same as controller, which in 0.025s by default, since a new row in the file will be published every ros spin. In the .txt file, make sure that the first three columns are position reference, and you can add 3 more columns of twist reference, or 6 more columns of twist&acceleration reference. You can choose whether to add yaw reference, if yes, always add the reference yaw angle in rad at the last column. Therefore, the .txt file can have either 3 column (position), 6 column (position,twist), 9 column (position,twist,acceleration), 4 column (position,yaw), 7 column (position,twist,yaw), and 10 column (position,twist,acceleration,yaw).

Refer to example_mission_node.cpp for more details.
