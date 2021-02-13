# ROS package for UC1 simulation

> :warning: **set the GAZEBO_MODEL_PATH env variable to your local path**:  in **uc1_simulation/launch/uc1_station_simulation.launch**

## Launch file

To simulate the robot in gazebo run:

`roslaunch uc1_simulation uc1_station_simulation.launch`

## Launch dual arm cabling simulation

To simulate the dual arm cabling run: 

`roslaunch uc1_simulation dual_arm_operation_simulation_.launch`

or:

`roslaunch uc1_simulation dual_arm_operation_ur5e.launch`

If you want to load the simulation with the ur5e as left arm. 
In both cases be sure to set the argument 'cabling' to true, then add the database running: 

`roslaunch database_wires dual_arm_demo.launch`

(the version in the branch devel of wires_robotic_platform repository)

## Launch dual arm pin to pin simulation

To simulate the dual arm pin to pin run:

`roslaunch uc1_simulation dual_arm_p2p_ur5e.launch`

Be sure to set the argument 'pin_to_pin' to true, then add the database which is the same of the cabling simulation.

## TODO list

- [x] moveit! integration and testing 
- [ ] integration with CAD interface (T3.1)
- [ ] dual-arm planner development and validation
