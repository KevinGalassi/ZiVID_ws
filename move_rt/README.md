# MOVE-RT

[![pipeline status](https://dei-gitlab.dei.unibo.it/lar/move_rt/badges/master/pipeline.svg)](https://dei-gitlab.dei.unibo.it/lar/move_rt/-/commits/master)

## General Description

This package addresses the need for a standardized modular approach to the task priority control. It is particularly suitable for mobile manipulators operating in dynamically changing environments and in presence of human operators.
Task priority control enable the real time execution of several robotic tasks running in parallel and organized according to a specific hierarchy establishing the priority among tasks, i.e. lower priority tasks do not influence on the behavior of higher priority ones.
Common tasks for a robotic manipulator are singularity avoidance, joint limit avoidance, collision avoidance, joint speed limitation other than conventional end-effector position control.

Additional documentation can be found on the <a href="https://dei-gitlab.dei.unibo.it/lar/move_rt/-/wikis/home">Wiki</a> page.

## Testing

In order to test this library, launch the UR5 test simulation

```
roslaunch move_rt robot_simulation.launch
```

To change the configuration, edit the `config/tasks_arm_ur5.yaml` file. First controller in this file has the highest priority. After that all you have to create an instance of ControlManager class, and call update and control functions. An example of the use of this library is inside `src/move_rt.cpp`. This file created an executable that you can run once you are connected to the robot.

Any controller you want to add should be inherited from PriorityLevel class. You can see the implemented controller to follow the same strategy.

## License
move_rt is released with a Apache 2.0 license. For full terms and conditions, see the [LICENSE](LICENSE) file.


***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 
