# modular_description
ROS package containing modular's simulation scripts and launch files

## Docker image
A ready-to-use Docker container is provided, and it can be executed with `./run-docker.bash`. Upon
first execution, a lot of data might be downloaded. The container can be used to follow the rest 
of this readme.

## Dependencies

 - ROS (desktop full is recommended, `moveit-core`)
 - XBot2 binaries (see [here](https://advrhumanoids.github.io/xbot2/quickstart.html#system-setup) for instructions)
 - The Modular Python3 package (TBD - NOT DISTRIBUTED YET)
 
## Setup
Drop into a catkin workspace's `src/` folder. If you're using `catkin_tools`, you might need to build the workspace.
 
## Quickstart (CONCERT example)

#### Launch the simulation environment, including the `xbot2` process
```
mon launch modular_gazebo concert.launch
```
![Screenshot from 2022-10-17 18-44-46](https://user-images.githubusercontent.com/22152172/196235597-9850b985-72cf-4bfd-a0e3-28dedcb12420.png)


#### Launch XBot2's monitoring GUI
```
xbot2-gui
```

#### Run a homing motion (it is a default, simple real-time plugin)
```
rosservice call /xbotcore/homing/switch 1
```
or click *Start* on the GUI, next to the *homing* label.

![Screenshot from 2022-10-17 18-43-39](https://user-images.githubusercontent.com/22152172/196235414-8a4d1282-0122-416d-bf4a-04242abe7d32.png)

#### Enable robot control via ROS
```
rosservice call /xbotcore/ros_ctrl/switch 1
```
or click *Start* on the GUI, next to the *ros_ctrl* label. NOTE: you must not publish messages on the `/xbotcore/command` topic when starting this module!
Messages published on the `/xbotcore/command` topic are now forwarded to the simulator. This can be done (for debugging purposes) also via the GUI's sliders.

#### Move the base with IK 
First, enable the *ros_ctrl* module. Then, invoke the following launch file
```
mon launch modular_cartesio concert.launch xbot:=true gui:=true
```
![Screenshot from 2022-10-17 18-51-15](https://user-images.githubusercontent.com/22152172/196236956-f50d8b5a-fea2-4c0a-865f-bfdf74d216f0.png)
Then, right-click on the interactive marker, and select *Continuous Ctrl*. Move the marker around, and see the resulting motion in Gazebo.

**Note** that this last part requires additional dependencies (see also `setup-docker.bash`), that can be installed via the *hhcm-forest* tool. Follow instructions from [here](https://github.com/ADVRHumanoids/multidof_recipes) and then invoke
```
forest grow centauro_cartesio
```


