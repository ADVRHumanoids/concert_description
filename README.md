# concert_description
ROS package containing modular's simulation scripts and launch files

## Docker image
A ready-to-use Docker container is provided, and it can be executed with `.docker/run-docker.bash`. Upon
first execution, a lot of data might be downloaded. The container can be used to follow the rest 
of this readme.

To **update the image** to the latest version
```bash
docker pull arturolaurenzi/concert_description
```

To **locally build the image** 
```bash
.docker/build-docker.bash [--no-cache] 
```

## Dependencies

 - ROS (desktop full is recommended, `moveit-core`)
 - XBot2 binaries (see [here](https://advrhumanoids.github.io/xbot2/quickstart.html#system-setup) for instructions)
 - The [modular](https://github.com/ADVRHumanoids/modular_hhcm) Python3 package (will be installed by [forest](https://github.com/ADVRHumanoids/forest))
 
## Setup
In addition to using Docker, you can setup concert_description using [forest](https://github.com/ADVRHumanoids/forest). 

1. **Install forest**:
```
[sudo] pip3 install hhcm-forest
```

2. Create a **forest workspace**. We are going to call it *concert_ws* for the sake of this example:
```
mkdir concert_ws && cd concert_ws
```

3. **Initialize** the forest workspace and **add recipes**: 
```
forest init
source setup.bash
echo "source $PWD/setup.bash" >> /home/USER/.bashrc
forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag master 
```
Where you should substitute USER with your username.

*Optional*: If you don't have any ssh key set up in your system run also:
```
export HHCM_FOREST_CLONE_DEFAULT_PROTO=https
```
and consider adding it to the .bashrc

4. Finally, just run:
```
forest grow concert_description
```
which will clone this repo and install the [modular](https://github.com/ADVRHumanoids/modular_hhcm) package. 

If you have the **XBot2 binaries** installed you are ready to simulate the CONCERT robot!

---

*P.S.* If you want to run also [this IK example](https://github.com/ADVRHumanoids/concert_description#move-the-base-with-ik) remember to also run:
```
forest grow centauro_cartesio -j 4
```


<!-- Drop into a catkin workspace's `src/` folder. If you're using `catkin_tools`, you might need to build the workspace. -->
 
## Quickstart (CONCERT example)

#### Launch the simulation environment, including the `xbot2` process
```
mon launch concert_gazebo concert.launch [rviz:=true]
```
![Screenshot from 2022-10-17 18-44-46](https://user-images.githubusercontent.com/22152172/196235597-9850b985-72cf-4bfd-a0e3-28dedcb12420.png)
![MicrosoftTeams-image (4)](https://user-images.githubusercontent.com/22152172/209342651-12d59ce9-7d62-43fe-8a55-970304862c75.png)

*Note*: For selecting to simulate **sensors** or not, the launch file accepts also a series of additional arguments. 
For example to run a simulation that will load also the gazebo plugins for the **Realsense cameras**, the **Velodyne lidars** and the **ultrasound sensors** run:
```
mon launch concert_gazebo concert.launch realsense:=true velodyne:=true ultrasound:=true
```
You'll need to have the proper dependencies installed in your setup in order for sensor simulation to work. See the [forest recipe for this package](https://github.com/ADVRHumanoids/multidof_recipes/blob/master/recipes/concert_description.yaml).

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
First, make sure that the *ros_ctrl* module is enable, and that the robot arm is not in a singular configuration (e.g., run the homing module once). Then, invoke the following launch file
```
mon launch concert_cartesio concert.launch xbot:=true gui:=true
```
![Screenshot from 2022-10-17 18-51-15](https://user-images.githubusercontent.com/22152172/196236956-f50d8b5a-fea2-4c0a-865f-bfdf74d216f0.png)
Then, right-click on the interactive marker, and select *Continuous Ctrl*. Move the marker around, and see the resulting motion in Gazebo.

**Note** that this last part requires additional dependencies (see also `setup-docker.bash`), that can be installed via the *hhcm-forest* tool. Follow instructions from [here](https://github.com/ADVRHumanoids/multidof_recipes) and then invoke
```
forest grow centauro_cartesio
```

**Note** to control the base in velocity mode (i.e., via `geometry_msgs/TwistStamped` messages), you must first invoke the following ROS service:
```
rosservice call /cartesian/base_link/set_control_mode velocity
```
Upon succesful return, you can move the base by *continuously* sending velocity commands to the topic `/cartesian/base_link/velocity_reference`; note that the `msg.header.frame_id` field of the published messages can be usefully set to `base_link` in order to have the commanded twist interpreted w.r.t. the local frame.

## Deploy instructions

When launching the simulation environment (`mon launch concert_gazebo concert.launch`) a Python file is used to generate the robot model and write the URDF, SRDF, etc. By default this file is the `concert_example.py` script in `concert_examples`, although it can be changed by passing the path to another script to the `modular_description` argument of the launch file.

Executing the Python script, the required files will be generated in the `/tmp` folder and will be used by Gazebo, XBot2, etc. To save these files in a non-temporary folder the **deploy** argument can be passed to the Python script. For instance running:
```
roscd concert_examples
python3 concert_example.py --deploy ~/concert_ws/ros_src/ --robot-name my_concert_package
```
will deploy a **ROS package** called `my_concert_package` in the `~/concert_ws/ros_src` directory. This can now be used as an independent ROS package, that can be shared or stored as usual.

## Further documentation

The robot API: https://advrhumanoids.github.io/XBotInterface/

XBot2: https://advrhumanoids.github.io/xbot2/ , https://github.com/ADVRHumanoids/xbot2_examples

CartesIO: https://advrhumanoids.github.io/CartesianInterface/
