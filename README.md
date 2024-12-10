
## :package: Package Overview

- [`diffbot_base`](./diffbot_base): ROS Control hardware interface including [`controller_manager`](http://wiki.ros.org/controller_manager) control loop for the real robot. The [`scripts` folder](./diffbot_base/scripts) of this package contains the low-level `base_controller` that is running on the Teensy microcontroller.
- [`diffbot_bringup`](./diffbot_bringup): Launch files to bring up the hardware drivers (camera, lidar, imu, ultrasonic, ...) for the real DiffBot robot.
- [`diffbot_control`](./diffbot_control): Configurations for the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller) of ROS Control used in Gazebo simulation and the real robot.
- [`diffbot_description`](./diffbot_description): URDF description of DiffBot including its sensors.
- [`diffbot_gazebo`](./diffbot_gazebo): Simulation specific launch and configuration files for DiffBot.
- [`diffbot_msgs`](./diffbot_msgs): Message definitions specific to DiffBot, for example the message for encoder data.
- [`diffbot_navigation`](./diffbot_navigation): Navigation based on [`move_base` package](http://wiki.ros.org/move_base); launch and configuration files.
- [`diffbot_slam`](./diffbot_slam): Simultaneous localization and mapping using different implementations (e.g., [gmapping](http://wiki.ros.org/gmapping)) to create a map of the environment

## Installation

The packages are written for and tested with [ROS 1 Noetic](http://wiki.ros.org/noetic) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/).
For the real robot [Ubuntu Mate 20.04](https://ubuntu-mate.org/download/arm64/focal/) for arm64 is installed on the [Raspberry Pi 4 B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) with 4GB. The communication between the mobile robot and the work pc is done by configuring the [ROS Network](http://wiki.ros.org/ROS/NetworkSetup), see also the [documentation](./docs/ros-network-setup.md).

### Dependencies

The required Ubuntu packages are listed in software package sections found in the [documentation](https://ros-mobile-robots.com/packages/packages-setup/#obtain-system-dependencies). Other ROS catkin packages such as [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros) need to be cloned into the catkin workspace.

For an automated and simplified dependency installation process install the [`vcstool`](https://github.com/dirk-thomas/vcstool), which is used in the next steps.

```console
sudo apt install python3-vcstool
```

### :hammer: How to Build

To build the packages in this repository including the Remo robot follow these steps:

1. `cd` into an existing ROS Noetic [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) or create a new one:
   ```console
   mkdir -p catkin_ws/src
   ```

2. Clone this repository in the `src` folder of your ROS Noetic catkin workspace:

   ```console
   cd catkin_ws/src
   ```

   ```console
   git clone https://github.com/fjp/diffbot.git
   ```
   
3. Execute the `vcs import` command from the root of the catkin workspace and pipe in the `diffbot_dev.repos` or `remo_robot.repos` YAML file, depending on where you execute the command, either the development PC or the SBC of Remo to clone the listed dependencies. Run the following command only on your development machine:

   ```
   vcs import < src/diffbot/diffbot_dev.repos
   ```

   Run the next command on Remo robot's SBC:

   ```
   vcs import < src/diffbot/remo_robot.repos
   ```
   
4. Install the requried binary dependencies of all packages in the catkin workspace using the following [`rosdep` command](http://wiki.ros.org/rosdep#Install_dependency_of_all_packages_in_the_workspace):

   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. After installing the required dependencies build the catkin workspace, either with [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make):

   ```console
   catkin_ws$ catkin_make
   ```
   or using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/):

   ```console
   catkin_ws$ catkin build
   ```
   
6. Finally, source the newly built packages with the `devel/setup.*` script, depending on your used shell:

   For bash use:

   ```console
   catkin_ws$ source devel/setup.bash
   ```

   For zsh use:

   ```console
   catkin_ws$ source devel/setup.zsh
   ```

## Usage

The following sections describe how to run the robot simulation and how to make use of the real hardware using the available package launch files.

### Gazebo Simulation with ROS Control

Control the robot inside Gazebo and view what it sees in RViz using the following launch file:

```console
roslaunch diffbot_control diffbot.launch
```

This will launch the default diffbot world `db_world.world`.

To run the [turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/models/turtlebot3_world) 
make sure to download it to your `~/.gazebo/models/` folder, because the `turtlebot3_world.world` file references the `turtlebot3_world` model.
After that you can run it with the following command:

```console
roslaunch diffbot_control diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

| `db_world.world` | `turtlebot3_world.world` | 
|:-------------------------------------:|:--------------------------------:|
| ![corridor-world](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/control/diffbot_world_control.png) | ![turtlebot3-world](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/control/diffbot-turtlebot3-world.png) |

#### Navigation

To navigate the robot in the Gazebo simulator in `db_world.world` run the command:

```console
roslaunch diffbot_navigation diffbot.launch
```

This uses a previously mapped map of `db_world.world` (found in [`diffbot_navigation/maps`](./diffbot_navigation/maps/)) that is served by
the [`map_server`](http://wiki.ros.org/map_server). With this you can use the [2D Nav Goal in RViz](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#A2D_Nav_Goal) directly to let the robot drive autonomously in the `db_world.world`.

[![DiffBot navigation](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/raw/main/docs/resources/navigation/db_world-nav.gif)](https://youtu.be/2SwFTrJ1Ofg)

To run the `turtlebot3_world.world` (or your own stored world and map) use the same `diffbot_navigation/launch/diffbot.launch` file but change
the `world_name` and `map_file` arguments to your desired world and map yaml files:

```console
roslaunch diffbot_navigation diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world' map_file:='$(find diffbot_navigation)/maps/map.yaml'
```

[![DiffBot navigation](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/raw/main/docs/resources/navigation/diffbot-navigation-gazebo-turtlebot3-world-small.gif)](https://youtu.be/2SwFTrJ1Ofg)

#### SLAM

To map a new simulated environment using slam gmapping, first run

```console
roslaunch diffbot_gazebo diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

and in a second terminal execute

```console
roslaunch diffbot_slam diffbot_slam.launch slam_method:=gmapping
```

Then explore the world with the [`teleop_twist_keyboard`](http://wiki.ros.org/teleop_twist_keyboard) or with the already launched [`rqt_robot_steering`](https://wiki.ros.org/rqt_robot_steering) GUI plugin:

[![DiffBot slam](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/slam/diffbot-slam.gif)](https://youtu.be/gLlo5V-BZu0)

When you finished exploring the new world, use the [`map_saver`](http://wiki.ros.org/map_server#map_saver) node from the [`map_server`](http://wiki.ros.org/map_server) package to store the mapped enviornment:

```console
rosrun map_server map_saver -f ~/map
```


### RViz

View just the `diffbot_description` in RViz.

```console
roslaunch diffbot_description view_diffbot.launch
```
![DiffBot RViz](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/rviz_diffbot_meshes.png)



Your contributions are most welcome. These can be in the form of raising issues, creating PRs to correct or add documentation and of course solving existing issues or adding new features.


## :pencil: License

`diffbot` is licenses under the [BSD 3-Clause](./LICENSE).
See also [open-source-license-acknowledgements-and-third-party-copyrights.md](open-source-license-acknowledgements-and-third-party-copyrights.md).
The [documentation](https://ros-mobile-robots.com/) is licensed differently,
visit its [license text](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io#license) to learn more.
