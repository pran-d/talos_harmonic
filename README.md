# `talos_harmonic`

This project provides useful tools to ease the deployment of the TALOS robot
using ROS Jazzy and Gazebo Harmonic.

## Install

### Setup

Following the classical ROS2 workflow, create a colcon `<WORKSPACE>` with a
`src` folder (set `<WORKSPACE>` accordingly):

```sh
WS=<WORKSPACE>; mkdir -p ${WS}/src && cd ${WS}/src
```

Then clone this repo inside `<WORKSPACE>/src` using one of:

* [Recommanded] SSH
```sh
git clone git@github.com:ArthurVal/tiago_lfc.git
```

* HTTPS
```sh
git clone https://github.com/ArthurVal/tiago_lfc.git
```

### Dependencies

> [!tip]
> You can check for any deps availability on your system using<br>
> `ros2 pkg list | grep <NAME>`

#### `tiago_description`

If the `tiago_description` dependency is not installed you can fetch them inside
your workspace by doing the following:

> [!important]
> The following commands expect you to be inside `<WORKSPACE>/src`
> (i.e. `cd <WORKSPACE>/src` beforehands)

```sh
wget -O - https://github.com/Tiago-Harmonic/tiago_robot/archive/jazzy.tar.gz | tar -xz --strip=1 tiago_robot-jazzy/tiago_description
wget -O - https://github.com/Tiago-Harmonic/omni_base_robot/archive/jazzy.tar.gz | tar -xz --strip=1 omni_base_robot-jazzy/omni_base_description
wget -O - https://github.com/Tiago-Harmonic/pmb2_robot/archive/jazzy.tar.gz | tar -xz --strip=1 pmb2_robot-jazzy/pmb2_description
wget -O - https://github.com/Tiago-Harmonic/pal_robotiq_gripper/archive/jazzy.tar.gz | tar -xz --strip=1 pal_robotiq_gripper-jazzy/pal_robotiq_description
wget -O - https://github.com/Tiago-Harmonic/pal_hey5/archive/jazzy.tar.gz | tar -xz --strip=1 pal_hey5-jazzy/pal_hey5_description
wget -O - https://github.com/Tiago-Harmonic/pal_gripper/archive/jazzy.tar.gz | tar -xz --strip=1 pal_gripper-jazzy/pal_gripper_description
git clone https://github.com/Tiago-Harmonic/launch_pal.git
git clone https://github.com/Tiago-Harmonic/pal_urdf_utils.git --branch jazzy
```

> [!note]
> Currently this fetches **ONLY** the strict minimum dependencies required to
> make `tiago_lfc` work.<br/>
> If needed, you can fetch the full set of dependencies using:<br/>
> `vcs import . < tiago_lfc/dependencies/tiago_robot.repos`

#### [`linear_feedback_controller`](https://github.com/loco-3d/linear-feedback-controller)

If the `linear_feedback_controller` is not installed you can fetch them inside
your workspace by doing the following:

> [!important]
> The following command expect you to be inside `<WORKSPACE>/src`
> (i.e. `cd <WORKSPACE>/src` beforehands)

```sh
vcs import . < tiago_lfc/dependencies/lfc.repos
```

Additionally, you can install the `linear_feedback_controller` build
dependencies on your system using:

```sh
rosdep install -i -t build --from-paths linear_feedback_controller --from-paths linear_feedback_controller_msgs
```

### Build

Using `colcon`:

```sh
cd <WORKSPACE>
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

> [!tip]
> Don't forget to source the global ROS setup file before building:<br>
> `source /opt/ros/<DISTRO>/setup.<EXTENSION>`

## Usage

### TLDR

> [!tip]
> Don't forget to source the local setup file at least once after building
> it.<br>
> `source <WORKSPACE>/install/local_setup.<EXTENSION>`

- Terminal 1: Spawn GZ gui + Contains logs from `ros_control` etc...
```sh
ros2 launch tiago_lfc gz_tiago_lfc.launch.py
```

- Terminal 2: Interact with GZ
```sh
ros2 launch tiago_lfc switch_controllers.launch.py controllers:='lfc jse' activate:=True
```

### Launch files

This package provides several launch files that can simply be used through:<br/>
`ros2 launch tiago_lfc <FILE> [ARGS...]` .

> [!tip]
> Use `-s` (`ros2 launch tiago_lfc <FILE> -s`) to access the full list of
> arguments

* Generic launch files (doesn't depends on Tiago at all):

| **Launch file**                              | **Description**                                                                              |
|----------------------------------------------|----------------------------------------------------------------------------------------------|
| `robot_description_from_xacro.launch.py`     | Create any `robot_description` from xacro file (mappings can be specified through arguments) |
| `robot_state_publisher_from_xacro.launch.py` | Create `robot_description` (same as above) and start the `robot_state_publisher` associated  |
| `load_controllers.launch.py`                 | Load ROS2 control controllers.                                                               |
| `switch_controllers.launch.py`               | Activate/Deactivate controllers.                                                             |
| `gz_control.launch.py`                       | Interact with the controls of a  GZ world                                                    |
| `gz_server.launch.py`                        | Launch any GZ server                                                                         |
| `gz_spawn.launch.py`                         | Spawn any GZ model inside any GZ world                                                       |

* Tiago's specific launch files (use hard path to `tiago_description` xacro's location):

| **Launch file**                         | **Description**                                           |
|-----------------------------------------|-----------------------------------------------------------|
| `tiago_robot_description.launch.py`     | Create Tiago's `robot_description`                        |
| `tiago_robot_state_publisher.launch.py` | Start Tiago's `robot_state_publisher`                     |
| `gz_tiago_lfc.launch.py`                | Launch everything needed at once to spawn Tiago inside GZ |

> [!note]
> The files used/required from `tiago_description` are: <br/>
> - `tiago_description/robots/tiago.urdf.xacro`;<br/>
> - `tiago_description/config/tiago_configuration.yaml`;

## Common Issues

### [GZ] Robot's model is incomplete

When spawning the robot inside Gazebo, you may see that it's 'incomplete',
missing some 3D pieces on the model.

This is due to the fact that, when running the gazebo server, you have to tell
gazebo the location of 3D models defined wihtin the URDF.

To fix this, you can either:
- Update `GZ_SIM_RESOURCE_PATH` environment variable accordingly (`export
  GZ_SIM_RESOURCE_PATH=<PATH>:<PATH>:...`);
- Use `resource_path:=<PATH>:<PATH>` launch argument;

> [!tip]
> If you only have the `*_description` folders inside `<WORKSPACE>/src`, you can
> simply set the above mentionned variable to `<WORKSPACE>/src`
> (e.g. :`resource_path:=<WORKSPACE>/src`).

### [GZ] ros2_control's `/controller_manager` is not running

If, after launching the simulation, `ros2 node list` doesn't show the
`/controller_manger` and launching the tiago's simulation shows the following
log:

```sh
...
[gz-2] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgz_ros2_control-system.so] : Could not find shared library.`
...
```

It means that gazebo failed to launch the `ros2_control` plugin.

> [!tip]
> You can easily confirm that the plugin is launched by looking at the logs coming
> from `[gz_ros_control]` listing all hardware interfaces.

You can check for the plugin's availability/location on your system using:

```sh
$ dpkg -S libgz_ros2_control
ros-jazzy-gz-ros2-control: /opt/ros/jazzy/lib/libgz_ros2_control-system.so
```

> [!note]
>  If the above command returned:<br>
> `dpkg-query: no path found matching pattern`<br>
> It means that `gz-ros2-control` is not installed.<br>
> You can install it through: `apt install ros-<DISTRO>-gz-ros2-control`

Then, you have to tell GZ the `libgz_ros2_control-system.so` dir path (for some
reasons GZ doesn't automatically add this path to the system plugin lookup path)
by either:
- Update `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable accordingly (`export
  GZ_SIM_SYSTEM_PLUGIN_PATH=/path/to/dir`);
- Use `system_plugin_path:=/path/to/dir` launch argument;

> [!tip]
> You can use the following to automatically get the dir location:<br>
> `dpkg -S libgz_ros2_control | awk '{ print $2 }' | xargs dirname`<br>
> And do the following one liner:<br>
> `export GZ_SIM_SYSTEM_PLUGIN_PATH=$(dpkg -S libgz_ros2_control | awk '{ print $2 }' | xargs dirname)`
