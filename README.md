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
git clone git@github.com:pran-d/talos_harmonic.git
```

* HTTPS
```sh
git clone https://github.com/pran-d/talos_harmonic.git
```

### Dependencies

> [!tip]
> You can check for any deps availability on your system using<br>
> `ros2 pkg list | grep <NAME>`

Clone the repository containing the Talos robot description package from PAL Robotics into the `${WS}/src` directory:
```sh
git clone https://github.com/pal-robotics/talos_robot.git
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

- Terminal 1: Spawn GZ gui
```sh
ros2 launch talos_harmonic talos_spawn.launch.py
```

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
