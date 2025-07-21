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

* [Recommended] SSH
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

1. Clone the ROS2 Talos robot description package into the `${WS}/src` directory:

    ```sh
    git clone https://github.com/pran-d/talos_robot.git
    ```
> [!note] 
> This fork from the [original repository](https://github.com/pal-robotics/talos_robot.git) contains some minor modifications to make it work using ROS2 Control on Gazebo Harmonic. 
> It still contains plugins from the older Gazebo version, which raises errors. These may be ignored unless the plugins are required.

2. Clone the `linear_feedback_controller` and `linear_feedback_controller_msgs` packages into the `${WS}/src` directory:

    ```sh
    git clone https://github.com/pran-d/linear-feedback-controller.git
    git clone https://github.com/loco-3d/linear-feedback-controller-msgs.git
    ```
> [!note] 
> This fork from the [original repository](https://github.com/loco-3d/linear-feedback-controller) contains a fix in the initialization of the feedback gain matrix. 
> Clone the original instead if this [pull request](https://github.com/loco-3d/linear-feedback-controller/pull/105/commits/58ae655923bcc444e9b8a85fd599e7382bb9b429) is merged.

3. Clone the `gz_gep_tools` package to reconfigure Talos into the half-sitting position on initialization.
    ```sh
    git clone https://github.com/pran-d/gz_gep_tools.git
    ```
> [!note]
> This fork from the [original repository](https://gitlab.laas.fr/ostasse/gz_gep_tools) contains some minor modifications to make it work with a URDF spawned inside an empty world (as opposed to a SDF world containing the robot model already).

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

> [!tip]
> Don't forget to source the local setup file at least once after building
> it.<br>
> `source <WORKSPACE>/install/local_setup.<EXTENSION>`

### Terminal 1: Spawn GZ gui with TALOS robot, load controllers
```sh
ros2 launch talos_harmonic talos_gz_load.launch.py
```
### Terminal 2: 
#### Step 1 - Send the robot into the half-sitting configuration
```sh
ros2 run gz_gep_tools control_loop t
```
#### Step 2- Play the physics and activate the controllers:
```sh
ros2 launch talos_harmonic switch_controllers.launch.py
```

## Common Issues

### [LFC] Nan detected in the robot state interface 

When activating the controllers, you may face an error saying the joint states contain NaN values.

This could be due to some race conditions which cause the LFC to launch before the pass-through controllers can populate the sensor variables from the simulation.

To fix this:
- wait for the controllers to load fully and wait few seconds before activating them.
- increase the time delay between resuming the physics and activating the controllers.

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
