# talos_mpc

`talos_mpc` is a ROS 2 package that provides controllers for the PAL Robotics **Talos** humanoid robot. 

1. **PD Controller** — a Proportional-Derivative controller for baseline joint-space tracking.
2. **MPC Controller** — a Model Predictive Controller that uses the [Crocoddyl](https://github.com/loco-3d/crocoddyl) library to solve the underlying OCP in real time.


## ⚙️ Usage 

They are written as nodes that publish data to and receive data from topics created by a [linear feedback controller](https://github.com/pran-d/linear-feedback-controller.git). The nodes can be run from a ROS2 launch file as shown below.

> [!note]
> This should be done along with the activation of pass-through controllers and linear feedback controllers as shown in `activate_controllers.launch.py` in the [talos_harmonic](https://github.com/pran-d/talos_harmonic/tree/dev/talos_harmonic) package.

1. **PD Controller**

    ```
    return LaunchDescription([ 
        Node(
            package="talos_mpc",
            executable="pd_controller",
            output="screen",
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )
    ])
    ```

2. **MPC Controller** (using Crocoddyl)

    ```
    return LaunchDescription([ 
        Node(
            package="talos_mpc",
            executable="mpc_ros_interface",
            output="screen",
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )
    ])
    ```
