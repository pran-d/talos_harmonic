# dummy_hardware

`dummy_hardware` is a ROS 2 package containing a Gazebo plugin designed to create **dummy hardware interfaces**, when certain desired command or state interfaces are **not included in the URDF** but are required by the controllers.

This package allows you to manually define custom interfaces and populate state interface data from an external ROS 2 topic, enabling integration of controllers without modifying the robot description (URDF/Xacro).

The only modification required is adding the plugin within a `ros2_control` tag to the robot description:
```
<ros2_control name="dummy_hardware" type="system">
    <hardware>
        <plugin>dummy_hardware_gz/DummyHardwareInterfaceGz</plugin>
    </hardware>
</ros2_control>
```
