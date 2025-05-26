# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from ros_gz_bridge.actions import RosGzBridge
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():

#     topic_names_dict = {
#         "torso": 2,
#         "head": 2,
#         "leg": {"left": 6, "right": 6},
#         "arm": {"left": 7, "right": 7},
#         "gripper": ["left", "right"]
#     }

#     topic_names = []

#     for key, value in topic_names_dict.items():
#         if isinstance(value, int):
#             topic_names.extend([f"{key}_{i+1}_joint" for i in range(value)])
#         elif isinstance(value, dict):
#             for subkey, subvalue in value.items():
#                 topic_names.extend([f"{key}_{subkey}_{i+1}_joint" for i in range(subvalue)])
#         elif isinstance(value, list): 
#             for subkey in value:
#                 topic_names.append(f"{key}_{subkey}_joint")
        
#     topics = []
#     for topic in topic_names:
#         topics.append({
#             'name': f"/apply_force_{topic}",
#             'type': f"sensor_msgs/msg/Float64", 
#             'gz_topic': f"/model/Pyrene/joint/{topic}/cmd_force"
#         })

#     return LaunchDescription([
#         DeclareLaunchArgument('bridge_name', default_value='controller_bridge'),

#         RosGzBridge(
#             bridge_name=LaunchConfiguration('bridge_name'),
#             topics=topics,
#         ),
#     ])
