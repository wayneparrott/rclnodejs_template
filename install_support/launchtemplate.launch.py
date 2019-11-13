from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
  return LaunchDescription([
    launch_ros.actions.Node(
      package='_PKG_', node_executable='_EXECUTABLE_', output='screen'),
    launch_ros.actions.Node(
      package='ros_tf2', node_executable='static_transform_publisher', output='screen',
      arguments=[0, 0, 1, 0, 0, 0, 'Temperature', 'map']),
    launch_ros.actions.Node(
      package='rviz2', 
      node_executable='rviz2', 
      output='screen',
      arguments=['-d', _RVIZ_CONFIG_PATH_'])
  ])
