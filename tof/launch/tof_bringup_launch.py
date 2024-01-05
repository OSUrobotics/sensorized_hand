import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tof',
            executable='tof_publisher',
            name='tof_publisher_right',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"publisher_name": "tof_right"},
                {"i2c_bus": "/dev/i2c-4"}]),
        launch_ros.actions.Node(
            package='tof',
            executable='tof_publisher',
            name='tof_publisher_left',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"publisher_name": "tof_left"},
                {"i2c_bus": "/dev/i2c-1"}])
  ])