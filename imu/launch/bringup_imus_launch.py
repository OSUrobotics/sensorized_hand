import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='imu',
            executable='imu_publisher',
            name='imu_publisher_right',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"publisher_name": "imu_right"},
                {"imu_bus": 6}]),
        launch_ros.actions.Node(
            package='imu',
            executable='imu_publisher',
            name='imu_publisher_left',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"publisher_name": "imu_left"},
                {"imu_bus": 5}])
  ])