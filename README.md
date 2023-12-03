colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

source /opt/ros/humble/setup.bash
. install/setup.bash
pip install setuptools==58.2.0

ros2 run tof talker 

colcon build --packages-select tof

ros2 interface show hand_msgs/msg/Tofzone


colcon clean workspace