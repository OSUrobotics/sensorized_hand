colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON


. install/setup.bash

ros2 run tof talker 

colcon build --packages-select <>

ros2 interface show hand_msgs/msg/Tofzone


colcon clean workspace