#build 
 - colcon build --packages-select joystick_pubsub

#add source 
 - source /opt/itkomi/install/setup.bash

#run joystick
 - ros2 run joystick_pubsub it_joystick