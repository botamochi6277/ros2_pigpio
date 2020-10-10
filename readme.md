# ROS2 PIGPIO

A ROS2 package to use [pigpio](http://abyz.me.uk/rpi/pigpio/).


## Read Input Pin

To read inputted signal on GPIO-21 (PULL_UP), run a following command. 
```
ros2 run ros2_pigpio gpio_reader --ros-args --param pin:=21
```

You can set a input pin as pull-down as below, 
```
ros2 run ros2_pigpio gpio_reader --ros-args --param pin:=21 --param is_pull_up:=false
```