# ROS2 PIGPIO

A ROS2 package to use [pigpio](http://abyz.me.uk/rpi/pigpio/).

## Install

1. Install pigpio following official instruction
1. Download this repository in your ros2 workspace
1. build this package


## Read Input Pin

To read inputted signal on GPIO-21 (PULL_UP), run a following command. 
```
ros2 run ros2_pigpio gpio_reader --ros-args --param pin:=21
```

You can set a input pin as pull-down as below, 
```
ros2 run ros2_pigpio gpio_reader --ros-args --param pin:=21 --param is_pull_up:=false
```

## Write Output Pin
To write signal with GPIO-21, run a following command. 
```
ros2 run ros2_pigpio gpio_writer --ros-args --param pin:=21 
```

Write high signal:
```
ros2 topic pub --once gpio_output_21 std_msgs/msg/Bool "{data:{true}}"
```

Write low signal:
```
ros2 topic pub --once gpio_output_21 std_msgs/msg/Bool "{data:{}}"
```