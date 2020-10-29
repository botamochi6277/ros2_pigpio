# ROS2 PIGPIO

A ROS2 package to use [pigpio](http://abyz.me.uk/rpi/pigpio/).
This package is tested with Raspberry Pi 4 Model B with Ubuntu 20.04 LTS and foxy.

## Install

1. Install pigpio following official instruction
1. Download this repository in `<your ros2 workspace>/src`
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

## Write PWM Signals

To write pwm signal with GPIO-18, run a following command. 

```
ros2 run ros2_pigpio gpio_pwm_writer  --ros-args --param pin:=18
```

A duty cycle is 0--255 which corresponded with 0%--100%
Write 128 (50%) pwm signal:

```
ros2 topic pub --once gpio_pwm_18 std_msgs/msg/Int16 '{data: 128}'
```