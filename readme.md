# ROS2 PIGPIO

A ROS2 package to use [pigpio](http://abyz.me.uk/rpi/pigpio/).
This package is tested with Raspberry Pi 4 Model B with Ubuntu 20.04 LTS and foxy.

## Install

1. Install pigpio following [official instruction](https://abyz.me.uk/rpi/pigpio/download.html)
1. Download this repository in `<your ros2 workspace>/src`
1. build this package with `colcon build --packages-select ros2_pigpio`


## Preparing

```bash
sudo pigpiod # run pigpio daemon
cd <ros2_ws> # go to your ros2 workspace
source install/local_setup.bash # install local setup
```

## Read Input Pin

To read inputted signal on GPIO-21 (PULL_UP), run the following command. 
A publisher will send read value of the pin.
```
ros2 run ros2_pigpio gpio_reader --ros-args --param pin:=21
```

You can set a input pin in pull-down with the below, 
```
ros2 run ros2_pigpio gpio_reader --ros-args --param pin:=21 --param is_pull_up:=false
```

## Write Output Pin
To write signal with GPIO-21, run a following command. A subscriber will wait for your inputs.
```
ros2 run ros2_pigpio gpio_writer --ros-args --param pin:=21 
```

When you submit a value to the subscriber, it write the signal on the pin. 
Write high signal:
```
ros2 topic pub --once gpio_output_21 std_msgs/msg/Bool '{data: true}'
```

Write low signal:
```
ros2 topic pub --once gpio_output_21 std_msgs/msg/Bool '{data: false}'
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


## Drive Wheels with DC-Motors (H-Bridge)

You can control DC motors with H-Bridge motor drivers.

To control the driver, run `wheels`.
```
ros2 run ros2_pigpio wheels 
```

`wheels` uses [Maker Drive](https://www.cytron.io/p-maker-drive-simplifying-h-bridge-motor-driver-for-beginner) and  GPIO-17, 18 and GPIO-27, 22 for Motor1 and 2, respectively in default settings.

You can drive wheels with publishing `/wheels` (topic).

Move Forward: 
```
ros2 topic pub --once /wheels geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 1.0, y: 0.0, z: 0}}"
```

Move Backward
```
ros2 topic pub --once /wheels geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 1.0, y: 0.0, z: 0}}"
```

Turn Left
```
ros2 topic pub --once /wheels geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

Turn Right
```
ros2 topic pub --once /wheels geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

Stop
```
ros2 topic pub --once /wheels geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```