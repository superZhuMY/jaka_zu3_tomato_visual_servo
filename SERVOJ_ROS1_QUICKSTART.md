# JAKA Zu3 ROS1 ServoJ quick start

This branch adds a small, independent ServoJ path for Ubuntu 20.04 + ROS Noetic. It does not change the current YOLO, hand-eye calibration, MoveIt, or tomato-grasp scripts.

## What was added

- `jaka_servo_control`: a fixed-rate bridge from a ROS topic to the existing `/jaka_driver/servo_j` service.
- `servoj_sine_test.py`: a low-amplitude J6 sine-wave bench test.
- Input validation in `jaka_driver`: ServoP and ServoJ now require exactly six finite values.
- Driver shutdown cleanup: ROS shutdown requests `servo_move_enable(false)`.

The official driver still calls `robot.servo_j(..., MoveMode::INCR)`, so every command is a six-joint increment for one control period.

## Build

```bash
cd ~/jaka_robot
catkin build
source devel/setup.bash
```

## Test order

1. Put the robot in a collision-free pose and keep the emergency stop within reach.
2. Start only the JAKA driver. Replace the IP if needed:

```bash
roslaunch jaka_driver robot_start_launch.launch ip:=10.5.5.100
```

3. Check that the driver services exist:

```bash
rosservice list | grep jaka_driver
```

4. Start the conservative J6 test:

```bash
roslaunch jaka_servo_control servoj_test.launch
```

Default motion is J6, ±0.05 rad (about ±2.9 degrees), 0.15 Hz. The per-cycle command is clamped to 0.001 rad, the bridge runs at 125 Hz, and it disables JAKA servo mode after 0.10 s without a fresh command.

Stop with Ctrl+C. The test publishes zero increments, requests bridge shutdown, and the driver also disables servo mode during ROS shutdown.

## Manual command interface

For the next visual-servo controller, publish six joint increments at a fixed rate:

```bash
rostopic pub -r 125 /jaka_servo_control/joint_increment std_msgs/Float64MultiArray \
  "layout: {dim: [], data_offset: 0}
data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0002]"
```

Enable and disable the bridge explicitly:

```bash
rosservice call /jaka_servo_control/enable "data: true"
rosservice call /jaka_servo_control/enable "data: false"
```

Do not run MoveIt trajectory execution and ServoJ at the same time. Use MoveIt only to reach a pre-grasp pose; stop it before enabling ServoJ.

## Next implementation

Replace `servoj_sine_test.py` with a controller that subscribes to the filtered tomato target, computes `Delta q = q_dot * dt` using a damped least-squares Jacobian inverse, and publishes the result to `/jaka_servo_control/joint_increment`.
