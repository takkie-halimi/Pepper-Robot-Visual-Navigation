#!/bin/bash

# Wait for the command topic to be there
until rostopic info /pepper/LeftArm_controller/command > /dev/null 2>&1 && \
      rostopic info /pepper/RightArm_controller/command > /dev/null 2>&1
do
  echo "Waiting for controllers to be ready..."
  sleep 1.0
done

echo "Lowering arms."
sleep 5.0

rostopic pub --once /pepper/LeftArm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: [LElbowRoll, LElbowYaw, LShoulderPitch, LShoulderRoll, LWristYaw]
points:
- positions: [-0.10, 0.0, 1.45, 0.10, -1.0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}" &



rostopic pub --once /pepper/RightArm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: [RElbowRoll, RElbowYaw, RShoulderPitch, RShoulderRoll, RWristYaw]
points:
- positions: [0.10, 0.0, 1.45, -0.10, 1.0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}" &

wait