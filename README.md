# holydrone_identification

ROS 1 package for BRCT first-order response identification and flight-based
Gazebo physics identification. The vehicle, Gazebo, PX4, and MAVROS launch
assets remain in the separate `holydrone` package.

## Build

```bash
cd /home/holy/QUAD_RL/deployment
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m pip install pyulog
```

## BRCT first-order identification

```bash
roslaunch holydrone_identification real.launch
```

The node pre-streams hover commands. Arm and enter `OFFBOARD` manually to
start the excitation. Exiting `OFFBOARD` aborts the experiment; a completed
sequence requests `POSCTL`. It never arms or takes off automatically.

Process two independent PX4 ULog flights for fitting and validation:

```bash
rosrun holydrone_identification brct1order_process.py \
  --train /data/brct_train.ulg \
  --validation /data/brct_validation.ulg \
  --output /data/brct1order.yaml
```

## Gazebo physics identification

Fill in the measured mass and verify rotor geometry in
`config/gazebo_physics.yaml`, then run:

```bash
roslaunch holydrone_identification gazebo_physics_real.launch
```

Process independent fitting and validation logs:

```bash
rosrun holydrone_identification gazebo_physics_process.py \
  --train /data/physics_train.ulg \
  --validation /data/physics_validation.ulg \
  --config $(rospack find holydrone_identification)/config/gazebo_physics.yaml \
  --output /data/gazebo_physics.yaml
```

## SITL

```bash
roslaunch holydrone_identification sim.launch output:=/data/id_train
```

The online PX4 rate-loop candidate experiment lives in the separate
`holydrone_autotune` package.
