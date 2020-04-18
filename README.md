# CAN Lab Self Driving Car Project

## Dependency
- ROS Melodic
- [MuSHR Project](https://mushr.io)

## How to use
- Clone the project into your ROS workspace

```bash
cd ~/catkin_ws/src/
git clone git@github.com:nyucan/can-racecar-ros.git
```

- Build the ROS project
```bash
cd ~/catkin_ws
catkin_make
```

- Launch the simulator
```bash
roslaunch can_racecar simulator.launch
```

## Modification on MuSHR project
- How to disable the output from `racecar_state`
  - Open the launch file at `mushr/mushr_base/mushr_base/launch/includes/racecar_state.launch`
  - Remove the `output="screen"` from line 5
  - The new line 5 should be:

```
<node pkg="mushr_base" type="racecar_state.py" name="racecar_state">
```
