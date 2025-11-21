# Velocity-Level CLIK of UAMs

This is the code for a ROS2 controller node that computes the Closed-Loop Inverse Kinematics (CLIK) of the Unmanned Aerial Manipulator (UAM) of the Department of Industrial Engineering of University of Padova (UniPD). It works for ROS2 Humble (Ubuntu 22.04).

The UAM is composed by:
- A custom hexarotor platform assembled at DII with Tarot T960 frame. The UAV mounts a Pixhawk 6C autopilot with PX4 v1.15.2 installed.
- A commercial robotic arm: the Trossen WidowX250S Mobile.

<div align="center">
  <img src="media/UAM.gif" alt="UAM">
</div>

This code for now allows to keep the end-effector at a certain pose and to track a circular trajectory.

## Prerequisites

1) First you need to have followed all [the steps](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#amd64-architecture) for having the `interbotix_ws` in your machine.

2. For the SITL simulation in [Gazebo Harmonic](https://docs.px4.io/main/en/sim_gazebo_gz/#gazebo-simulation) you will need to [clone the source code of the PX4-Autopilot](https://docs.px4.io/main/en/dev_setup/building_px4.html#building-px4-software). You will need also to copy the `t960a.sdf` files to the `~/PX4-Autopilot/Tools/simulation/gz/models` folders. Remember also to copy the `6006_gz_t960a` file in the folder `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`.

3. For the dynamics and kinematics computations you will need to have [Pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/index.html) properly installed on your machine. For the intallation using ROS2 follow the steps at [this link](https://github.com/stack-of-tasks/pinocchio#ros). The main steps are

```bash
sudo apt install ros-$ROS_DISTRO-pinocchio
sudo apt-get install ros-${ROS_DISTRO}-kinematics-interface-pinocchio
```

## Installation

```bash
cd ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms
git clone git@github.com:P3dr1x/clik1_node_pkg.git
cd ~/interbotix_ws
colcon build --packages-select clik1_node_pkg --symlink-install
```
Do not worry if some warnings on Pinocchio arise after the build. 

## Usage with PX4 SITL simulation

In the first terminal launch

```bash
ros2 launch clik1_node_pkg clik_sitl.launch.py
```
If you want to subscribe to the simulated signals coming from PX4 for getting drone pose use `ros2 launch clik1_node_pkg clik_sitl.launch.py real_system:=true`. Pose data are taken from the `/real_t960a_pose` topic.

> [!NOTE] 
> Be sure that PX4 topics are exposed to ROS2. If in SITL simulation run `sudo MicroXRCEAgent udp4 -p 8888` in another terminal.

If you want also Rviz visualization in order to see the desired pose vs the actual one, in another terninal launch

```bash
ros2 launch clik1_node_pkg clik_uam_visual.launch.py
```
Also here you can use the `real_system:=true` option.

In order to plan the cartesian trajectory, in another terminal run

```bash 
ros2 run clik_node_pkg planner 
```
Now the user will be asked to choose which action to perform with the end-effector (for now only positioning and circu;ar trakjectory tracking can be ordered).
1. If `Positioning` is chosen, user will be asked to type the desired EE pose w.r.t. the current pose of the manipulator base. The user has to type 7 numbers (desired position + quaternion). 
If no or invalid input is given by the user, the desired relative EE pose commanded will be `{0.45 0.0 0.36 0 0 0 1}`.
2. If `Circular trajectory (x-z plane)` is chosen, the user will be asked to insert 2 parameters of the trajectory (radius of the trajectory and time of completion). The trajectory will begin from the current EE pose.
3. If `Polyline trajectory` is chosen, the user will be asked to input the trajectory waypoints coordinates. If Enter is pressed, a default trajectory will be commanded (a rectangle). The end-effector will go in each commanded waypoint through a linear path. The user can also choose the time of travel for the segments in the path.

For running the controller 

```bash
ros2 run clik1_node_pkg clik_uam_node --ros-args -p k_err_x_:=50.0
```


> [!NOTE] 
> By default the node runs with the parameter `use_gazebo_pose:=true`. This means that the node will try to subscribe to the `/world/default/dynamic_pose/info` topic bridged from Gazebo to ROS2 for getting the UAV pose. 

<div align="center">
  <img src="media/cerchio_uam.gif" alt="UAM">
</div>

## Node parameters

Parameter      |Default value |   Description    |
|-------------------|---------------|------------|
| `use_gazebo_pose` | `true` | The node will try to subscribe to the `/world/default/dynamic_pose/info` topic bridged from Gazebo to ROS2 for getting the UAV pose. 
| `k_err_x_` | `20.0` | Is the gain value for the EE feedback. 
| `real_system` | `false` | In some nodes is this parameter that decides if to subscribe to the `/real_t960a_pose` topic.
| `damping_` | `1e-4` | Damping parameter for damped pseudoinversion
| `redundant` | `false` | Choose wether you want to command both position and orientation to the EE or only the position (in that case set `true`).
| `control_rate_hz` | `100.0` | Frequency at which the `update()` loop of the controller node will operate.
| `<joint_name>_weight` | `15.0`, `25.0` | Set the weight of the specific joint. This influences the weight matrix used in the weighted pseudoinversion. You can choose `shoulder`, `forearm_roll`, `wrist_rotate` joints.


## Usage with real system (Motion Capture)

0. Power on the onboard computer. Make sure that the USB cable connecting it to the USB-to-serial dongle is not connected at the startup. Connect it later.

1. Connect to the onboard computer with GCS. If GCS is connected to the same `unipd_DII_RAL` portable network type:

```bash
sudo ssh interbotix@192.168.1.125
```
2. Make sure that the Windows machine is streaming and that both the `t960a` and the `end_effector` bodies are visible in the scene. On the onboard computer type

```bash
cd mocap_px4_bridge_ws
. install/setup.bash
ros2 launch natnet_ros2 natnet_ros2.launch.py conf_file:=drone_plus_arm.yaml
```

3. For having MAVLINK telemetry on the GCS type:
```bash
sudo systemctl start mavlink-router
```
4. Make sure that the arm is connected through USB to the onboard computer.

5. Launch 
```bash
ros2 launch clik1_node_pkg clik_real.launch.py
```

This should also open a Rviz session where it is possible to visualize the configuration of the UAM in real-time.

6. Run the controller
```bash
ros2 run clik1_node_pkg clik_uam_node --ros-args -p real_system:=true
```
7. Run the planner
```bash
ros2 run clik1_node_pkg planner
```

8. Stay safe and enjoy ;)

## Mathematics

The algorithm computes in real-time the reference velocities for manipulators motors in order to get the EE to complete the task planned through the `planner` node. This is done through:

$$\dot{\mathbf{q}} = [\mathbf{J}_{\text{gen}}]^{\dagger}(\mathbf{\nu}_{\text{e,des}}+[\mathbf{K}]\mathbf{e}_x)$$

where:

- $[\mathbf{J}_{\text{gen}}]$ is the **Generalized Jacobian matrix**.
- $\mathbf{\nu}_{\text{e,des}}$ is the EE desired twist
- $[\mathbf{K}]$ is the gain matrix. For now is simply `k_err_x_*Identity(6,6)`.
- $\mathbf{e}_x$ is EE task-space error vector. It is computed as $\mathbf{e}_x = \log ([\mathbf{T}_{w,e}]_{des}[\mathbf{T}_{w,e}]^{-1})$.

For more info check the papers (please consider citing):

- [Pasetto, A.; Vyas, Y.; Cocuzza, S. Zero Reaction Torque Trajectory Tracking of an Aerial Manipulator through Extended Generalized Jacobian. Appl. Sci. 2022, 12, 12254](https://doi.org/10.3390/app122312254)

- [Pedrocco, M.; Pasetto, A.; Fanti, G.; Benato, A.; Cocuzza, S. Trajectory Tracking Control of an Aerial Manipulator in the Presence of Disturbances and Model Uncertainties. Appl. Sci. 2024, 14, 2512](https://doi.org/10.3390/app14062512)