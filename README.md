# Mission_coordination_project

Connect to [RDS](https://app.theconstructsim.com/#/) with your logins.

Go to "My rosjects" and run the project that you created.

## Clone the Project repository
In the same terminal, follow the instructions **ONE AFTER THE OTHER**:

```bash
cd ~/catkin_ws/src && git clone https://github.com/mmahdi/Mission_coordination_project.git

cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash
```
## How to use the simulation

To launch the simulatiom on Gazebo run the following line :

```bash
roslaunch evry_project_description simu_robot.launch
```

## Selecting the strategy to deploy

We developped 3 strategies for the robots to reach their targets. 

### Timing

To run the timing strategy, open a new terminal window/tab and run the following line :

```bash
roslaunch evry_project_strategy agent_timing.launch nbr_robot:=3
```

### Roundabout

To run the roundabout strategy, open a new terminal window/tab and run the following line :

```bash
roslaunch evry_project_strategy agent_roundabout.launch nbr_robot:=3
```
