# GROUP 36
- D’Alberton Enrico, enrico.dalberton@studenti.unipd.it
- Girotto Pietro, pietro.girotto@studenti.unipd.it
- Jassal Arjun, arjun.jassal@studenti.unipd.it

## Folder Setup
Before starting, please make sure to have the `/navigation` package and the `/tiago-iaslab-simulation` inside the `~/catkin_ws/src` folder. This is what your folders should look like:

    /catkin_ws
    │
    └── /src
      ├── /navigation
      │ └── ... (other contents of the navigation package)
      │
      └── /tiago-iaslab-simulation
        └── ... (other contents of the tiago-iaslab-simulation package)


## Workspace Setup
On the first terminal, build the workspace with the following commands:

    start_tiago 
    source /opt/ros/noetic/setup.bash && source /tiago_public_ws/devel/setup.bash && cd ~/catkin_ws/ && catkin build && source devel/setup.bash

On two other terminals, run the following commands:

    start_tiago 
    source /opt/ros/noetic/setup.bash && source /tiago_public_ws/devel/setup.bash && cd ~/catkin_ws && source devel/setup.bash

## Running the Simulation
We need three terminals, all of them inside `~/catkin_ws` folder:

**Terminal 1:**

    roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library

**Terminal 2:**

    roslaunch tiago_iaslab_simulation navigation.launch

**Terminal 3:**

    roslaunch navigation launch_movement.launch x:=X y:=Y or:=OR

Parameters: `X`, `Y`, `OR` are respectively x and y coordinate and yaw of the final position.
Suggested values are `X=11.0`, `Y=0.0` and `OR=280.0`, to reach them run:

    roslaunch navigation launch_movement.launch x:=11.0 y:=0.0 or:=280.0