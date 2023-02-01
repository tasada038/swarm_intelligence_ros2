# swarm_intelligence_ros2
ROS2 package for swarm intelligence.

## Requirements
- Linux OS
    - Ubuntu 20.04 Laptop PC
- ROS
    - Foxy Fitzroy

## Install & Build
The following commands download a package from a remote repository and install it in your colcon workspace.

```
mkdir -p ~/manta_ws/src
cd ~/manta_ws/src
git clone https://github.com/tasada038/swarm_intelligence_ros2.git
cd ~/manta_ws && colcon build -
```

## Run
Run the following command to run the component node.

### PSO
1. Publish the /input data
```
ros2 run swarm_intelligence_ros2 test_data
```

2. Subscribe the /input data and, publish pso value of /output
```
ros2 run swarm_intelligence_ros2 pso_topic
```

If you want to use ROS2 service client communication, run the following command.

```
ros2 launch swarm_intelligence_ros2 pso_cli.launch.py
```

### MRFO
1. Publish the /input data
```
ros2 run swarm_intelligence_ros2 test_data
```

2. Subscribe the /input data and, publish mrfo value of /output
```
ros2 run swarm_intelligence_ros2 mrfo_node
```

### swarm_intelligence_ros2 parameter
The parameters for the swarm_intelligence_ros2 are as follows.
```
$ ros2 param list
/pso_node:
  c1
  c2
  dimension
  dt
  inertia
  swarm_size
  use_sim_time

/mrfo_node:
  dimension
  max_iter
  population
  use_sim_time
```

## License
This repository is licensed under the Apache 2.0, see LICENSE for details.