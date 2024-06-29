# swarm_intelligence_ros2
ROS2 package for swarm intelligence.

## Supported ROS 2 distributions

[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

## Requirements
- Ubuntu OS PC
  - Ubuntu 22.04 Humble

## Install & Build
The following commands download a package from a remote repository and install it in your colcon workspace.

```sh: Terminal
mkdir -p ~/manta_ws/src
cd ~/manta_ws/src
git clone https://github.com/tasada038/swarm_intelligence_ros2.git
cd ~/manta_ws && colcon build --packages-select swarm_intelligence_ros2
```

## Run
Run the following command to run the component node.

### PSO
1. Publish the /input data
```sh: Terminal
ros2 topic pub /input std_msgs/msg/Float32 "{data: 50.0}"
```

2. Subscribe the /input data and, publish pso value of /output
```sh: Terminal
ros2 run swarm_intelligence_ros2 pso_topic
```

### MRFO
1. Publish the /input data
```sh: Terminal
ros2 topic pub /input std_msgs/msg/Float32 "{data: 50.0}"
```

2. Subscribe the /input data and, publish mrfo value of /output
```sh: Terminal
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

[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html

[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
