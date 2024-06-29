// Copyright (c) 2024 Takumi Asada
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SWARM_INTELLIGENCE_ROS2__PSO__PSO_SERVICE__PSO_COMPONENT_HPP_
#define SWARM_INTELLIGENCE_ROS2__PSO__PSO_SERVICE__PSO_COMPONENT_HPP_

#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cpp_srvcli_srv/srv/function.hpp"
#include "swarm_intelligence_ros2/pso/pso_service/particle_service_component.hpp"
#include <memory>

using Function = cpp_srvcli_srv::srv::Function;


class PsoComponent : public rclcpp::Node
{
	public:
		ParticleComponent* particles;
		PsoComponent(const rclcpp::NodeOptions & options);
		~PsoComponent();
		void move();
		// particle swarm size = particle * size
		int swarm_size = 100;
		int dimension;
		int dt;	
		double rand01 = ((double)rand() / RAND_MAX);

		double *global_best_pos = nullptr;
		double global_best_value = DBL_MIN;
		int gbest_num;
	private:
		rclcpp::Service<Function>::SharedPtr srv_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

		void srv_callback(const std::shared_ptr<Function::Request> request,
							const std::shared_ptr<Function::Response> response);
};

#endif // SWARM_INTELLIGENCE_ROS2__PSO__PSO_SERVICE__PSO_COMPONENT_HPP_