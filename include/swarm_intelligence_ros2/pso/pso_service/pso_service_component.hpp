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