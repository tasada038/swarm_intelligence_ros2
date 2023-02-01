#ifndef SWARM_INTELLIGENCE_ROS2__PSO__PSO_TOPIC__PSO_TOPIC_COMPONENT_HPP_
#define SWARM_INTELLIGENCE_ROS2__PSO__PSO_TOPIC__PSO_TOPIC_COMPONENT_HPP_

#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


class PsoComponent : public rclcpp::Node
{
	public:
		PsoComponent(const rclcpp::NodeOptions & options);
		~PsoComponent();
		struct particle 
		{
			double *position_ = nullptr;
			double *velocity_ = nullptr;
			double *personal_best_pos_ = nullptr;
			double personal_best_value_ = DBL_MIN;
			double value;
		};
		struct particle *particles;  //Declarations that use pointers in structures
		double rand01 = ((double)rand() / RAND_MAX);
		double *global_best_pos = nullptr;
		double global_best_value = DBL_MIN;
		int gbest_num;

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
        void timer_callback(const std_msgs::msg::Float32::SharedPtr msg_data);
		void particle_init();
	    void evaluate(int swarm);
		void particle_move(int swarm);
		void personal_update(int swarm);
		void swarm_move();
		double xminj = -1;
		double xmaxj = 1;
		double vminj = -1;
		double vmaxj = 1;
		int swarm_size_;  // particle swarm size = particle * size
		int dimension_;  // particle dimensions = number of solutions
		int dt_;  // period
		double inertia_;  // Inertia
		double c1_, c2_;  // acc rand range
};

#endif // SWARM_INTELLIGENCE_ROS2__PSO__PSO_TOPIC__PSO_TOPIC_COMPONENT_HPP_