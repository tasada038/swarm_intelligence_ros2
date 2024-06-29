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

#ifndef SWARM_INTELLIGENCE_ROS2__MRFO__MRFO_COMPONENT_HPP_
#define SWARM_INTELLIGENCE_ROS2__MRFO__MRFO_COMPONENT_HPP_

#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class MrfoComponent : public rclcpp::Node
{
	public:
		MrfoComponent(const rclcpp::NodeOptions & options);
		~MrfoComponent();
		void move(int current_iter, int max_iter);
		struct manta_ray{
			double *position_ = nullptr;
			double value;
		};
		struct manta_ray *manta;  //Declarations that use pointers in structures
		double *global_best_pos = nullptr;
		double global_best_value = DBL_MIN;
		int gbest_num;

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
        void timer_callback(const std_msgs::msg::Float32::SharedPtr msg_data);
		void manta_init();
		void evaluate(int population);

		int dimension_; // MantaRays dimensions = number of solutions
		int population_n; // the size of plankton population 餌の数
		int max_iter; // maximum number of iterations　反復回数
		double xminj = -1;
		double xmaxj = 1;
		double rand_select_foraging;
		double coef;
		double cutoff_rand, r, r1, r2, r3, xrand;
		double alpha, beta;
		const int s = 2; // somersault factor that decides the somersault range of matna rays
};

#endif // SWARM_INTELLIGENCE_ROS2__MRFO__MRFO_COMPONENT_HPP_