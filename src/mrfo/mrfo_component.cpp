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


#include "swarm_intelligence_ros2/mrfo/mrfo_component.hpp"

using namespace std::chrono_literals;

MrfoComponent::MrfoComponent(const rclcpp::NodeOptions & options)
: Node("mrfo_node", options)
{
    this->declare_parameter("dimension", 2);
    this->declare_parameter("population", 30);
    this->declare_parameter("max_iter", 5);

	subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
		"/input",
		10,
		std::bind(&MrfoComponent::timer_callback, this, std::placeholders::_1)\
	);
    publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("/output", 10);
}

MrfoComponent::~MrfoComponent()
{
	delete [] manta->position_;
	delete [] global_best_pos;
}

void MrfoComponent::timer_callback(
	const std_msgs::msg::Float32::SharedPtr msg_data)
{
    srand((unsigned int)time(NULL));
    // Get parameter
	dimension_ = this->get_parameter("dimension").get_parameter_value().get<int>();
	population_n = this->get_parameter("population").get_parameter_value().get<int>();
    max_iter = this->get_parameter("max_iter").get_parameter_value().get<int>();

	// Initialize manta
	this->manta_init();

	double input_data = msg_data->data;
	RCLCPP_INFO(this->get_logger(), "subscribe data %f:", input_data);

    std_msgs::msg::Float32 data;
    std_msgs::msg::Float32 data2;
	// request->a = number of iterations
	for (int i=0; i < max_iter; i++){
    RCLCPP_INFO(this->get_logger(), "Generation %d:", i);
		this->move(i, max_iter);
	}

    data.data = global_best_pos[0];
	data2.data = global_best_pos[1];
    publisher_->publish(data);
	publisher_->publish(data2);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "global best (x, y): (%f, %f)", data.data, data2.data);
}

void MrfoComponent::manta_init(){
	// Initialize the manta ray
	manta = new manta_ray[population_n];
	for (int i=0; i<population_n; i++){
		manta[i].position_ = new double[dimension_];
		for (int j=0; j<dimension_; j++){
			manta[i].position_[j] = xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
		}
		this->evaluate(i);
	}

    // Initialize global best pos and value
	global_best_pos = new double[dimension_];
	gbest_num = 0;
	for (int i=0; i<population_n; i++){
			if(manta[i].value < manta[gbest_num].value){
				gbest_num = i;
			}
	}
	global_best_value = manta[gbest_num].value;
	for (int j=0; j<dimension_; j++){
    global_best_pos[j] = manta[gbest_num].position_[j];
	}
}

void MrfoComponent::evaluate(int population){
	double A = manta[population].position_[0];
	double B = manta[population].position_[1];
	manta[population].value = A*A + B*B;
}

void MrfoComponent::move(int current_iter, int max_iter)
{
	for (int i=0; i < population_n; i++){
		// Cyclone Foraging
		rand_select_foraging = ((double)rand() / RAND_MAX);
		if (rand_select_foraging < 0.5){
			// RCLCPP_INFO(this->get_logger(), "Cyclone:");
			coef = double(current_iter)/double(max_iter);
			cutoff_rand = ((double)rand() / RAND_MAX);
			r = ((double)rand() / RAND_MAX);
			r1 = ((double)rand() / RAND_MAX);
			beta = 2 * std::exp(r1*(max_iter - current_iter + 1) / max_iter) * sin(2 * M_PI * r1);

			if (coef < cutoff_rand){
				// RCLCPP_INFO(this->get_logger(), "cutoff\n");
				xrand =  xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
				for (int j=0; j<dimension_; j++){
					if(i == 0){
						manta[i].position_[j] = xrand
								+ r * (xrand - manta[i].position_[j])
								+ beta * (xrand - manta[i].position_[j]);
					}
					else{ // population = 2 - N
						manta[i].position_[j] = xrand
								+ r * (manta[i-1].position_[j] - manta[i].position_[j])
								+ beta * (xrand - manta[i].position_[j]);
					}
				}
			}
			else{
				// RCLCPP_INFO(this->get_logger(), "normal\n");
				for (int j=0; j<dimension_; j++){
					if(i == 0){
						manta[i].position_[j] = global_best_pos[j]
								+ r * (global_best_pos[j] - manta[i].position_[j])
								+ beta * (global_best_pos[j] - manta[i].position_[j]);
					}
					else{ // population = 2 - N
						manta[i].position_[j] = global_best_pos[j]
								+ r * (manta[i-1].position_[j] - manta[i].position_[j])
								+ beta * (global_best_pos[j] - manta[i].position_[j]);
					}
				}
			}
		}

		// Chain Foraging
		else{
			// RCLCPP_INFO(this->get_logger(), "Chain\n");
			r = ((double)rand() / RAND_MAX);
			alpha = 2 * r * (sqrt(fabs(log(r))));
			for (int j=0; j<dimension_; j++){
				if(i == 0){
					manta[i].position_[j] = manta[i].position_[j]
							+ r * (global_best_pos[j] - manta[i].position_[j])
							+ alpha * (global_best_pos[j] - manta[i].position_[j]);
				}
				else{ // population = 2 - N
					manta[i].position_[j] = manta[i].position_[j]
							+ r * (manta[i-1].position_[j] - manta[i].position_[j])
							+ alpha * (global_best_pos[j] - manta[i].position_[j]);
				}
			}
		}
		if (manta[i].value < global_best_value){
			gbest_num = i;
			global_best_value = manta[gbest_num].value;
			for(int j=0; j<dimension_; j++){
				global_best_pos[j] = manta[gbest_num].position_[j];
			}
		}
	}

    // Somersault Foraging
	for (int i=0; i<population_n; i++){
		r2 = ((double)rand() / RAND_MAX);
		r3 = ((double)rand() / RAND_MAX);
		for (int j=0; j<dimension_; j++){
			manta[i].position_[j] = manta[i].position_[j]
					+ s * (r2 * global_best_pos[j] - r3 * manta[i].position_[j]);
		}
	}
    RCLCPP_INFO(this->get_logger(), "global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);
}
