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

#include "swarm_intelligence_ros2/pso/pso_topic/pso_topic_component.hpp"

using namespace std::chrono_literals;

PsoComponent::PsoComponent(const rclcpp::NodeOptions & options)
: Node("pso_node", options)
{
    this->declare_parameter("swarm_size", 100);
    this->declare_parameter("dimension", 2);
    this->declare_parameter("dt", 50);
    this->declare_parameter("inertia", 0.9);
    this->declare_parameter("c1", 0.8);
    this->declare_parameter("c2", 0.8);

	subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
		"/input", 
		10, 
		std::bind(&PsoComponent::timer_callback, this, std::placeholders::_1)\
	);
    publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("/output", 10);
}

PsoComponent::~PsoComponent(){
	delete [] particles->position_;
	delete [] particles->velocity_;
	delete [] particles->personal_best_pos_;
	delete [] global_best_pos;
}

void PsoComponent::timer_callback(
	const std_msgs::msg::Float32::SharedPtr msg_data)
{
    // Get parameter
	swarm_size_ = this->get_parameter("swarm_size").get_parameter_value().get<int>();
	dimension_ = this->get_parameter("dimension").get_parameter_value().get<int>();
    dt_ = this->get_parameter("dt").get_parameter_value().get<int>();
    inertia_ = this->get_parameter("inertia").get_parameter_value().get<double>();
    c1_ = this->get_parameter("c1").get_parameter_value().get<double>();
    c2_ = this->get_parameter("c2").get_parameter_value().get<double>();

	// Initialize particle
	this->particle_init();

	double input_data = msg_data->data;
	RCLCPP_INFO(this->get_logger(), "subscribe data %f:", input_data);

    std_msgs::msg::Float32 data;
    std_msgs::msg::Float32 data2;
	// request->a = number of iterations
	for (int i=0; i < dt_; i++){
    	RCLCPP_INFO(this->get_logger(), "Generation %d:", i);
		this->swarm_move();
	}	

    data.data = global_best_pos[0];
	data2.data = global_best_pos[1];
    publisher_->publish(data);
	publisher_->publish(data2);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "global best (x, y): (%f, %f)", data.data, data2.data);
}

void PsoComponent::swarm_move()
{
	// RCLCPP_INFO(this->get_logger(), "swarm %d:", swarm_size_);
	for (int i=0; i < swarm_size_; i++){

		this->particle_move(i);
		this->personal_update(i);

		if (particles[i].value < global_best_value){
			gbest_num = i;
			global_best_value = particles[gbest_num].value;
			for(int j=0; j<dimension_; j++){
				global_best_pos[j] = particles[gbest_num].position_[j];
			}
		}
	}
    RCLCPP_INFO(this->get_logger(), "global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);

}

void PsoComponent::particle_init(){
	// Initialize the particle
	particles = new particle[swarm_size_];
	for (int i=0; i<swarm_size_; i++){
		particles[i].position_ = new double[dimension_];
		particles[i].velocity_ = new double[dimension_];
		particles[i].personal_best_pos_ = new double[dimension_];
		for (int j=0; j<dimension_; j++){
			particles[i].position_[j] = xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
			particles[i].velocity_[j] = vminj + (vmaxj - vminj) * ((double)rand() / RAND_MAX);
			particles[i].personal_best_pos_[j] = particles[i].position_[j];
		}
		// printf("Init pos %f, %f", particles[i].position_[0], particles[i].position_[1]);
		this->evaluate(i);
	}
	// Initialize global best pos and value
	global_best_pos = new double[dimension_];
	gbest_num = 0;
	for (int i=0; i<swarm_size_; i++){
		if(particles[i].value < particles[gbest_num].value){
			gbest_num = i;
		}
	}
	global_best_value = particles[gbest_num].value;
	for (int j=0; j<dimension_; j++){
		global_best_pos[j] = particles[gbest_num].position_[j];
	}
}

void PsoComponent::particle_move(int swarm){
	for (int j=0; j<dimension_; j++){
		particles[swarm].velocity_[j] = inertia_ * particles[swarm].velocity_[j]
			+ c1_ * ((double)rand() / RAND_MAX) * (particles[swarm].personal_best_pos_[j] - particles[swarm].position_[j])
			+ c2_ * ((double)rand() / RAND_MAX) * (global_best_pos[j] - particles[swarm].position_[j]);
		particles[swarm].position_[j] = particles[swarm].position_[j] + particles[swarm].velocity_[j];
	}
	this->evaluate(swarm);
}

// Optimize the parameters of this function
void PsoComponent::evaluate(int swarm){
	double A = particles[swarm].position_[0];
	double B = particles[swarm].position_[1];
	particles[swarm].value = A*A + B*B;
}

void PsoComponent::personal_update(int swarm){
    if (particles[swarm].value < particles[swarm].personal_best_value_){ 
		for(int j=0; j<dimension_; j++){
			particles[swarm].personal_best_pos_[j] = particles[swarm].position_[j];
		}
		particles[swarm].personal_best_value_ = particles[swarm].value;
	}
}