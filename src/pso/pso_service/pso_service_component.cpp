#include "swarm_intelligence_ros2/pso/pso_service/pso_service_component.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

PsoComponent::PsoComponent(const rclcpp::NodeOptions & options)
: Node("pso_node", options)
{
	particles = new ParticleComponent[swarm_size];
	dimension = particles->dimension_;
	dt = particles->dt_;
	global_best_pos = new double[dimension];
	// debag
	// printf("swarmsize: %d\n",swarm_size);
	// printf("dimension: %d\n",dimension);
	// for (int i=0; i < swarm_size; i++){
	// 	printf("%d: x:%f, y:%f, value:%f\n",
	// 	    i, particles[i].position_[0], particles[i].position_[1], particles[i].value);
	// }
	gbest_num = 0;
	for (int i=0; i<swarm_size; i++){
			if(particles[i].value < particles[gbest_num].value){
				gbest_num = i;
			}
	}
	global_best_value = particles[gbest_num].value;
	for (int i=0; i<dimension; i++){
	    global_best_pos[i] = particles[gbest_num].position_[i];
		// printf("Init gbp %d: %f\n",i, global_best_pos[i]);
	}
    srv_ = create_service<Function>(
        "service_test", std::bind(&PsoComponent::srv_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("/output", 10);
}

PsoComponent::~PsoComponent()
{
	delete [] particles;
	delete [] global_best_pos;
}

void PsoComponent::srv_callback(const std::shared_ptr<Function::Request> request,
                  const std::shared_ptr<Function::Response> response)
{
    std_msgs::msg::Float32 data;
    std_msgs::msg::Float32 data2;
	// request->a = number of iterations
	for (int i=0; i < int(request->a); i++){
    	RCLCPP_INFO(this->get_logger(), "Generation %d:", i);
		this->move();
	}	
	// add service, nothing to do with PSO
    // response->d = request->a + request->b + request->c;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request  a: %f, b: %f, c: %f",
    //             request->a, request->b, request->c);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f]", response->d);
    data.data = global_best_pos[0];
	data2.data = global_best_pos[1];
    publisher_->publish(data);
	publisher_->publish(data2);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "global best (x, y): (%f, %f)", data.data, data2.data);
}

void PsoComponent::move()
{
	for (int i=0; i < swarm_size; i++){
		particles[i].move(global_best_pos);
		// particles[i].evaluate();
		particles[i].personal_update();
		if (particles[i].value < global_best_value){
			gbest_num = i;
			global_best_value = particles[gbest_num].value;
			for(int j=0; j<dimension; j++){
				global_best_pos[j] = particles[gbest_num].position_[j];
				// printf("gbp %d: %f\n",j, global_best_pos[j]);
			}
		}
	}
	// printf("global best: (x, y) = %f, %f: value = %f\n",
	// 	global_best_pos[0], global_best_pos[1], global_best_value);
    RCLCPP_INFO(this->get_logger(), "global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);
}
