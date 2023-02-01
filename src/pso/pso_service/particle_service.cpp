#include "swarm_intelligence_ros2/pso/pso_service/particle_service_component.hpp"

ParticleComponent::ParticleComponent(){
	position_ = new double[dimension_];
	velocity_ = new double[dimension_];
	personal_best_pos_ = new double[dimension_];
	// Initialize of pos, velocity, pbest
	for(int i=0; i<dimension_; i++){
		position_[i] = xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
		velocity_[i] = vminj + (vmaxj - vminj) * ((double)rand() / RAND_MAX);
		personal_best_pos_[i] = position_[i];
	}
	evaluate();
}

ParticleComponent::~ParticleComponent(){
	delete[] position_;
	delete[] velocity_;
	delete[] personal_best_pos_;
 }

void ParticleComponent::move(double *gbp){
	for (int i=0; i<dimension_; i++){
		velocity_[i] = inertia_ * velocity_[i]
				+ c1_ * ((double)rand() / RAND_MAX) * (personal_best_pos_[i] - position_[i])
				+ c2_ * ((double)rand() / RAND_MAX) * (gbp[i] - position_[i]);
		position_[i] = position_[i] + velocity_[i];
	}
	evaluate();
}

// Optimize the parameters of this function
void ParticleComponent::evaluate(){
	double A = position_[0];
	double B = position_[1];
	value = A*A + B*B;
}

void ParticleComponent::personal_update(){
    if (value < personal_best_value_){
		for(int i=0; i < dimension_; i++){
			personal_best_pos_[i] = position_[i];
		}
		personal_best_value_ = value;
	}
}