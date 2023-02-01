#ifndef SWARM_INTELLIGENCE_ROS2__PSO__PSO_SERVICE__PARTICLE_COMPONENT_HPP_
#define SWARM_INTELLIGENCE_ROS2__PSO__PSO_SERVICE__PARTICLE_COMPONENT_HPP_

#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>

class ParticleComponent{
	public:
		ParticleComponent();
		~ParticleComponent();
		void move(double *gbp);
		void personal_update();

		int dimension_ = 2;  // particle dimensions = number of solutions
		int dt_ = 30;  // period
		double inertia_ = 0.9;  // Inertia
		double c1_ = 0.8;  // acc rand range c1
		double c2_ = 0.8;  // acc rand range c2
		double xminj = -1;
		double xmaxj = 1;
		double vminj = -1;
		double vmaxj = 1;
		double *position_ = nullptr;
		double *velocity_ = nullptr;
		double *personal_best_pos_ = nullptr;
		double personal_best_value_ = DBL_MIN;
		double value;
	private:
		void evaluate();
};

#endif // SWARM_INTELLIGENCE_ROS2__PSO__PSO_SERVICE__PARTICLE_COMPONENT_HPP_