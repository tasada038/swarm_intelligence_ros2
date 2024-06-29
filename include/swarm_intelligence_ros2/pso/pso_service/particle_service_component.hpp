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