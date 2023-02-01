#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>


class Pso{
	public:
		Pso();
		~Pso();
		void swarm_move();
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
	    void evaluate(int swarm);
		void particle_move(int swarm);
		void personal_update(int swarm);
		double xminj = -1;
		double xmaxj = 1;
		double vminj = -1;
		double vmaxj = 1;
		int swarm_size_ = 100;  // particle swarm size = particle * size
		int dimension_ = 2;  // particle dimensions = number of solutions
		int dt_ = 30;  // period
		double inertia_ = 0.9;  // Inertia
		double c1_ = 0.8;  // acc rand range
		double c2_ = 0.8;
};

Pso::Pso(){
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

Pso::~Pso(){
	delete [] particles->position_;
	delete [] particles->velocity_;
	delete [] particles->personal_best_pos_;
	delete [] global_best_pos;
}

void Pso::evaluate(int swarm){
	double A = particles[swarm].position_[0];
	double B = particles[swarm].position_[1];
	particles[swarm].value = A*A + B*B;
	// printf("value: %f\n", particles[swarm].value);
}

void Pso::particle_move(int swarm){
	for (int j=0; j<dimension_; j++){
		particles[swarm].velocity_[j] = inertia_ * particles[swarm].velocity_[j]
			+ c1_ * ((double)rand() / RAND_MAX) * (particles[swarm].personal_best_pos_[j] - particles[swarm].position_[j])
			+ c2_ * ((double)rand() / RAND_MAX) * (global_best_pos[j] - particles[swarm].position_[j]);
		particles[swarm].position_[j] = particles[swarm].position_[j] + particles[swarm].velocity_[j];
	}
	this->evaluate(swarm);
}

void Pso::personal_update(int swarm){
    if (particles[swarm].value < particles[swarm].personal_best_value_){ 
		for(int j=0; j<dimension_; j++){
			particles[swarm].personal_best_pos_[j] = particles[swarm].position_[j];
		}
		particles[swarm].personal_best_value_ = particles[swarm].value;
	}
}

void Pso::swarm_move(){
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
	printf("global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);
}

int main(){
	srand((unsigned int)time(NULL));

	Pso swarm;
	for (int i=0; i < 30; i++){
		printf("Generation %d, ", i);
		swarm.swarm_move();
	}
    return 0;
}