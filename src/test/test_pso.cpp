#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>


class Particle
{
	public:
		Particle();
		~Particle();
		void move(double *gbp);
		void personal_update();

		// particle dimensions = number of solutions
		int dimension_ = 2;
		// period
		int dt_ = 30;
		// Inertia
		double inertia_ = 0.9;
		// acc rand range
		double c1_ = 0.8;
		double c2_ = 0.8;

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

Particle::Particle(){
	// dimension_の粒子を1つ用意
	position_ = new double[dimension_];
	velocity_ = new double[dimension_];
	personal_best_pos_ = new double[dimension_];

	// Initialize of pos, velocity, pbest
	for(int i=0; i<dimension_; i++){
		position_[i] = xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
		velocity_[i] = vminj + (vmaxj - vminj) * ((double)rand() / RAND_MAX);
		personal_best_pos_[i] = position_[i];
	}

	// debag
    // printf("Init pos %f, %f", position_[0], position_[1]);

	// 1つの粒子の評価値を算出
	evaluate();
}

Particle::~Particle(){
	delete[] position_;
	delete[] velocity_;
	delete[] personal_best_pos_;
 }

void Particle::move(double *gbp){
	for (int i=0; i<dimension_; i++){
		velocity_[i] = inertia_ * velocity_[i]
				+ c1_ * ((double)rand() / RAND_MAX) * (personal_best_pos_[i] - position_[i])
				+ c2_ * ((double)rand() / RAND_MAX) * (gbp[i] - position_[i]);
		position_[i] = position_[i] + velocity_[i];
	}

	evaluate();
}

// 目的関数 この関数のパラメータを最適化する
// Optimize the parameters of this function
void Particle::evaluate(){
	// double A = particles_->position_[0];
	// double B = particles_->position_[1];
	// double theta = particles_->position_[2];
	// double t = particles_->position_[3];
	// double c = particles_->position_[4];

	// return A/B*sin(theta + t) - c;

	double A = position_[0];
	double B = position_[1];
	value = A*A + B*B;
	// printf("value: %f\n", value);
}

void Particle::personal_update(){

    if (value < personal_best_value_){
		for(int i=0; i < dimension_; i++){
			personal_best_pos_[i] = position_[i];
		}
		personal_best_value_ = value;
	}
}


class Swarm{
	public:
		Particle* particles;
		Swarm();
		~Swarm();
		void move();
		// particle swarm size = particle * size
		int swarm_size = 100;
		int dimension;
		int dt;	
		double rand01 = ((double)rand() / RAND_MAX);

		double *global_best_pos = nullptr;
		double global_best_value = DBL_MIN;
		int gbest_num;
};

Swarm::Swarm()
{
	particles = new Particle[swarm_size];
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
}

Swarm::~Swarm()
{
	delete [] particles;
	delete [] global_best_pos;
}

void Swarm::move()
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

	printf("global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);
}


int main(){
	srand((unsigned int)time(NULL));

	Swarm swarm;
	for (int i=0; i < swarm.dt; i++){
		printf("Generation %d, ", i);
		swarm.move();
	}
	
    return 0;
}