#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>
#include <cmath>

class MantaRays
{
	public:
		MantaRays();
		~MantaRays();
		// MantaRays dimensions = number of solutions
		int dimension_ = 2;

		double xminj = -1;
		double xmaxj = 1;
		double *position_ = nullptr;
		double value;

	private:
		void evaluate();
};

MantaRays::MantaRays(){
	position_ = new double[dimension_];

	// Initialize the position
	for(int i=0; i<dimension_; i++){
		position_[i] = xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
	}

	evaluate();
}

MantaRays::~MantaRays(){
	delete[] position_;
 }

// Optimize the parameters of this function
void MantaRays::evaluate(){
	double A = position_[0];
	double B = position_[1];
	value = A*A + B*B;
	// printf("value: %f\n", value);
}


class Foraging{
	public:
		MantaRays* manta_rays;
		Foraging();
		~Foraging();
		void move(int current_iter, int max_iter);
		int max_iter = 5; // maximum number of iterations　反復回数
		int population_n = 100; // the size of plankton population 餌の数
		int dimension;
		double rand_select_foraging = ((double)rand() / RAND_MAX);

		double coef;
		double cutoff_rand, r, r1, r2, r3, xrand;
		double xminj, xmaxj;
		double alpha, beta;
		int s = 2; // somersault factor that decides the somersault range of matna rays
		double *global_best_pos = nullptr;
		double global_best_value = DBL_MIN;
		int gbest_num;
};

Foraging::Foraging()
{
	manta_rays = new MantaRays[population_n];
	dimension = manta_rays->dimension_;
	global_best_pos = new double[dimension];
	xminj = manta_rays->xminj;
	xmaxj = manta_rays->xmaxj;

	// debag
	// printf("population: %d\n",population_n);
	// printf("dimension: %d\n",dimension);
	// for (int i=0; i < population_n; i++){
	// 	printf("%d: x:%f, y:%f, value:%f\n",
	// 	    i, manta_rays[i].position_[0], manta_rays[i].position_[1], manta_rays[i].value);
	// }

	gbest_num = 0;
	for (int i=0; i<population_n; i++){
			if(manta_rays[i].value < manta_rays[gbest_num].value){
				gbest_num = i;
			}
	}

	global_best_value = manta_rays[gbest_num].value;

	for (int i=0; i<dimension; i++){
	    global_best_pos[i] = manta_rays[gbest_num].position_[i];
		// printf("Init gbp %d: %f\n",i, global_best_pos[i]);
	}
}

Foraging::~Foraging()
{
	delete [] manta_rays;
	delete [] global_best_pos;
}


void Foraging::move(int current_iter, int max_iter)
{
	for (int i=0; i < population_n; i++){
		// Cyclone Foraging
		if (rand_select_foraging < 0.5){
			// printf("Cyclone:");
			coef = double(current_iter)/double(max_iter);
			cutoff_rand = ((double)rand() / RAND_MAX);
			r = ((double)rand() / RAND_MAX);
			r1 = ((double)rand() / RAND_MAX);
			beta = 2 * std::exp(r1*(max_iter - current_iter + 1) / max_iter) * sin(2 * M_PI * r1);

			if (coef < cutoff_rand){
				// printf("cutoff\n");
				xrand =  xminj + (xmaxj - xminj) * ((double)rand() / RAND_MAX);
				for (int j=0; j<dimension; j++){
					if(i == 0){
						manta_rays[i].position_[j] = xrand 
								+ r * (xrand - manta_rays[i].position_[j]) 
								+ beta * (xrand - manta_rays[i].position_[j]);
					}
					else{ // population = 2 - N
						manta_rays[i].position_[j] = xrand 
								+ r * (manta_rays[i-1].position_[j] - manta_rays[i].position_[j]) 
								+ beta * (xrand - manta_rays[i].position_[j]);
					}
				}		
			}
			else{
				// printf("normal\n");
				for (int j=0; j<dimension; j++){
					if(i == 0){
						manta_rays[i].position_[j] = global_best_pos[j] 
								+ r * (global_best_pos[j] - manta_rays[i].position_[j]) 
								+ beta * (global_best_pos[j] - manta_rays[i].position_[j]);
					}
					else{ // population = 2 - N
						manta_rays[i].position_[j] = global_best_pos[j] 
								+ r * (manta_rays[i-1].position_[j] - manta_rays[i].position_[j]) 
								+ beta * (global_best_pos[j] - manta_rays[i].position_[j]);
					}	
				}
			}

		}

		// Chain Foraging
		else{
			// printf("Chain\n");
			r = ((double)rand() / RAND_MAX);
			alpha = 2 * r * (sqrt(fabs(log(r))));
			for (int j=0; j<dimension; j++){
				if(i == 0){
					manta_rays[i].position_[j] = manta_rays[i].position_[j]
							+ r * (global_best_pos[j] - manta_rays[i].position_[j]) 
							+ alpha * (global_best_pos[j] - manta_rays[i].position_[j]);
				}
				else{ // population = 2 - N
					manta_rays[i].position_[j] = manta_rays[i].position_[j] 
							+ r * (manta_rays[i-1].position_[j] - manta_rays[i].position_[j]) 
							+ alpha * (global_best_pos[j] - manta_rays[i].position_[j]);
				}				
			}
		}

		if (manta_rays[i].value < global_best_value){
			gbest_num = i;
			global_best_value = manta_rays[gbest_num].value;
			for(int j=0; j<dimension; j++){
				global_best_pos[j] = manta_rays[gbest_num].position_[j];
				// printf("gbp %d: %f\n",j, global_best_pos[j]);
			}
		}
	}

    // Somersault Foraging
	for (int i=0; i<population_n; i++){
		r2 = ((double)rand() / RAND_MAX);
		r3 = ((double)rand() / RAND_MAX);
		for (int j=0; j<dimension; j++){
			manta_rays[i].position_[j] = manta_rays[i].position_[j]
					+ s * (r2 * global_best_pos[j] - r3 * manta_rays[i].position_[j]);
		}
	}

	printf("global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);
}


int main(){
	srand((unsigned int)time(NULL));

	Foraging foraging;
	for (int i=1; i<foraging.max_iter; i++){
		printf("Generation %d, ", i);
		foraging.move(i, foraging.max_iter);
	}
	
    return 0;
}