#include <stdlib.h>
#include<stdio.h>
#include <time.h>
#include <cfloat>
#include <cmath>

class Mrfo{
	public:
		Mrfo();
		~Mrfo();
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
		void evaluate(int population);

		int dimension_ = 2; // MantaRays dimensions = number of solutions
		double xminj = -1;
		double xmaxj = 1;
		int population_n = 30; // the size of plankton population 餌の数
		int max_iter = 5; // maximum number of iterations　反復回数

		double rand_select_foraging;
		double coef;
		double cutoff_rand, r, r1, r2, r3, xrand;
		double alpha, beta;
		const int s = 2; // somersault factor that decides the somersault range of matna rays
};

Mrfo::Mrfo()
{
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

Mrfo::~Mrfo()
{
	delete [] manta->position_;
	delete [] global_best_pos;
}

void Mrfo::evaluate(int population){
	double A = manta[population].position_[0];
	double B = manta[population].position_[1];
	manta[population].value = A*A + B*B;
}

void Mrfo::move(int current_iter, int max_iter)
{
	for (int i=0; i < population_n; i++){
		// Cyclone Foraging
		rand_select_foraging = ((double)rand() / RAND_MAX);
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
				// printf("normal\n");
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
			// printf("Chain\n");
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
				// printf("gbp %d: %f\n",j, global_best_pos[j]);
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

	printf("global best: (x, y) = %f, %f: value = %f\n",
		global_best_pos[0], global_best_pos[1], global_best_value);
}


int main(){
	srand((unsigned int)time(NULL));

	Mrfo foraging;
	for (int i=1; i<5; i++){
		printf("Generation %d, ", i);
		foraging.move(i, 5);
	}
	
    return 0;
}