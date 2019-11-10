#include "mpi.h"
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iostream>

// Compile with mpic++ and run with mpirun -np 4
// In this exercise, you have to fill the sections *** TO COMPLETE ***


// Distribution parameters
const double MU_1 = 0.0;
const double SIGMA_1 = 1.0;
const double MU_2 = 2.0;
const double SIGMA_2 = 1.0;
// Samples per process
const int S = 1000000;

double random_0_1() {
	return (double) rand() / RAND_MAX;
}

double sample_normal(double mu, double sigma) {
	double u_1 = random_0_1();
	double u_2 = random_0_1();
	double z = sqrt(-2.0 * log(u_1)) * cos(2.0 * M_PI * u_2);
	return mu + z * sigma;
}

double pdf_normal(double x, double mu, double sigma) {
	return (1.0 / (sqrt(2.0 * M_PI) * sigma)) * exp(-0.5 * pow((x - mu) / sigma, 2));
}

// Sum distance over S samples of X_1 (constant 2 divisor factored out to global average)
double compute() {
	double summation = 0.0;
	for (int i = 0; i < S; i++) {
		double x_i = sample_normal(MU_1, SIGMA_1);
		double pdf_1 = pdf_normal(x_i, MU_1, SIGMA_1);
		double pdf_2 = pdf_normal(x_i, MU_2, SIGMA_2);
		summation += fabs(1.0 - (pdf_2 / pdf_1));
	}
	return summation;
}

int main(int argc, char *argv[]) {
	// Initialize MPI
	MPI_Init(&argc, &argv);
	// Seed RNG with current time AND rank identifier
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	srand(442 + rank );
	// Local computation
	
	double summation;
	// *** TO COMPLETE ***
	
	// Reduce local results (dummy result, by default summing on root 0)
	double total ;
	// *** TO COMPLETE ***
	 
	// Compute global average and output result
	if (rank == 0) {
		int nprocs;
		MPI_Comm_size(MPI_COMM_WORLD, &nprocs);
		total /= 2.0 * S * nprocs;
		std::cout << total << std::endl;
	}
	// Finalize and exit
	MPI_Finalize();
}
