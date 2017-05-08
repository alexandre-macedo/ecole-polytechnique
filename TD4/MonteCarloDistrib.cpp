#include "Molecule.h"
#include <iostream>
#include <cstdlib>
#include <limits>
#include "mpi.h"

unsigned long monteCarlo(unsigned long e, std::vector<Boule> mol,
		double BB[3][2]) {
	// TODO: complete this function, exactly as in Ex. 3
	unsigned long ePrime = 0;
	return ePrime;
}

int main(int argc, char **argv) {
	const int root = 0;

	int numtasks, taskid;
	MPI_Status status;

	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &taskid);
	MPI_Comm_size(MPI_COMM_WORLD, &numtasks);

	if (argc != 3) {
		if (taskid == root) {
			std::cerr << "Usage : " << argv[0]
				<< " e moleculefile (where e = # Monte Carlo trials)"
				<< std::endl;
		} else {
			// print nothing, fail quietly
		}
		MPI_Finalize();
		return 1;
	}

	const unsigned long e = atol(argv[1]);
	if (e <= 0) {
		if (taskid == root) {
			std::cerr << "Argument must be an integer > 0" << std::endl;
		} else {
			// print nothing, fail quietly
		}
		MPI_Finalize();
		return 1;
	}

	// In the spirit of reproducibility, explicit initialisation of seed
	srand(taskid);

	unsigned long elocal = 0;  // the number of samples for this process

	// TODO: the root decides how may samples will be handled by each of the
	// processors, and MPI_Scatters its decision

	// Load molecule.
	std::vector<Boule> molecule = readMolecule(argv[2]);
	// Bounding box.
	double BB[3][2];
	boundingBox(BB, molecule);

	unsigned long ePrime;
	unsigned long ePrimelocal = monteCarlo(elocal, molecule, BB);

	// TODO: Compute the global ePrime using MPI_Reduce.

	if (taskid == root) {
		// The root computes and prints the final result.
		double vBB = 1.0;
		for (unsigned int i = 0; i < 3; i++) {
			vBB *= (BB[i][1] - BB[i][0]);
		}
		double vol = vBB * (double(ePrime) / double(e));
		// Set maximal precision when printing doubles.
		std::cout.precision(std::numeric_limits<double>::digits10 + 1);
		std::cout << "volume : " << vol << std::endl;
	} else {
		// nothing left to be done.
	}

	MPI_Finalize();
	return 0;
}
