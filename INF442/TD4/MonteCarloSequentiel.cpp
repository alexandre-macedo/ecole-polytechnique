#include "Molecule.h"
#include <iostream>
#include <cstdlib>
#include <limits>
#include <math.h>

unsigned long monteCarlo(unsigned long e, std::vector<Boule> mol,
		double BB[3][2]) {
	//unsigned int s = 0; // par exemple/defaut
	//srand(s); // pour pouvoir tester et reproduire le comportement du programme
	double point[3];
	unsigned long ePrime = 0;
	for (unsigned long i =0; i < e; i++)
	{
	randomPoint(point, BB);	
 	//std::cout << point[0];
	for (unsigned int j=0; j < mol.size(); j++)
		if (mol[j].contains(point))
		{
			ePrime++;
			break;
		}
	}
	return ePrime;
}

int main(int argc, char **argv) {
	if (argc != 3) {
		std::cerr << "Usage : " << argv[0]
			<< " e moleculefile (where e = # Monte Carlo trials)"
			<< std::endl;
		return 1;
	}

	const long e = atol(argv[1]);
	if (e <= 0) {
		std::cerr << "Argument must be an integer > 0" << std::endl;
		return 1;
	}

	// In the spirit of reproducibility, explicit initialisation of seed
	srand(0);

	// Load molecule.
	std::vector<Boule> molecule = readMolecule(argv[2]);
	// Bounding box.
	double BB[3][2];
	boundingBox(BB, molecule);

	// Compute volume.
	unsigned long ePrime = monteCarlo(e, molecule, BB);
	// TODO: compute correct volume.
	//std::cout << ePrime;
	double vol = (double)ePrime / (double) e * fabs(BB[0][0] - BB[0][1]) * fabs(BB[1][0] - BB[1][1]) * fabs(BB[2][0] - BB[2][1]);

	// Set maximal precision when printing doubles.
	std::cout.precision(std::numeric_limits<double>::digits10 + 1);
	std::cout << "volume : " << vol << std::endl;

	return 0;
}
