#include "Molecule.h"
#include <fstream>
#include <limits>
#include <cstdlib>

double squaredDistance(const double p[3], const double q[3]) {
	double x = 0.0;
	x += (p[0] - q[0]) * (p[0] - q[0]);
	x += (p[1] - q[1]) * (p[1] - q[1]);
	x += (p[2] - q[2]) * (p[2] - q[2]);
	return x;
}

Boule::Boule(const double c[3], const double r) {
	centre[0] = c[0];
	centre[1] = c[1];
	centre[2] = c[2];
	radius = r;
}

bool Boule::contains(const double pt[3]) {
	return squaredDistance(pt, centre) <= radius * radius;
}

// reads a molecule from a text file.
// File format: one line per Boule, each line in the form "x y z r"
// where (x,y,z) is the centre and r is the radius of the Boule
// No checking is performed here, we assume that the file is well-formed!
std::vector<Boule> readMolecule(const char *filename) {
	std::vector<Boule> mol;
	std::ifstream file(filename);
	double c[3], r;
	while (file >> c[0] >> c[1] >> c[2] >> r) {
		Boule b(c, r);
		mol.push_back(b);
	}
	return mol;
}

// Constructs a suitable bounding box for a molecule,
// storing it in the first argument
void boundingBox(double BB[3][2], std::vector<Boule> mol) {
	double max_x, min_x, max_y, min_y, max_z, min_z;
	BB[0][0] = BB[1][0] = BB[2][0] = std::numeric_limits<double>::infinity();
	BB[0][1] = BB[1][1] = BB[2][1] = -std::numeric_limits<double>::infinity();
	for (unsigned i=0; i < mol.size(); i++)
	{
	max_x = mol[i].centre[0] +  mol[i].radius;
	min_x = mol[i].centre[0] -  mol[i].radius;
	max_y = mol[i].centre[1] +  mol[i].radius;
	min_y = mol[i].centre[1] -  mol[i].radius;
	max_z = mol[i].centre[2] +  mol[i].radius;
	min_z = mol[i].centre[2] -  mol[i].radius;
	if (BB[0][0] > min_x)
		BB[0][0] = min_x;
	if (BB[1][0] > min_y)
		BB[1][0] = min_y;
	if (BB[2][0] > min_z)
		BB[2][0] = min_z;
	if (BB[0][1] < max_x)
		BB[0][1] = max_x;
	if (BB[1][1] < max_y)
		BB[1][1] = max_y;
	if (BB[2][1] < max_z)
		BB[2][1] = max_z;
	}

}

// A random point in the given bounding box;
// the first argument is modified to hold the result.
// Uses rand( ) (which should be seeded externally).
void randomPoint(double pt[3], const double BB[3][2]) {
	for (int i=0; i<3; i++)
		pt[i]=(BB[i][1]-BB[i][0])*rand()/(double)RAND_MAX+BB[i][0];
}
