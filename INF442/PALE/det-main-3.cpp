#include "det-sparse.hpp"

#include <iostream>

int main() {

	std::string nomFichier("MatriceB.txt");

	MatriceCreuse A(nomFichier);

	std::cout << "Nom de fichier: " << nomFichier << std::endl;

	std::cout << std::endl;

	std::cout << "A (version creuse):" << std::endl;
	A.afficherCreuse();

	std::cout << std::endl;

	std::cout << "A (version dense):" << std::endl;
	A.afficherDense();
	std::cout << std::endl;

	std::cout << "Determinant: " << A.Determinant(0) << std::endl;
	
	return 0;

}
