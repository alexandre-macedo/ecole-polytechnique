#include "busline.hpp"

#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
	std::list <Station *> sta;
	std::vector <Station *> linestarts;
	
	InputParser input(argc, argv);
    const std::string &filenameinput = input.getCmdOption("-f");
    if (filenameinput.empty()){
		std::cout << "Pas de fichier de données !" << std::endl;
		return -1;
    }
	
	if (!exists_file(filenameinput)) {
		std::cout << "Le fichier de données n'existe pas !" << std::endl;
		return -1;
	}

	Lecture(filenameinput, sta, linestarts, false);
		
	if (MontrerLignes(linestarts)) std::cout << "ERREUR à la visualisation de lignes" << std::endl;
	
	return 0;
}
