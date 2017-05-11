#include "busline.hpp"

#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
	std::list <Station *> sta;
	std::vector <Station *> linestarts;
	//Fichiers de données: BusHK.txt BusCornella.txt

    InputParser input(argc, argv);
    const std::string &filenameinput = input.getCmdOption("-f");
    if (filenameinput.empty()){
		std::cout << "Pas de fichier de données !" << std::endl;
		return -1;
    }

	if (Lecture(filenameinput, sta, linestarts, true)) std::cout << "ERREUR à la lecture du fichier de données." << std::endl;
		
	return 0;
}
