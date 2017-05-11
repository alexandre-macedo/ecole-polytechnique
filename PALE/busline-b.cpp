#include "busline.hpp"

#include <iostream>
#include <fstream>

/* A executer avec input BusHK.txt */

int main(int argc, char **argv) {
	std::list <Station *> sta;
	std::vector <Station *> linestarts;
	std::string filenameoutput = "sortie-HK-b.txt";

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
	
	std::cout << std::endl << "- Insertion d'un nouvel arrêt (RoyalHongKongYachtClub) entre CrossHarbourTunnelToll et CharterHouse, ligne 2" << std::endl;
	Station * inserted = Inserer("RoyalHongKongYachtClub", 2, getStationByName("CrossHarbourTunnelToll", sta), getStationByName("CharterHouse", sta), sta, linestarts);
	if (! inserted) {
		std::cout << "ERREUR ! L'insertion a echouée" << std::endl;
		return -1;		
	}

	if (MontrerLigne(linestarts, 2)) std::cout << "ERREUR à la visualisation de lignes" << std::endl;

	std::cout << std::endl << "- Suppression de l'arrêt SouthornPlayground (1,2)" << std::endl;
	if (Effacer(getStationByName("SouthornPlayground", sta), sta, linestarts)) {
		std::cout << "ERREUR ! L'arrêt n'a pas pu être supprimé (SouthornPlayground)" << std::endl;
		return -1;		
	}

	int arr[2] = {1,2};
	std::vector<int> lines (arr, arr + sizeof(arr) / sizeof(arr[0]) );
	//std::vector<int> lines = {1,2};		// C++-11
	
	if (MontrerLignes(linestarts, lines)) std::cout<<"ERREUR à la visualisation de lignes" << std::endl;
	
	std::cout << std::endl;
	
	Ecriture(filenameoutput, sta, linestarts);
	
	return 0;
}

