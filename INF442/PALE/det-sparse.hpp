	#pragma once

#include <string>
#include <vector>

class MatriceCreuse {

public:

	MatriceCreuse(int i, int j, MatriceCreuse* G);
	MatriceCreuse(const std::string& nomFichier);
	~MatriceCreuse();

	MatriceCreuse * SubmatriceMineur(int i, int j);
	long Determinant(int show);

	void afficherDense();
	void afficherCreuse();

	// le nombre de colonnes de la matrice (le nombre de lignes est ligne.size())
	int nombreDeColonnes;

	// ligne[i] contient les elements non nuls de la ligne i
	std::vector<std::vector<double> > ligne;

	// indice[i] contient les indices des elements non nuls de la ligne i
	std::vector<std::vector<int> > indice;			

private:

	int detSarrus2();

};
