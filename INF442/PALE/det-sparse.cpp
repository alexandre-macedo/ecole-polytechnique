#include "det-sparse.hpp"

#include <iostream>
#include <fstream>

MatriceCreuse::MatriceCreuse(int i, int j, MatriceCreuse* G) {

        this->nombreDeColonnes = G->nombreDeColonnes -1;

	std::vector<double> ligneCourante;
        std::vector<int> indiceCourant;
        
	for (int k = 0; k < G->ligne.size(); ++k) {
                for (int n = 0; n < G->ligne[k].size(); ++n) {
                        if (j!=G->indice[k][n]) {
                                ligneCourante.push_back(G->ligne[k][n]);
                                indiceCourant.push_back(G->indice[k][n]-1);
                        }
                }
      		if (k!=i) {
                   this->ligne.push_back(ligneCourante);
                   this->indice.push_back(indiceCourant);
      		}
        }

}



MatriceCreuse::MatriceCreuse(const std::string& nomFichier) {

	// construction d'une matrice creuse a partir d'une matrice dense lue dans un fichier

	std::ifstream fichier(nomFichier.c_str());

	int nombreDeLignes;

	fichier >> nombreDeLignes;
	fichier >> nombreDeColonnes;

	for (int i = 0; i < nombreDeLignes; ++i) {

		std::vector<double> ligneCourante;
		std::vector<int> indiceCourant;

		for (int j = 0; j < nombreDeColonnes; ++j) {

			double element;
			fichier >> element;

			if (element != 0.0) {

				ligneCourante.push_back(element);
				indiceCourant.push_back(j);
			}
		}

		ligne.push_back(ligneCourante);
		indice.push_back(indiceCourant);
	}
}

MatriceCreuse::~MatriceCreuse() {}

MatriceCreuse * MatriceCreuse::SubmatriceMineur(int i, int j) {
	static MatriceCreuse B(i, j, this);
	
	return &B;
}

long MatriceCreuse::Determinant(int show = 0) {

        if (nombreDeColonnes != ligne.size()) {         // matrice non carrée
                return 0;
        }

        if (nombreDeColonnes == 1) {
      return ligne[0][0];
   }
   if (nombreDeColonnes == 2) {
      if (ligne[0].size() == 2) {
         if (ligne[1].size() == 2) {
            return ligne[0][0]*ligne[1][1]-ligne[0][1]*ligne[1][0];
         } else if (indice[1][0] ==0) {
            return -ligne[0][1]*ligne[1][0];
         }
      } else {
         if (indice[0][0] == 0) {
            if (indice[1][0]==1) {
               return ligne[0][0]*ligne[1][0];
            } else if (ligne[1].size()==2) {
               return ligne[0][0]*ligne[1][1];
            } else {
               return 0;
            }
         } else {
            if (indice[1][0]==0) {
               return -ligne[0][0]*ligne[1][0];
            } else {
               return 0;
            }
         }
      }
   }

   long sum = 0;
   for (int i=0 ; i<nombreDeColonnes; i++) {
     // on choisit j=0
      if (indice[i][0]==0) {
         MatriceCreuse m = MatriceCreuse(i,0,this);
         long el = m.Determinant();
         if (i%2 == 0) {
            sum += el;
         }
         else {
            sum -= el;
         }
      }
   }
   return sum;


}

void MatriceCreuse::afficherDense() {

	for (unsigned int i = 0; i < ligne.size(); ++i) {

		int indiceColonne = 0;

		for (unsigned int j = 0; j < ligne[i].size(); ++j) {

			while (indiceColonne < indice[i][j]) {

				std::cout << "0 ";
				indiceColonne++;

			}

			std::cout << ligne[i][j];
			indiceColonne++;
			if (j < ligne[i].size() - 1) std::cout << " ";

		}

		while (indiceColonne < nombreDeColonnes) {

			std::cout << " 0";
			indiceColonne++;

		}

		std::cout << std::endl;

	}

}

void MatriceCreuse::afficherCreuse() {

	for (unsigned int i = 0; i < ligne.size(); ++i) {

		for (unsigned int j = 0; j < ligne[i].size(); ++j) {

			std::cout << "(" << indice[i][j] << ", " << ligne[i][j] << ")";
			if (j < ligne[i].size() - 1) std::cout << " ";

		}

		std::cout << std::endl;

	}

}

int MatriceCreuse::detSarrus2() {

	std::vector <std::vector <int> > val(2, std::vector <int>(2));

	for (int i = 0; i < 2; ++i) {
		if (ligne[i].size() > 0) {
			if (indice[i][0] == 0) {
				val[i][0] = (int) ligne[i][0];
				if (ligne[i].size() > 1) {
					val[i][1] = (int) ligne[i][1];
				}
			}
			if (indice[i][0] == 1) {
				val[i][1] = (int) ligne[i][0];
			}
		}
	}

	return ( val[0][0] * val[1][1] - val[1][0] * val[0][1] );
}
