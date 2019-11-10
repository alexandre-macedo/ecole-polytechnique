#include "busline.hpp"

#include <iostream>
#include <fstream>
#include <list>
#include <assert.h>

template<typename T>
void pop_front(std::vector<T>& vec)
{
    assert(!vec.empty());
    vec.erase(vec.begin());
}

bool exists_file (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

void Station::constructeur(std::string name, int line, Station * previous, Station * next, bool debut) {
	this->name = name;
	this->addLine(line, previous, next, debut);	
}

// Constructeurs
Station::Station(std::string name, int line, Station * previous, Station * next) {	
	constructeur(name, line, previous, next, false);
}

Station::Station(std::string name, int line, Station * previous) {	
	constructeur(name, line, previous, NULL, false);
}

Station::Station(std::string name, int line) {	
	constructeur(name, line, NULL, NULL, false);
}

Station::Station(std::string name, int line, bool debut) {	
	constructeur(name, line, NULL, NULL, debut);
}

// Destructeur
Station::~Station() {}

int Station::howManyLines() {
	return this->nLines;
}

int Station::removeLine(int line) {
	if (! this->getLine(line)) return -1;
	
	Pos * line_to_remove = this->getLine(line);
	
	this->nLines --;
	
	if (this->getSuccessor(line)) {
		if (this->getPredecessor(line)) {
			this->getSuccessor(line)->getLine(line)->addAdjacents(this->getPredecessor(line), NULL);
		}
		else if (this->getLine(line)->isDebutDeLigne()) {
			this->getSuccessor(line)->getLine(line)->removePredecessor();
			this->getSuccessor(line)->getLine(line)->setDebutDeLigne();
		}
		else {
			std::cout << "ERREUR ! Données inconsistentes." << std::endl; 
		}
	}

	if (this->getPredecessor(line)) {
		if (this->getSuccessor(line)) {
			this->getPredecessor(line)->getLine(line)->addAdjacents(NULL, this->getSuccessor(line));
		}
		else {
			this->getPredecessor(line)->getLine(line)->removeSuccessor();
		}
	}
		
	this->connectedLines.erase(std::remove(this->connectedLines.begin(), this->connectedLines.end(), line_to_remove), this->connectedLines.end());
	
	delete line_to_remove;
	
	return 0;
}

int Station::addLine(int line, Station * previous, Station * next, bool debut) {
	if (this->getLine(line)) return -1; // la ligne passe deja par l'arret

	Pos * newPos;
	newPos = new Pos(line, previous, next, debut);
	this->nLines ++;
	this->connectedLines.push_back(newPos);
	
	return 0;
}

int Station::addLine(int line, Station * previous, bool debut) {
	return addLine(line, previous, NULL, debut);
}

int Station::addLine(int line, bool debut) {
	return addLine(line, NULL, NULL, debut);
}

Pos * Station::getLine(int line) {
	std::vector<Pos*>::iterator ite;
	for (ite = (this->connectedLines).begin(); ite!=(this->connectedLines).end(); ++ite) {
		if ((*ite)->idLine == line) return (*ite);
	}
	return NULL;
}

Station * Station::getPredecessor(int line) {
	Pos * inLine = this->getLine(line);
	if (inLine) { return inLine->getPredecessor(); }
	
	return NULL;
}

Station * Station::getSuccessor(int line) {
	Pos * inLine = this->getLine(line);
	if (inLine) { return inLine->getSuccessor(); }
	
	return NULL;
}

// Constructeur
Pos::Pos(int line, Station * nextEast, Station * nextWest, bool debut) {
	this->idLine = line;
	this->nextEast = NULL;
	this->nextWest = NULL;
	
	this->debutDeLigne = debut;
		
	addAdjacents(nextEast, nextWest);
}

// Destructeur
Pos::~Pos() {}

Station * Pos::getPredecessor() {
	if (this->nextEast) return this->nextEast;
	else return NULL;
}

Station * Pos::getSuccessor() {
	if (this->nextWest) return this->nextWest;
	else return NULL;
}

void Pos::addAdjacents(Station * nextEast, Station * nextWest) {
	if (nextEast) this->nextEast = nextEast;
	if (nextWest) this->nextWest = nextWest;
}

void Pos::setDebutDeLigne() {
	this->debutDeLigne = true;
}

void Pos::unsetDebutDeLigne() {
	this->debutDeLigne = false;
}

bool Pos::isDebutDeLigne() {
	return this->debutDeLigne;
}

void Pos::removePredecessor() {
	this->nextEast = NULL;
}

void Pos::removeSuccessor() {
	this->nextWest = NULL;
}

// Autres fonctions

int MontrerArret(Station * st) {
	if (!st) return -1;

	std::vector<Pos *>::iterator it2;
	std::cout << "Arrêt " << (st)->name << ": " << std::endl;
	for (it2=((st)->connectedLines).begin(); it2!=((st)->connectedLines).end(); ++it2) {
		std::cout << "  Ligne " << (*it2)->idLine << "; ";
		if ((*it2)->getPredecessor()) std::cout << "antérieur " << ((*it2)->getPredecessor())->name << " ; ";
		if ((*it2)->getSuccessor()) std::cout << "suivant " << (*it2)->getSuccessor()->name;
		std::cout << std::endl;
	}
	return 0;
}

int MontrerArrets(std::list<Station *> sta) {
	/* a: A remplir */
	return 0;
}

int MontrerLigne(std::vector<Station *> debut, int ligne) {
        std::cout << "  Ligne " << ligne << ":";
   for (std::vector<Station *>::iterator it = debut.begin() ; it< debut.end() ;
         it++ ){
         std::cout << ' ';
         std::cout << (*it)->name;
      }
      std::cout <<std::endl;
        return -1;
}

}

int MontrerLignes(std::vector<Station *> debut) { 
	for (int i=0; i<debut.size(); ++i) {
		if (MontrerLigne(debut, i+1)) return -1;
	}
	return 0;
}

int MontrerLignes(std::vector<Station *> debut, std::vector<int> lignes) {
	for (int i=0; i<lignes.size(); ++i) {
		if (lignes[i]>debut.size()) return -1;
		if (MontrerLigne( debut, lignes[i])) return -1;
	}
	return 0;
}

/* b: A remplir */
Station * Inserer(std::string newStName, int line, Station * arr1, Station * arr2, std::list <Station *> &sta, std::vector <Station *> &linestarts) {

}

/* b: A remplir */
Station * Inserer(std::string newStName, int line, Station * arr, std::list <Station *> &sta, std::vector <Station *> &linestarts) {

}

/* b: A remplir */
int Effacer(Station * st, std::list <Station *> &sta, std::vector <Station *> &linestarts) {

}

Station * getStationByName(std::string nameStation, std::list <Station *> sta) {
	std::list<Station *>::iterator ite;
	
	for (ite = sta.begin(); ite != sta.end(); ++ite) {
		if ((*ite)->name == nameStation) return (*ite);
	}

	return NULL;
}

int Lecture(std::string filename, std::list<Station *> &sta, std::vector<Station *> &linestarts, bool montrer) {

	std::ifstream fichier;
	Station * newStation, * previous;
	int totLignes, nStations, nLignes = 0;
	std::string newStationName;

	fichier.open(filename.c_str());

	if (not fichier.good()) {
		return -1;
	}

	std::cout << "Lecture du fichier de données: " << filename << std::endl;

	fichier >> totLignes;
	if (montrer) std::cout << "  " << totLignes << " lignes enregistrées." << std::endl;

	while (nLignes < totLignes) {
		nLignes ++;
		
		fichier >> nStations;
		if (montrer) std::cout << "  Ligne " << nLignes << ", " << nStations << " arrêts." << std::endl;

		previous = NULL;

		for (int i=0; i< nStations; ++i) {
			fichier >> newStationName;
			// l'arret existe deja ?
			newStation = (Station *) getStationByName(newStationName, sta);
			if (!newStation) {
				// si non, creer objet Station
				if (previous) {
					if (! previous->getLine(nLignes)) {
						std::cerr << "ERREUR ! Exécution abortée." << std::endl;
						exit(-1);
					}
					newStation = new Station(newStationName, nLignes, previous );
				}
				else {
					newStation = new Station(newStationName, nLignes, true);		
				}
				sta.push_back(newStation);
				if (montrer) std::cout << "  Nouvel arrêt " << newStation->name << ", ligne " << nLignes << "." << std::endl;
			}
			else {
				if (previous) {
					if (!newStation->getLine(nLignes)) {
						newStation->addLine(nLignes, previous, false);
					} // autrement c'est un cycle
				}
				else {
					newStation->addLine(nLignes, true);
				}
				if (montrer) std::cout << "  L'arrêt " << newStation->name << " existait déjà." << std::endl;				
			}
			
			if (previous) {
				previous->getLine(nLignes)->addAdjacents(NULL, newStation);
			}
			else {
				linestarts.push_back(newStation);
			}
			
			previous = newStation;
		}
	}
		
	fichier.close();
	return 0;
}

int Ecriture(std::string filename, std::list<Station *> sta, std::vector<Station *> linestarts) {
	std::ofstream fichier;
	bool cycleTermine;
	int numArrets;
	std::vector<std::string> arretsDUneLigne;

	std::cout << "Ecriture du fichier output: " << filename << std::endl;
	
	fichier.open(filename.c_str());
	
	fichier << linestarts.size() << std::endl;
	for (int i = 1; i <= linestarts.size(); ++i) {
		// Compter le numero d'arrets
		cycleTermine = false;
		numArrets = 0;
		Station * st = linestarts[i-1];
		while (not cycleTermine) {
			arretsDUneLigne.push_back(st->name);
			numArrets ++;
			if ( st->getLine(i)->getSuccessor() ) { 
				if (st->getSuccessor(i)->getLine(i)) {

					if ( st->getSuccessor(i)->getLine(i)->isDebutDeLigne() ) {
						cycleTermine = true;
						arretsDUneLigne.push_back(st->getSuccessor(i)->name);
					}
				}
			}
			else {
				cycleTermine = true;
			}
			if (not cycleTermine) {
				if (st->getLine(i)->getSuccessor()) st = st->getLine(i)->getSuccessor();
			}
		}
		fichier << numArrets << std::endl;
		
		// Lister tous les noms d'arret
		while (arretsDUneLigne.size()) {
			fichier << arretsDUneLigne.front() << std::endl;
			pop_front(arretsDUneLigne);				
		} 
	}

	fichier.close();
	
	return 0;	
}
