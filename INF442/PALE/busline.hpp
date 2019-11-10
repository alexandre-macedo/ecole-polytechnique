	#pragma once

#include <string>
#include <vector>
#include <list>
#include <algorithm>

class Station;

class Pos {
	private:
		bool debutDeLigne;
		Station * nextEast;
		Station * nextWest;

	public:
		int idLine;
	
		Station * getSuccessor();
		Station * getPredecessor();
		void setDebutDeLigne();
		void unsetDebutDeLigne();
		bool isDebutDeLigne();
		Pos(int line, Station * nextEast, Station * nextWest, bool debut);
		~Pos();
		
		void addAdjacents( Station * nextEast, Station * nextWest);
		void removeSuccessor();
		void removePredecessor();
};

class Station {
	private:
		void constructeur(std::string name, int line, Station * previous, Station * next, bool debut);
		int nLines;
	public:
		std::string name;

		std::vector <Pos *> connectedLines;

		Station * getPredecessor(int line);
		Station * getSuccessor(int line);

		int howManyLines();
			
		int addLine(int line, Station * previous, Station * next, bool debut);
		int addLine(int line, Station * previous, bool debut);
		int addLine(int line, bool debut);
		
		int removeLine(int line);

		Pos * getLine(int line);

		Station(std::string name, int line, Station * previous, Station * next);
		Station(std::string name, int line, Station * previous);
		Station(std::string name, int line, bool debut);
		Station(std::string name, int line);

		~Station();
};

Station * getStationByName(std::string nameStation, std::list <Station *> sta);

int Effacer(Station * st, std::list <Station *> &sta, std::vector <Station *> &linestarts);
Station * Inserer(std::string newStName, int line, Station * arr, std::list <Station *> &sta, std::vector <Station *> &linestarts);
Station * Inserer(std::string newStName, int line, Station * arr1, Station * arr2, std::list <Station *> &sta, std::vector <Station *> &linestarts);

int MontrerArret(Station * st);
int MontrerArrets(std::list<Station *> sta);
int MontrerLigne(std::vector<Station *> debut, int ligne);
int MontrerLignes(std::vector<Station *> debut);
int MontrerLignes(std::vector<Station *> debut, std::vector<int> lignes);
int Lecture(std::string filename, std::list<Station *> &sta, std::vector<Station *> &linestarts, bool montrer);
int Ecriture(std::string filename, std::list<Station *> sta, std::vector<Station *> linestarts);

bool exists_file (const std::string& name);

class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }
        /// @author iain
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            return empty_string;
        }
        /// @author iain
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector <std::string> tokens;
        std::string empty_string;
};
