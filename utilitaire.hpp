#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <random>
#include "emplacement.hpp"

static cpu_set_t cpuset;

// Retourne une valeur réelle comprise dans [0.0,n[
double generateDoubleRand(double n);

// Retourne une valeur entiere comprise dans [0,n]
int generateRand(int n);

// Indique si une valeur est dans un vecteur ou non
bool isInVector(std::vector<int>& vectorInt, int x);

// Indique si une valeur est dans un vecteur ou non
bool isInVector(std::vector<std::string>& vectorString, std::string x);

// Enleve x du vecteur vectorInt
void removeFromVector(std::vector<int>& vectorInt, int x);

template<typename T>
double moyenneVector(std::vector<T>& vec) {
	double moyenne = 0.0;
	for (const double& elem : vec) {
		moyenne += elem;
	}
	moyenne = moyenne / (double)vec.size();
	return moyenne;
}

// Le vecteur doit etre trie avant l'appel de cette fonction
long medianeVector(std::vector<int> &vec);

int lowestInVector(std::vector<int> &vec);

template<typename T>
double ecartTypeVector(const std::vector<T>& vec, double moyenne) {
    double variance = 0.0;
    for (const auto& elem : vec) {
        variance += (elem - moyenne)*(elem - moyenne);
    }
    variance /= vec.size();
    return std::sqrt(variance);
}

// Fonction de sort par x
bool comparePtrEmplacement(Emplacement* a, Emplacement* b);

// Fonction de sort par x et y
bool comparePtrEmplacementTri(Emplacement* a, Emplacement*b);

// n la seed (0 par défaut), resetting indique si la seed doit être réinitialisée lors de l'appel de resetSeed
void initSameSeed(unsigned int n=0,bool resetting=true);

// n la seed(0 par défaut), incrémente la seed du tid, chaque thread a une seed différente. resetting indique si la seed doit être réinitialisée lors de l'appel de resetSeed
void initSameSeedIncThread(unsigned int n=0,bool resetting=true);

// resetting indique si la seed doit être réinitialisée lors de l'appel de resetSeed
void initRandomSeed(bool resetting=false);

// Reset la seed du thread numThread, resetSameSeed indique si la seed doit avoir la même valeur ou non. forceReset réinitialise peut importe la valeur de resetting.
void resetSeed(int numThread, bool resetSameSeed=false, bool forceReset=false);

// Indique si la seed est aléatoire ou fixe.
std::string getTypeSeed();

int nombreIterationRecuit(double t,double cool,double seuil);

// Renvoie la seed actuellement assignée au thread tid.
unsigned int getSeed(int tid);

// Renvoie vrai si la seed est fixe.
bool isSeedFixe();

// Renvoie vrai si la seed est reset a chaque run dans logCSV
bool isSeedResetting(int numThread);

// Renvoie true si le string b est présent dans le string a
bool containsString(std::string a, std::string b);

#endif