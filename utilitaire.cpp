#include "utilitaire.hpp"
#include <random>
#include <algorithm>
#include "emplacement.hpp"
#include <iostream>
#include <cstring>
#include <omp.h>
#include "personnel.hpp"

using std::min;
using std::max;

std::vector<std::mt19937*> genVector;
thread_local int numGen=0;

bool isSeedRandom;
unsigned int seed;
std::vector<unsigned int> seedThread;
std::vector<bool> resetSeedThread;

std::string typeSeed;

// Retourne une valeur réelle comprise dans [0.0,n[
double generateDoubleRand(double n) {
    std::uniform_real_distribution<> dis(0.0, n);
#if defined(OPENMP_INSTALLED)
    return dis(*genVector[omp_get_thread_num()]);
#else
    return dis(*genVector[numGen]);
#endif
}

// Retourne une valeur entiere comprise dans [0,n]
int generateRand(int n) {
    std::uniform_int_distribution<> dis(0, n);
#if defined(OPENMP_INSTALLED)
    return dis(*genVector[omp_get_thread_num()]);
#else
    return dis(*genVector[numGen]);
#endif
}

void initSameSeed(unsigned int n, bool resetting) {
    std::cout << "---------- SEED FIXE: " << n << " ----------\n";
    typeSeed = "FIXE";
    isSeedRandom = false;
    seed = n;
    genVector.clear();
    int maxThread = omp_get_max_threads();
    for (int i=0;i<maxThread;i++) {
        genVector.push_back(new std::mt19937(n));
        seedThread.push_back(n);
        resetSeedThread.push_back(resetting);
    }
}

void initSameSeedIncThread(unsigned int n,bool resetting) {
    std::cout << "---------- SEED FIXE INC: " << n << " ----------\n";
    typeSeed = "FIXE INC";
    isSeedRandom = false;
    seed = n;
    genVector.clear();
    int maxThread = omp_get_max_threads();
    for (int i=0;i<maxThread;i++) {
        genVector.push_back(new std::mt19937(n+i));
        seedThread.push_back(n+i);
        resetSeedThread.push_back(resetting);
    }
}

void initRandomSeed(bool resetting) {
    typeSeed = "RANDOM";
    isSeedRandom = true;
    genVector.clear();
    int maxThread = omp_get_max_threads();
    for (int i=0;i<maxThread;i++) {
        std::random_device rd;
        unsigned int tmpSeed = rd();
        genVector.push_back(new std::mt19937(tmpSeed));
        seedThread.push_back(tmpSeed);
        resetSeedThread.push_back(resetting);
    }
    std::cout << "---------- SEED RANDOM: " << getSeed(0) << " ----------\n";
}

void resetSeed(int numThread, bool resetSameSeed, bool forceReset) {
    numGen = numThread;
    if (resetSeedThread[numThread]||forceReset) {
        if (isSeedRandom) {
            if (resetSameSeed) {
                genVector[numThread] = new std::mt19937(seedThread[numThread]);
            }
            else {
                std::random_device rd;
                unsigned int tmpSeed = rd();
                genVector[numThread] = new std::mt19937(tmpSeed);
                seedThread[numThread] = tmpSeed;
            }
        }
        else {
            genVector[numThread] = new std::mt19937(seed);
        }
    }
}

std::string getTypeSeed() {
    return typeSeed;
}

unsigned int getSeed(int tid) {
    return seedThread[tid];
}

bool isSeedFixe() {
    return !isSeedRandom;
}

bool isSeedResetting(int numThread) {
    return resetSeedThread[numThread];
}

// Indique si une valeur est dans un vecteur ou non
bool isInVector(std::vector<int>& vectorInt, int x) {
    if (std::find(vectorInt.begin(),vectorInt.end(),x) != vectorInt.end())
        return true;
    return false;
}

bool isInVector(std::vector<std::string>& vectorString, std::string x) {
    if (std::find(vectorString.begin(),vectorString.end(),x) != vectorString.end())
        return true;
    return false;
}

// Le vecteur doit etre trie avant l'appel de cette fonction
long medianeVector(std::vector<int> &vec) {
    int taille = vec.size();
    if (taille % 2 == 1) {
        return vec[taille / 2];
    }
    else {
        int nbCroisement1 = vec[taille / 2];
        int nbCroisement2 = vec[(taille / 2) - 1];

        return ((nbCroisement1 + nbCroisement2) / 2.0);
    }
}

int lowestInVector(std::vector<int> &vec) {
    int lowest = vec[0];
    for (int i=1;i<vec.size();i++) {
        if (vec[i] < lowest) {
            lowest = vec[i];
        }
    }
    return lowest;
}

bool comparePtrEmplacement(Emplacement* a, Emplacement* b) { return (a->getX() < b->getX()); }

bool comparePtrEmplacementTri(Emplacement* a, Emplacement*b) {
    if (a->getX() == b->getX()) {
        return (a->getY() < b->getY());
    }
    return (a->getX() < b->getX());
}

void removeFromVector(std::vector<int>& vec, int x) {
    for (int i=0;i<vec.size();i++) {
        if (vec[i] == x) {
            vec[i] = vec[vec.size()-1];
            vec.pop_back();
            return;
        }
    }
}

int nombreIterationRecuit(double t,double cool,double seuil) {
    return (int)(((log(seuil/t)))/(log(cool)));
}

bool containsString(std::string a, std::string b) {
    std::transform(a.begin(), a.end(), a.begin(),
    [](unsigned char c){ return std::tolower(c); });
    std::transform(b.begin(), b.end(), b.begin(),
    [](unsigned char c){ return std::tolower(c); });
    return strstr(a.c_str(),b.c_str());
}