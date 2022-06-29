#ifndef OPTIM_ALG
#define OPTIM_ALG

#include <ogdf/basic/GridLayout.h>
#include "edgeMap.hpp"
#include "nodeMap.hpp"
#include <chrono>
#include <cmath>
#include <fstream>

using ogdf::GridLayout;
using ogdf::node;
using ogdf::IPolyline;
using ogdf::IPoint;
using ogdf::ListIterator;

//renvoie n fois la variance étant données :
// - la somme des longueurs des edge dans sommeLong
// - la somme des longeurs des edge au carré dans sommeLong2
double calcNVar(double sommeLong, double sommeLong2) {
	return mapEdgeLength.size() * sommeLong2 - sommeLong * sommeLong;
}

//calcule la somme des longueurs des edge dans sommeLong
//calcule la somme les longueurs des edge au carre dans sommeLong2
//calcule n fois la variance dans nVar
void prepCalcNVar(double& sommeLong, double& sommeLong2, double& nVar) {
	sommeLong = 0;
	sommeLong2 = 0;
	for (auto it = mapEdgeLength.begin(); it != mapEdgeLength.end(); it++) {
		sommeLong += it->second;
		sommeLong2 += it->second * it->second;
	}
	nVar = calcNVar(sommeLong, sommeLong2);
}

//prend en parametre une longueur d'un edge et la soustrait de sommeLong et sommeLong2
void deleteEdgeNVar(double edgeLength, double& sommeLong, double& sommelong2) {
	sommeLong -= edgeLength;
	sommelong2 -= edgeLength*edgeLength;
}

//prend en paramètre une longueur d'un edge et l'ajoute à sommeLong et someLong2
void addEdgeNVar(double edgeLength, double& sommeLong, double& sommeLong2) {
	sommeLong += edgeLength;
	sommeLong2 += edgeLength * edgeLength;
}

// Ecris dans le CSV pour un double
void writeCsvDouble(string nom, double data, double variance) {
	std::ofstream csv(nom, std::ios::app);
	if (csv.is_open()) {
		csv << data;
		csv << ",";
		csv << variance;
		csv << '\n';
		csv.close();
	}
}

// Ecris dans le CSV pour un Unsigned long long
void writeCsvULL(string nom, unsigned long long data, double variance) {
	std::ofstream csv(nom, std::ios::app);
	if (csv.is_open()) {
		csv << data;
		csv << ",";
		csv << variance;
		csv << '\n';
		csv.close();
	}
}

// Ecris le nombre de tour et la variance dans un fichier tout les intervalle de tours
void checkTour(unsigned long long& nbTour, unsigned long long& lastWrittenTour, int intervalle, double variance, bool forceWrite) {
	nbTour++;
	if ((nbTour - lastWrittenTour > intervalle)||forceWrite) {
		lastWrittenTour = nbTour;
		writeCsvULL("dataTurn.csv", nbTour, variance);
	}
}

// Ecris le temps d'exécution et la variance dans un fichier csv tout les intervalle de temps
void checkTime(std::chrono::system_clock::time_point start, std::chrono::system_clock::time_point& lastWritten, double intervalle, double variance, bool forceWrite) {
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> secondsBetweenWrites = end - lastWritten;
	if ((secondsBetweenWrites.count() > intervalle)||(forceWrite)) {
		lastWritten = end;
		std::chrono::duration<double> secondsTotal = end - start;
		writeCsvDouble("dataTime.csv", secondsTotal.count(), variance);
	}
}

// Fonction qui modifie le coefficient de recuit simulé, le nombre de tour et le booleen qui indique la phase du recuit
void modifCoeffRecuit(double& coeff, double coeffDesc, double coeffMont, double coeffMax, double coeffMin, bool& recuitMontant, int& nbTour, int nbTourModifCoeff) {
	nbTour++;
	// Si assez de tour sont passé on décalle le coefficient
	if (nbTour >= nbTourModifCoeff) {
		nbTour = 0;
		// Si la phase est montante on augmente le coefficient
		if (recuitMontant) {
			if (coeff < 1.0) {
				coeff += coeffDesc;
			}
			else {
				coeff += coeffMont;
			}
		}
		else {
			if (coeff <= 1.0) {
				coeff -= coeffDesc;
			}
			else {
				coeff -= coeffMont;
			}
		}
		// Si on atteind le max ou le min, on change de phase
		double epsilon = 0.05;
		if ((coeff > coeffMax - epsilon) || (coeff < coeffMin + epsilon)) {
			recuitMontant = !recuitMontant;
		}
	}
}


#endif OPTIM_ALG