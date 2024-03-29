#ifndef NODISPRUN_H
#define NODISPRUN_H
#include <stdio.h>
#include <ogdf/basic/GridLayout.h>
#include <ogdf/basic/CombinatorialEmbedding.h>
#include <ogdf/basic/Layout.h>
#include "calcEdgeLength.hpp"
#include "EdgeMap.hpp"
#include "graphFunctions.hpp"
#include "NodeBend.hpp"
#include "optimAlg.hpp"
#include "jsonIO.hpp"
#include <random>
#include <chrono>
#include <cmath>
#include <fstream>

// Fait tourner l'algo s�lectionn� tant qu'il n'a pas une variance de 1 sans affichage openGL
// i==0 rouletteRusse i==1 recuit simul�
void runAlgo(int i, Graph& G, GridLayout& GL, const int gridWidth, const int gridHeight, int maxX, int maxY, int maxBends, string nomGraphe) {

	if (i == 0) {
		std::cout << "Starting Roulette Russe..." << std::endl;
	}
	else if (i == 1) {
		std::cout << "Starting Recuit Simule..." << std::endl;
	}
	else if (i == 2) {
		std::cout << "Starting Best Variance..." << std::endl;
	}
	else if (i == 10) {
		std::cout << "Starting Mixte(Recuit Simule + Best Variance)..." << std::endl;
	}
	else if (i == 11) {
		std::cout << "Starting Algorithme Angle..." << std::endl;
	}

	//debut ogdf
	node n = G.firstNode();
	ConstCombinatorialEmbedding CCE = ConstCombinatorialEmbedding{ G };
	double sommeLong = 0, sommeLong2 = 0, variance = 0;
	prepCalcNVar(sommeLong, sommeLong2, variance);
	double bestVariance = variance;

	// Initialisation des vecteurs de deplacements
	initListeDeplacements();

	// Chrono pour le temps d'exec, utilis� pour le stockage de donn�e pour la cr�ation de graphiques, a supprimer lors de vrai executions
	auto start = std::chrono::system_clock::now();
	auto lastWritten = std::chrono::system_clock::now();
	// NB tour pour le stockage de donn�e pour les graphiques, a supprimer lors de vrai executions
	unsigned long long totalTurn = 0;
	unsigned long long lastWrittenTurn = 0;

	// Parametre pour le recuit simul�
	double coeff = 1.0;
	// Decallage coeff descendant, on soustrait cette valeur
	double coeffDesc = 0.1;
	// Decallage coeff montant, on ajoute cette valeur
	double coeffMont = 0.5;
	// Max et Min du coeff
	double coeffMax = 5;
	double coeffMin = 0.1;
	// Indique si on est sur la vague montante du recuit simul�
	bool recuitMontant = false;
	// Nombre d'execution du recuit simule
	int nbTour = 0;
	// Nombre d'execution requise pour modifier le coeff
	int nbTourModifCoeff = 100;
	// Num�ro du dernier NodeBend d�plac� pour l'algo bestVariance
	int numLastMoved = -1;
	int numCourant = 0;

	// Utilis� pour l'algo mixte
	int nbTourDepuisBestVar = 0;

	// On ecris les donn�es de d�part
	//writeCsvULL("dataTurn.csv", nbTour, variance);
	//writeCsvDouble("dataTime.csv", 0, variance);

	double ratio = calcEdgeLengthRatio();
	double bestRatio = ratio;
	writeGraphInfo("allData.txt", nomGraphe,gridWidth,gridHeight,maxBends,ratio,variance);
	// Roulette russe
	if (i == 0) {
		while (bestRatio > 1.00005) {
			startRouletteRusse(GL, CCE, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
			// Sauvegarde du nouveau meilleur graphe best ratio
			ratio = calcEdgeLengthRatio();
			if (ratio < bestRatio) {
				bestRatio = ratio;
				writeToJson("bestResult.json", G, GL, gridWidth, gridHeight, maxBends);
				writeAllData("allData.txt", nbTour, variance, ratio, start);
				std::cout << "New Ratio: " << bestRatio << std::endl;
			}
			nbTour++;
			if (nbTour % 10000 == 0) {
				writeAllData("allData.txt", nbTour, variance, ratio, start);
			}
		}
	}
	// Recuit simul�
	else if (i == 1) {
		int nbTourCoeff = 0;
		while (bestRatio > 1.00005) {
			startRecuitSimule(coeff, GL, CCE, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
			modifCoeffRecuit(coeff, coeffDesc, coeffMont, coeffMax, coeffMin, recuitMontant, nbTourCoeff, nbTourModifCoeff);
			// Sauvegarde du nouveau meilleur graphe best ratio
			ratio = calcEdgeLengthRatio();
			if (ratio < bestRatio) {
				bestRatio = ratio;
				writeToJson("bestResult.json", G, GL, gridWidth, gridHeight, maxBends);
				writeAllData("allData.txt", nbTour, variance, ratio, start);
				std::cout << "New Ratio: " << bestRatio << std::endl;
			}
			nbTour++;
			if (nbTour % 10000 == 0) {
				writeAllData("allData.txt", nbTour, variance, ratio, start);
			}
		}
	}
	// Best Variance
	else if (i == 2) {
		while (bestRatio > 1.00005) {
			if (numLastMoved == numCourant) {
				break;
			}
			startBestVariance(GL, CCE, numCourant, numLastMoved, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
			numCourant = (numCourant + 1) % (int)vectorNodeBends.size();
			// Sauvegarde du nouveau meilleur graphe best ratio
			ratio = calcEdgeLengthRatio();
			if (ratio < bestRatio) {
				bestRatio = ratio;
				writeToJson("bestResult.json", G, GL, gridWidth, gridHeight, maxBends);
				writeAllData("allData.txt", nbTour, variance, ratio, start);
				std::cout << "New Ratio: " << bestRatio << std::endl;
			}
			nbTour++;
			if (nbTour % 10000 == 0) {
				writeAllData("allData.txt", nbTour, variance, ratio, start);
			}
		}
	}
	// Mixe recuit et bestVariance
	else if (i == 10) {
		while (bestRatio > 1.00005) {
			if (nbTourDepuisBestVar < 5000) {
				startRecuitSimule(coeff, GL, CCE, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
				modifCoeffRecuit(coeff, coeffDesc, coeffMont, coeffMax, coeffMin, recuitMontant, nbTour, nbTourModifCoeff);
				nbTourDepuisBestVar++;
			}
			else {
				if (numLastMoved == numCourant) {
					nbTourDepuisBestVar = 0;
				}
				startBestVariance(GL, CCE, numCourant, numLastMoved, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
				numCourant = (numCourant + 1) % (int)vectorNodeBends.size();
			}
			// Sauvegarde du nouveau meilleur graphe best ratio
			ratio = calcEdgeLengthRatio();
			if (ratio < bestRatio) {
				bestRatio = ratio;
				writeToJson("bestResult.json", G, GL, gridWidth, gridHeight, maxBends);
				writeAllData("allData.txt", nbTour, variance, ratio, start);
				std::cout << "New Ratio: " << bestRatio << std::endl;
			}
			nbTour++;
			if (nbTour % 10000 == 0) {
				writeAllData("allData.txt", nbTour, variance, ratio, start);
			}
		}
	}
	// Algo angle
	else if (i == 11) {
		int nbretour = 0;
		while (nbretour < 10) {
			nbTour = 0;
			while (nbTour < 20000000) {
				startSingleRecuitSimuleAngle(GL, CCE, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
				nbTour++;
				if ((nbTour % 100000) == 0) {
					std::cout << nbretour << " tour: " << nbTour << std::endl;
				}
			}
			nbretour++;
		}
		writeToJson("bestResult.json", G, GL, gridWidth, gridHeight, maxBends);
	}
}

#endif