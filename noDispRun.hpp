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

// Fait tourner l'algo sélectionné tant qu'il n'a pas une variance de 1 sans affichage openGL
// i==0 rouletteRusse i==1 recuit simulé
void runAlgo(int i, Graph& G, GridLayout& GL, const int gridWidth, const int gridHeight, int maxX, int maxY, int maxBends) {

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

	//debut ogdf
	node n = G.firstNode();
	ConstCombinatorialEmbedding CCE = ConstCombinatorialEmbedding{ G };
	double sommeLong = 0, sommeLong2 = 0, variance = 0;
	prepCalcNVar(sommeLong, sommeLong2, variance);
	double bestVariance = variance;

	// Chrono pour le temps d'exec, utilisé pour le stockage de donnée pour la création de graphiques, a supprimer lors de vrai executions
	auto start = std::chrono::system_clock::now();
	auto lastWritten = std::chrono::system_clock::now();
	// NB tour pour le stockage de donnée pour les graphiques, a supprimer lors de vrai executions
	unsigned long long totalTurn = 0;
	unsigned long long lastWrittenTurn = 0;

	// Parametre pour le recuit simulé
	double coeff = 1.0;
	// Decallage coeff descendant, on soustrait cette valeur
	double coeffDesc = 0.1;
	// Decallage coeff montant, on ajoute cette valeur
	double coeffMont = 0.5;
	// Max et Min du coeff
	double coeffMax = 5;
	double coeffMin = 0.1;
	// Indique si on est sur la vague montante du recuit simulé
	bool recuitMontant = false;
	// Nombre d'execution du recuit simule
	int nbTour = 0;
	// Nombre d'execution requise pour modifier le coeff
	int nbTourModifCoeff = 100;
	// Numéro du dernier NodeBend déplacé pour l'algo bestVariance
	int numLastMoved = -1;
	int numCourant = 0;

	// Utilisé pour l'algo mixte
	int nbTourDepuisBestVar = 0;

	// On ecris les données de départ
	writeCsvULL("dataTurn.csv", nbTour, variance);
	writeCsvDouble("dataTime.csv", 0, variance);

	int width, height;
	while (calcEdgeLengthRatio() > 1.00005) {
		float ratio;
		// Roulette russe
		if (i == 0) {
			startRouletteRusse(GL, CCE, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
		}
		// Recuit simulé
		else if (i == 1) {
			startRecuitSimule(coeff, GL, CCE, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
			modifCoeffRecuit(coeff, coeffDesc, coeffMont, coeffMax, coeffMin, recuitMontant, nbTour, nbTourModifCoeff);
		}
		// Best Variance
		else if (i == 2) {
			if (numLastMoved == numCourant) {
				break;
			}
			startBestVariance(GL, CCE, numCourant, numLastMoved, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
			numCourant = (numCourant + 1) % vectorNodeBends.size();
		}
		// Mixe recuit et bestVariance
		else if (i == 10) {
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
				numCourant = (numCourant + 1) % vectorNodeBends.size();
			}
		}
		// Sauvegarde du nouveau meilleur graphe
		if (variance < bestVariance) {
			bestVariance = variance;
			writeToJson("bestResult.json", G, GL, gridWidth, gridHeight, maxBends);
			//checkTime(start, lastWritten, 10, variance,true);
			checkTour(totalTurn, lastWrittenTurn, 10000, variance,true);
		}
		else {
			checkTime(start, lastWritten, 10, variance,false);
			checkTour(totalTurn, lastWrittenTurn, 10000, variance,false);
		}
	}
}

#endif