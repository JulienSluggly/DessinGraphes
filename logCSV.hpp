#ifndef LOGCSV_HPP
#define LOGCSV_HPP

#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include "ogdfFunctions.hpp"
#include "personnel.hpp"
#include <omp.h>
#include <climits>
#include <ctime>
#include <algorithm>

std::unordered_map<std::string,std::pair<int,int>> mapGraphPopGen;

void fillMap() {
	mapGraphPopGen["graph-1-input"] = std::pair<int,int>(1000,1000);
	mapGraphPopGen["graph-2-input"] = std::pair<int,int>(1000,1000);
	mapGraphPopGen["graph-3-input"] = std::pair<int,int>(1000,1000);
	mapGraphPopGen["graph-4-input"] = std::pair<int,int>(1000,1000);
	mapGraphPopGen["graph-5-input"] = std::pair<int,int>(1000,200);
	mapGraphPopGen["graph-6-input"] = std::pair<int,int>(500,100);
	mapGraphPopGen["graph-7-input"] = std::pair<int,int>(750,100);
	mapGraphPopGen["graph-8-input"] = std::pair<int,int>(70,100);
	mapGraphPopGen["graph-9-input"] = std::pair<int,int>(70,100);
	mapGraphPopGen["graph-10-input"] = std::pair<int,int>(20,50);
	mapGraphPopGen["graph-11-input"] = std::pair<int,int>(10,30);
	mapGraphPopGen["graph-12-input"] = std::pair<int,int>(10,30);
}

void fillLogsVector() {
	fillMap();
}

std::string getParamAsString(std::vector<std::vector<double>>& customParam) {
	std::stringstream paramStream;
	paramStream << std::fixed;
	paramStream << "{";
	for (std::vector<double>& param : customParam) {
		paramStream << "{";
		if (param.size() > 0) {
			for (int i=0;i<param.size();i++) {
				if (i==0) { paramStream << std::setprecision(0) << param[i]; }
				else { 
					if (param[i] > 1) { paramStream << std::setprecision(2) << param[i]; }
					else { paramStream << std::setprecision(9) << param[i]; }
				}
				if (i<param.size()-1) { paramStream << " "; }
			}
		}
		paramStream << "}";
	}
	paramStream << "}";
	return paramStream.str();
}

void saveGrapheDistanceCSV(Graphe& G, int tid) {
	if (G.recuitDistanceAll.size() > 0) {
		std::string nomFichier = chemin + "/resultats/" + to_string(tid) + "-recuitDistAll.csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);
		for (int i=0;i<G.recuitDistanceAll.size();i+=100) {
			resultats << i << "," << G.recuitDistanceAll[i] << std::endl;
		}
		resultats.close();
	}
	if (G.recuitDistanceUpgrade.size() > 0) {
		std::string nomFichier = chemin + "/resultats/" + to_string(tid) + "-recuitDistUp.csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);
		for (int i=0;i<G.recuitDistanceUpgrade.size();i+=100) {
			resultats << G.recuitDistanceUpgrade[i].first << "," << G.recuitDistanceUpgrade[i].second << std::endl;
		}
		resultats.close();
	}
	if (G.recuitDistanceUpgradeGlobal.size() > 0) {
		std::string nomFichier = chemin + "/resultats/" + to_string(tid) + "-recuitDistUpG.csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);
		for (int i=0;i<G.recuitDistanceUpgradeGlobal.size();i++) {
			resultats << G.recuitDistanceUpgradeGlobal[i].first << "," << G.recuitDistanceUpgradeGlobal[i].second << std::endl;
		}
		resultats.close();
	}
}

void generateCSV(int nbEssay, const std::string& methodePlacementName, const std::string& methodeAlgoName, std::string fileGraph, std::string fileSlots, std::vector<std::vector<double>> customParam={{}}, bool useReel=false, int tid=0) {
	string nomGraphe = fileGraph;
	std::reverse(nomGraphe.begin(), nomGraphe.end());
	nomGraphe = nomGraphe.substr(nomGraphe.find(".") + 1);
	nomGraphe = nomGraphe.substr(0, nomGraphe.find('/'));
	std::reverse(nomGraphe.begin(), nomGraphe.end());
	bool updateScore = containsString(methodeAlgoName,"Score");
	bool isGenetique = containsString(methodeAlgoName,"Genetique");
	bool isRecuit = containsString(methodeAlgoName,"Recuit");
	bool needTriangulation = containsString(methodeAlgoName,"TRE");
	bool needGrille = containsString(methodeAlgoName,"Grille");
	double moyenneCroisement, medianCroisement, ecartTypeCroisement;
	int meilleurCroisement = INT_MAX;
	int nbSolutionIllegale = 0, debugValue=-1;
	std::vector<int> croisementVector, edgeCostVector;
	std::vector<double> tempExecVector, tempBestVector, tempsPlacementVector; std::vector<int> bestIterationVector, lastIterationVector, nombreRecuitVector, totalInterVector, totalInterIllVector, placementInterVector;
	double tempsExecMoyenne, tempsBestMoyenne, tempsPlacementMoyenne, bestIterationMoyenne, lastIterationMoyenne, nombreRecuitMoyenne, totalInterMoyenne, totalInterIllMoyenne, edgeCostMoyenne, placementInterMoyenne;
	bool saveResult = true;
	int population, maxIteration;
	int nombreRecuit=0, nombreSlots, nombreCellule;
	auto totalStart = std::chrono::system_clock::now();
	std::chrono::duration<double> secondsTotalExec = totalStart - totalStart;
	for (int i = 1; ((((i <= nbEssay)&&(secondsTotalExec.count() < 3600))||(nbEssay==-1&&secondsTotalExec.count() < 3600))&&(i <= 100)); ++i) {
		resetSeed(tid,true);
		auto start = std::chrono::system_clock::now();
		Graphe G(nomGraphe);
		if (!useReel) { G.setupGraphe(fileGraph,fileSlots); }
		else { G.setupGrapheReel(fileGraph); G.useCoordReel = true; }
		G.fillCommonNodeVectors();
		double tempsBest = -1; int bestIteration = -1; int lastIteration = -1;
		if (isGenetique) { population = mapGraphPopGen[nomGraphe].first; maxIteration = mapGraphPopGen[nomGraphe].second; }
		saveResult = true;
		printf("Tid: %d | Iter: %d Max: %d | %s | %s | %s | Slots: %lu | Nodes: %lu | TotalRun: %.1fs | Seed: %u | Param: %s\n",tid,i,nbEssay,nomGraphe.c_str(),methodePlacementName.c_str(),methodeAlgoName.c_str(),G._emplacements.size(),G._noeuds.size(),secondsTotalExec.count(),getSeed(tid),getParamAsString(customParam).c_str());
		if (methodePlacementName == "Glouton") { if (!useReel) { G.glouton(); } }
		else if (methodePlacementName == "Glouton Revisite") { if (!useReel) { G.gloutonRevisite(); } }
		else if (methodePlacementName == "Glouton Gravite") { if (!useReel) { G.gloutonRevisiteGravite(); } }
		else if (methodePlacementName == "Glouton Voisin") { if (!useReel) { G.gloutonRevisiteVoisin(); } }
		else if (methodePlacementName == "OGDF") { 
#if defined(OGDF_INSTALLED)
			if (!useReel) { ogdfPlacementAuPlusProche(G); }
#else
			std::cout << "OGDF NOT INSTALLED.\n";
			return;
#endif
		}
		else if (methodePlacementName == "OGDFFMMM") {
#if defined(OGDF_INSTALLED)
			if (!useReel) { ogdfFastMultipoleMultilevelEmbedder(G); } else { ogdfFastMultipoleMultilevelEmbedderReel(G); }
#else
			std::cout << "OGDF NOT INSTALLED.\n";
			return;
#endif			
		}
		else if (methodePlacementName == "OGDFFMMMM") {
#if defined(OGDF_INSTALLED)
			if (useReel) { ogdfFastMultipoleMultilevelEmbedderReelMinute(G); }
#else
			std::cout << "OGDF NOT INSTALLED.\n";
			return;
#endif		
		}
		else if (methodePlacementName == "Stress") { if (!useReel) { G.stressMajorization(customParam); } else { G.stressMajorizationReel(); } }
		else if (methodePlacementName == "Stress Dyn Stress") { if (!useReel) { G.stressMajorization(customParam,1); } }
		else if (methodePlacementName == "Stress Dyn Cross") { if (!useReel) { G.stressMajorization(customParam,2); } }
		else if (methodePlacementName == "Glouton Grille") { if (!useReel) { G.gloutonRevisiteGrid(); } }
		else if (methodePlacementName == "Aleatoire") { if (!useReel) { G.placementAleatoire(); } else { G.placementAleatoireReel(); } }
		else if (methodePlacementName != "Aucun") {
			std::cout << "ERROR Aucune methode " << methodePlacementName << " trouve !\n";
			return;
		}
		sched_setaffinity(0, sizeof(cpuset), &cpuset);

		if (useReel) { G.translateGrapheToOriginReel(-1); }
		if (updateScore) { G.initGraphAndNodeScoresAndCrossings(); }
		if (needTriangulation) { G.triangulationDelaunay(); }
		if (needGrille) { G.setupGridAndRegistration(customParam); }

		auto finPlacement = std::chrono::system_clock::now();
		if (methodePlacementName != "Aucun") { if (!useReel) { placementInterVector.push_back(G.getNbCroisementDiff()); } else { placementInterVector.push_back(G.getNbCroisementDiffReel()); } }

		if (methodeAlgoName == "Recuit Simule") { if (!useReel) { G.recuitSimule(tempsBest,start,customParam,0.99999,100.0,0.0001,1,0,0,false,false); } else { G.recuitSimuleReel(tempsBest,start,customParam,0.99999,100.0,0.0001,1,0,0,false,false,false); } }
		if (methodeAlgoName == "Recuit Simule Grille") { if (!useReel) { G.recuitSimule(tempsBest,start,customParam,0.99999,100.0,0.0001,1,0,0,true,false); } else { G.recuitSimuleReel(tempsBest,start,customParam,0.99999,100.0,0.0001,1,0,0,true,false,false); } }
		else if (methodeAlgoName == "Recuit Simule Grille TME") { if (!useReel) { G.recuitSimule(tempsBest,start,customParam); } else { G.recuitSimuleReel(tempsBest,start,customParam); } }
		else if (methodeAlgoName == "Recuit Simule Grille BOX") { if (useReel) { G.recuitSimuleReel(tempsBest,start,customParam,0.99999,100.0,0.0001,1,0,4,true); } }
		else if (methodeAlgoName == "Rerecuit Simule Grille TME")  { if (!useReel) { G.rerecuitSimule(tempsBest,nombreRecuit,start,customParam); } else { G.rerecuitSimuleReel(tempsBest,nombreRecuit,start,customParam); } }
		else if (methodeAlgoName == "Rerecuit Simule Grille BOX") { if (useReel) { G.rerecuitSimuleReel(tempsBest,nombreRecuit,start,customParam,-1,0.99999,0.99,100.0,0.0001,1,0,4,true,false,false); } }
		else if (methodeAlgoName == "Rerecuit Simule Grille TME Temp") G.rerecuitSimule(tempsBest,nombreRecuit,start,customParam,-1,0.99999,0.99,0.1);
		else if (methodeAlgoName == "Rerecuit Simule Grille TME Cool") if (!useReel) { G.rerecuitSimule(tempsBest,nombreRecuit,start,customParam,-1,0.999999); } else { G.rerecuitSimuleReel(tempsBest,nombreRecuit,start,customParam,-1,0.999999); }
		else if (methodeAlgoName == "Rerecuit Simule Grille TME Cooler") G.rerecuitSimule(tempsBest,nombreRecuit,start,customParam,-1,0.9999999);
		else if (methodeAlgoName == "Rerecuit Simule Grille TME Cool Delay") G.rerecuitSimule(tempsBest,nombreRecuit,start,customParam,-1,0.999999,0.99,100.0,0.0001,2);
		else if (methodeAlgoName == "Rerecuit Simule Grille TME Cool Delay Temp") G.rerecuitSimule(tempsBest,nombreRecuit,start,customParam,-1,0.999999,0.99,0.1,0.0001,2);
		else if (methodeAlgoName == "Best Deplacement") G.bestDeplacement();
		else if (methodeAlgoName == "Genetique Recuit") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population, maxIteration, fileGraph, fileSlots, true);
		else if (methodeAlgoName == "Genetique Recuit Random") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population, maxIteration, fileGraph, fileSlots, true, true);
		else if (methodeAlgoName == "Genetique") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population, maxIteration, fileGraph, fileSlots);
		else if (methodeAlgoName == "Genetique Random") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population, maxIteration, fileGraph, fileSlots, false, true);
		else if (methodeAlgoName == "Genetique Score") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population,maxIteration,fileGraph,fileSlots,false,false,3);
		else if (methodeAlgoName == "Genetique Score Recuit") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population,maxIteration,fileGraph,fileSlots,true,false,3);
		else if (methodeAlgoName == "Genetique Enfant") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population,maxIteration,fileGraph,fileSlots,false,false,5);
		else if (methodeAlgoName == "Genetique Enfant Recuit") G.grapheGenetique(tempsBest,bestIteration,lastIteration, population,maxIteration,fileGraph,fileSlots,true,false,5);
		else if (methodeAlgoName != "Aucun") {
			std::cout << "ERROR Aucun algo " << methodeAlgoName << " trouve !\n";
			return;
		}

		auto end = std::chrono::system_clock::now();
		saveGrapheDistanceCSV(G,tid);
		std::chrono::duration<double> secondsTotal = end - start;
		std::chrono::duration<double> secondsPlacement = finPlacement - start;
		secondsTotalExec = end - totalStart;
		tempExecVector.push_back(secondsTotal.count());
		tempBestVector.push_back(tempsBest);
		tempsPlacementVector.push_back(secondsPlacement.count());
		bestIterationVector.push_back(bestIteration);
		lastIterationVector.push_back(lastIteration);
		nombreRecuitVector.push_back(nombreRecuit);
		nombreSlots = G._emplacements.size();
		nombreCellule = G.grillePtr.size();
		if (G.isNombreCroisementUpdated) { croisementVector.push_back(G.nombreCroisement); }
		else { if (!useReel) { croisementVector.push_back(G.getNbCroisementConst()); } else { croisementVector.push_back(G.getNbCroisementReelConst()); } }
		if (G.hasIllegalCrossing()) { nbSolutionIllegale++; }
		if (G._sm.G != nullptr) { edgeCostVector.push_back(G._sm.m_edgeCosts); }
		if (!useReel) { totalInterVector.push_back(G.getNbCroisementDiff()); } else { totalInterVector.push_back(G.getNbCroisementDiffReel()); }
		totalInterIllVector.push_back(G.nombreInterIll + G.nombreInterIllSelf);
		debugValue = G.debugEverything();
		if (debugValue > 0) { break; }
	}
	resetSeed(tid,false,true);
	if (saveResult) {
		std::sort(croisementVector.begin(), croisementVector.end());
		meilleurCroisement = croisementVector[0];
		moyenneCroisement = moyenneVector(croisementVector);
		medianCroisement = medianeVector(croisementVector);
		//ecartTypeCroisement = ecartTypeVector(croisementVector,moyenneCroisement);
		tempsPlacementMoyenne = moyenneVector(tempsPlacementVector);
		tempsExecMoyenne = moyenneVector(tempExecVector);
		tempsBestMoyenne = moyenneVector(tempBestVector);
		bestIterationMoyenne = moyenneVector(bestIterationVector);
		lastIterationMoyenne = moyenneVector(lastIterationVector);
		nombreRecuitMoyenne = moyenneVector(nombreRecuitVector);
		totalInterMoyenne = moyenneVector(totalInterVector);
		totalInterIllMoyenne = moyenneVector(totalInterIllVector);
		edgeCostMoyenne = moyenneVector(edgeCostVector);
		placementInterMoyenne = moyenneVector(placementInterVector);

		std::string nomFichier = chemin + "/resultats/" + nomGraphe + to_string(tid) + "-" + to_string(nombreSlots) + ".csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);

		char date[80];
#if defined(LINUX_OS)
		time_t t = time(0);
    	strftime(date, 80, "%d/%m/%Y", localtime(&t));
#endif

		resultats << std::fixed;
		resultats << methodePlacementName << "," // ALGO PLACEMENT
			<< methodeAlgoName << "," // ALGO DEPLACEMENT
			<< std::setprecision(0) << croisementVector.size() << ","; // NOMBRE RUN
		if (nombreSlots > 0) { resultats << nombreSlots << ","; } // NOMBRE EMPLACEMENT OU FLOTTANTS
		else { resultats << "FLOTTANTS,"; } // NOMBRE EMPLACEMENT OU FLOTTANTS
		resultats << nbSolutionIllegale << "," // NOMBRE DE RUN AVEC RESULTATS ILLEGAL
			<< totalInterMoyenne << "," << totalInterIllMoyenne << "," // NOMBRE TOTAL INTER ET NOMBRE TOTAL INTER ILLEGAL
			<< meilleurCroisement << ","; // MEILLEUR SCORE PARMIS LES RUNS
		if (moyenneCroisement > 100) { resultats << std::setprecision(0) << moyenneCroisement << ","; } // MOYENNE SCORE DES RUNS
		else { resultats << std::setprecision(1) << moyenneCroisement << ","; } // MOYENNE SCORE DES RUNS
		if (medianCroisement > 100) { resultats << std::setprecision(0) << medianCroisement << ","; } // MEDIANE SCORE DES RUNS
		else { resultats << std::setprecision(1) << medianCroisement << ","; } // MEDIANE SCORE DES RUNS
		resultats << placementInterMoyenne << ","; // MOYENNE INTERSECTION APRES PLACEMENT DES RUNS
		resultats << std::setprecision(0) << tempsPlacementMoyenne << "," << tempsBestMoyenne << "," << tempsExecMoyenne; // TEMPS MOYEN (PLACEMENT, MEILLEUR RESULTATS ALGO, TOTAL ALGO) DES RUNS
		if (isGenetique) { resultats << std::setprecision(0) << "," << population << "," << maxIteration << "," << bestIterationMoyenne << "," << lastIterationMoyenne; } // GENETIQUE POPULATION, MAX ITERATION, BEST ITERATION, LAST ITERATION
		else if (isRecuit) { resultats << "," << std::setprecision(1) << nombreRecuitMoyenne; } // NOMBRE RECHAUFFE RECUIT
		else { resultats << ","; }
		resultats << ",";
		if (isSeedResetting(tid)) { resultats << "R-"; } // SEED RESET
		else { resultats << "NR-"; } // SEED NO RESET
		resultats << getSeed(tid) << "," << machine; // SEED
		resultats << "," << getParamAsString(customParam); // CUSTOM PARAMS
		resultats << "," << nombreCellule; // NOMBRE CELLULE
		resultats << "," << debugValue << "," << date; // DEBUG VALUE ET DATE
		if (edgeCostVector.size() > 0) { resultats << ",EC:" << edgeCostMoyenne; } // MOYENNE EDGE COST
		resultats << "\n";
		resultats.close();
	}
}


#endif
