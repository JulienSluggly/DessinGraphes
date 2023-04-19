#include <string>
#include <stdio.h>
#include <random>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <cstdlib>

#include "graphe.hpp"
#include "dispOpenGL.hpp"
#include "noDispRuns.hpp"
#include "personnel.hpp"
#include "ogdfFunctions.hpp"
#include "logCSV.hpp"
#include "utilitaire.hpp"
#include "stressMaj.hpp"
#include "solver.hpp"

#if defined(GPERF_INSTALLED)
	#include <gperftools/profiler.h>
#endif

void initCPUSet() {
	omp_set_nested(1);
    int num_threads = ::omp_get_max_threads();
    CPU_ZERO(&cpuset);
    for (int i = 0; i < num_threads; i++) {
      CPU_SET(i, &cpuset);
    }
	sched_setaffinity(0, sizeof(cpuset), &cpuset);
}

void printDebugData(Graphe& G, double tempsBest, int bestIteration, int lastIteration, int nombreRecuit, std::chrono::time_point<std::chrono::system_clock> start, std::chrono::time_point<std::chrono::system_clock> finPlacement) {
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> secondsTotal = end - start;
	std::chrono::duration<double> secondsPlacement = finPlacement - start;
	std::chrono::duration<double> secondsAlgo = end - finPlacement;
	std::cout << std::fixed << secondsPlacement.count() << "s placement. " << secondsAlgo.count() << "s algo. " << secondsTotal.count() << "s total.\n";
	if (tempsBest != -1) std::cout << tempsBest << "s meilleur resultat.\n";
	if (bestIteration != -1) std::cout << "A la " << bestIteration << "eme iteration\n";
	if (lastIteration != -1) std::cout << "Max iteration: " << lastIteration << "\n";
	if (nombreRecuit != 0) std::cout << "Nombre de rechauffe: " << nombreRecuit << "\n";
	if ((G.useCoordReel)||(G.estPlace())) { 
		std::cout << "Nombre intersection apres placement: ";
		if (!G.useCoordReel) { std::cout << G.getNbCroisementConst() << std::endl; }
		else { std::cout << G.getNbCroisementReelConst() << std::endl; }
		if (G.hasIllegalCrossing())  {
			std::cout << "Solution illegale.\n";
			if (!G.useCoordReel) { G.getNbCroisementDiff(); } else { G.getNbCroisementDiffReel(); }
			std::cout << "Total Inter: " << G.nombreInter + G.nombreInterIll + G.nombreInterIllSelf << " normales: " << G.nombreInter << " illegales: " << G.nombreInterIll << " self: " << G.nombreInterIllSelf << std::endl;
		}
		G.debugEverything(false,false);
	}
	std::cout << "Setup complete!" << std::endl;
}

void runFuncOnFolder() {
	std::string path = chemin + "benchGraphs/sparceMC/";
	for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
		std::cout << "---------------------\n";
		Graphe G;
		std::ifstream file(dirEntry.path().string());
		if (file) {
			std::stringstream buffer;
			buffer << file.rdbuf();
			file.close();
			ogdfReadFromMM(G, buffer);
			std::string output = dirEntry.path().string();
			output = output + "clean";
			G.writeToJsonCleanGraphe(output);
		}
		std::cout << dirEntry.path().string() << " Grid" << std::endl;
		auto start = std::chrono::system_clock::now();
		double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit=0;
		std::cout << "Noeud: " << G._noeuds.size() << " Arete: " << G._aretes.size() << std::endl;
		auto finPlacement = std::chrono::system_clock::now();
		printDebugData(G,tempsBest,bestIteration,lastIteration,nombreRecuit,start,finPlacement);
	}
}

void runFuncOnAllGraphs() {
	for (int i=1;i<=12;i++) {
		std::cout << "---------------------\n";
		Graphe G;
		std::string numero = to_string(i);
		std::string nomFichierGraph = "graph-"+numero+"-input";
		std::string nomFichierSlots = numero+"-input-slots";
		std::cout << nomFichierGraph << " " << nomFichierSlots << std::endl;

		G.setupGraphe(nomFichierGraph,nomFichierSlots);
		auto start = std::chrono::system_clock::now();
		double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit=0;
		ogdfOther(G);
		auto finPlacement = std::chrono::system_clock::now();
		printDebugData(G,tempsBest,bestIteration,lastIteration,nombreRecuit,start,finPlacement);
	}
}

void runFuncOnAllGraphsAllSlots(bool useGrid=true) {
	int nbSlot = 4;
	if (!useGrid) { nbSlot--; }
	for (int i=5;i<=12;i++) {
		for (int j=0;j<nbSlot;j++) {
			std::cout << "---------------------\n";
			Graphe G;
			std::string numero = to_string(i);
			std::string nomFichierGraph = "graph-"+numero+"-input";
			std::string nomFichierSlots;
			if (j == 0) { nomFichierSlots = numero+"-input-slots"; }
			else if (j < 3) { nomFichierSlots = to_string(j+1) + "X-"+numero+"-input-slots"; }
			else { nomFichierSlots = "Grid"; }
			std::cout << nomFichierGraph << " " << nomFichierSlots << std::endl;

			G.setupGraphe(nomFichierGraph,nomFichierSlots);
			auto start = std::chrono::system_clock::now();
			double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit=0;
			std::cout << "Moyenne: " << G.distMoyNTirage(1000000) << std::endl;
			auto finPlacement = std::chrono::system_clock::now();
			printDebugData(G,tempsBest,bestIteration,lastIteration,nombreRecuit,start,finPlacement);
		}
	}
}

void stopGprofProfiler(bool useProfiler) {
#if defined(GPERF_INSTALLED)
	if (useProfiler) {
		ProfilerFlush();
		ProfilerStop();
		if (std::system("/home/uha/Documents/DessinGrapheCmake/build/generate_svg.sh") == -1) { std::cout << "Could not generate svg file.\n";  }
		else { std::cout << "SVG file generated successfully.\n"; }
	}
#endif
}

int main() {
	bool useProfiler = false;
#if defined(GPERF_INSTALLED)
	std::string cheminProfile = chemin + "profilerData/profile.output";
	if (useProfiler) { ProfilerStart(cheminProfile.c_str()); }
#endif
	initCPUSet();
	initRandomSeed();
	//initSameSeed();
	//initSameSeedIncThread();
	customRecuitFlottantsAllRuns(); return 0;
	bool useCoordReel = true;
	std::string nomFichierGraph = "graph-3-input";
	std::string nomFichierSlots = "3X-3-input-slots";
	//std::string nomFichierSlots = "Grid";
	std::cout << nomFichierGraph << " " << nomFichierSlots << std::endl;
	Graphe G(nomFichierGraph); G.useCoordReel = useCoordReel;
	std::string pathGraph = chemin + "exemple/Graphe/" + nomFichierGraph + ".json";
	//G.setupGraphe(nomFichierGraph,nomFichierSlots);
	//G.readFromJsonOldGraph(chemin + "automatique/auto21-10.json"); G.generateGrid(G._noeuds.size()/2,G._noeuds.size()/2);
	//ogdfReverse(G);
	//G.readFromJsonGraph("/home/uha/Documents/DessinGrapheCmake/src/benchGraphs/runs/mahindas.mtxclean");
	//G.readFromJsonGraph("/home/uha/Documents/DessinGrapheCmake/src/benchGraphs/runs/adjnoun.graphclean");
	G.readFromJsonGraph(pathGraph);
	G.fillCommonNodeVectors();
	//G.initCompleteGraph(9);
	int nbNoeud = std::min((int)G._noeuds.size()*2,6000);
	//if (!useCoordReel) { G.generateGrid(nbNoeud,nbNoeud); }
	std::cout << "Debut placement. Nombre Noeuds: " << G._noeuds.size() << " Nombre Aretes: " << G._aretes.size() << " Nombre Emplacement: " << G._emplacements.size() << " Nombre Cellules: " << (int)ceil(sqrt(G._aretes.size())*1.5)*(int)ceil(sqrt(G._aretes.size())*1.5) << " Connexe: " << G.isGrapheConnected() << std::endl;
	auto start = std::chrono::system_clock::now();
	double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit=0; 
	//G.grapheGenetique(tempsBest,bestIteration,lastIteration,100,1000,fileGraph,fileSlots,true,false,3);
	//G.grapheGenetique(tempsBest,bestIteration,lastIteration,300,1000,nomFichierGraph,nomFichierSlots,false,false,6);
	//std::cout << nombreIterationRecuit(150.0,0.999999,0.000001) << std::endl;
	//ogdfFastMultipoleMultilevelEmbedder(G);
	//G.stressMajorization({{}},1);
	//G.stressMajorization();
	//ogdfOther(G);
	//G.placementAleatoireReel();
	ogdfFastMultipoleMultilevelEmbedderReel(G);
	//ogdfFastMultipoleMultilevelEmbedderReelMinute(G);
	//G.forcePlacement();
	//G.stressMajorizationReel();
	G.translateGrapheToOriginReel(-1);
	G.setupGridAndRegistration({});
	sched_setaffinity(0, sizeof(cpuset), &cpuset);
	auto finPlacement = std::chrono::system_clock::now();
	//G.recuitSimule(tempsBest,start);
	//G.triangulationDelaunay();
	//G.recuitSimule(tempsBest,start,{},0.99999,100.0,0.0001,1,0,3);
	//G.rerecuitSimule(tempsBest,nombreRecuit,start,{},-1,0.99999,0.99,100.0,0.0001,1,0,2);
	//G.recuitSimuleReel(tempsBest,start,{},0.99999,100.0,0.0001,1,0,4,true);
	//G.recuitSimuleReel(tempsBest,start,{{9,0.99999},{13,1.5}},0.99999,100.0,0.0001,1,0,2,true);
	//G.recuitSimuleReel(tempsBest,start,{{13,1.5}},0.99999,0.01,0.0001,1,0,2,true);
	//G.recuitSimuleReelThreadSelection(tempsBest,start,{{}},0.99999,100.0,0.0001,1,0,2,true);/
	//G.recuitSimuleReelThread(tempsBest,start,{},0.99999,100.0,0.0001,1,0,2,true,false,false);
	//G.recuitSimuleReelThreadPool(tempsBest,start,{},0.99999,0.01,0.0001,1,0,2,true,false,false);
	//G.recuitSimuleReelThread(tempsBest,start,{},0.99999,100.0,0.0001,1,0,2,true,false,false);
	G.rerecuitSimuleReel(tempsBest,nombreRecuit,start,{{9,0.99999},{13,1.5}},-1,0.99999,0.99,100.0,0.0001,1,0,2,true);

	//G.afficherInfo();
	stopGprofProfiler(useProfiler);
	printDebugData(G,tempsBest,bestIteration,lastIteration,nombreRecuit,start,finPlacement);

	#ifdef OPENGL_INSTALLED
	bool useOpenGL = true;
	if (useOpenGL) {
		std::cout << "Grid: " << G.gridWidth << " " << G.gridHeight << std::endl;
		dispOpenGL(G,G.gridWidth,G.gridHeight,useCoordReel);
	}
	#endif
	
	return 0;
}