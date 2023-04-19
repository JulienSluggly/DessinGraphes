#ifndef NODISPRUN_HPP
#define NODISPRUN_HPP

#include <string>
#include <fstream>
#include "graphe.hpp"
#include "logCSV.hpp"
#include "personnel.hpp"
#include <filesystem>
#include <chrono>

void readOldFiles(Graphe& G) {
	std::string nomFichierGraph = "auto21-6";
	std::string nomFichier = "D:/The World/Cours/M2S1/ProjetDessinGraphe/GitHub/ProjetDessinDeGraphe/automatique/" + nomFichierGraph + ".json";
	//string nomFichier = "D:/The World/Cours/M2S1/ProjetDessinGraphe/GitHub/ProjetDessinDeGraphe/2019/" + nomFichierGraph + ".json";
	std::vector<int> tmpvec = G.readFromJsonOldGraph(nomFichier);
	std::string nomFichierSlots = nomFichierGraph + "-slots.json";
	std::string nomFichierGrapheSave = "graph-" + nomFichierGraph + ".json";
	bool save = false;
	if (save) {
		bool savex2 = false;
		bool savex3 = false;
		if (savex2) {
			G.generateMoreEmplacement(2);
			nomFichierSlots = "2X-" + nomFichierSlots;
		}
		else if (savex3) {
			G.generateMoreEmplacement(3);
			nomFichierSlots = "3X-" + nomFichierSlots;
		}
		G.writeToJsonSlots(nomFichierSlots);
		//writeToJsonGraph(G, nomFichierGrapheSave);
	}
}

void startRunsForAllSlots(std::pair<std::string, std::vector<std::string>>& pairGraphSlots, int nbRun, std::string methodePlacement, std::string methodeAlgo,std::vector<std::vector<double>> customParam, int tid) {
	for (int i = 0; i < pairGraphSlots.second.size(); i++) {
		generateCSV(nbRun, methodePlacement, methodeAlgo, pairGraphSlots.first, pairGraphSlots.second[i],customParam,tid);
	}
}

// A besoin d'au moin slotFiles.size() threads pour effectuer toutes les executions.
void customRecuit() {
	fillLogsVector();
	std::string nomFichierGraph = "graph-10-input";
	std::vector<std::string> slotFiles = { "11-input-slots", "2X-11-input-slots", "3X-11-input-slots", "GRID" };
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid < slotFiles.size()) {
		//if (tid < 1) {
			if (tid == 0) {
				printf("Number of threads working on training data: %d\n", nthreads);
			}
			std::vector<std::vector<std::vector<double>>> totalRuns;
			for (int i=0;i<totalRuns.size();i++) {
				generateCSV(-1,"Stress","Aucun",nomFichierGraph,slotFiles[tid],totalRuns[i],tid);
			}
			generateCSV(10,"OGDFFMMM","Recuit Simule Grille TME",nomFichierGraph,slotFiles[tid],{},false,tid);
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void customRecuitFlottants() {
	fillLogsVector();
	std::string nomFichierGraph = "graph-10-input";
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = omp_get_thread_num();
		nthreads = omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		std::vector<std::vector<std::vector<double>>> totalRuns;
		for (int i=0;i<totalRuns.size();i++) {
			if (i%nthreads == tid) {
				generateCSV(-1,"OGDFFMMM","Rerecuit Simule Grille TME",nomFichierGraph,"",totalRuns[i],true,tid);
			}
		}
		generateCSV(-1,"OGDFFMMM","Recuit Simule Grille TME",nomFichierGraph,"",{},true,tid);
		//generateCSV(1,"OGDFFMMM","Recuit Simule Grille TME",nomFichierGraph,"",{},true,tid);
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void customRecuitFlottantsAllRuns() {
	fillLogsVector();
	std::vector<std::string> graphVector;
	for (int i = 5; i <= 12; i++) {
		graphVector.push_back("graph-" + std::to_string(i) + "-input");
	}
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		int indexKey = 0;
		std::vector<std::vector<std::vector<double>>> totalRuns;
		totalRuns.push_back({{9,0.9999945},{13,1.5}});
		totalRuns.push_back({{9,0.999999},{13,1.5}});
		totalRuns.push_back({{9,0.99999945},{13,1.5}});
		for (const std::string& nomFichierGraph : graphVector) {
			if (indexKey%nthreads == tid) {
				for (int i=0;i<totalRuns.size();i++) {
					generateCSV(-1,"OGDFFMMM","Rerecuit Simule Grille TME",nomFichierGraph,"",totalRuns[i],true,tid);
				}
			}
			indexKey++;
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void customRecuitAllRuns() {
	fillLogsVector();
	std::vector<std::pair<std::string, std::vector<std::string>>> mapGraphSlots;
	for (int i = 1; i <= 12; i++) {
		mapGraphSlots.push_back({ "graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots", "2X-" + std::to_string(i) + "-input-slots", "3X-" + std::to_string(i) + "-input-slots", "GRID"} });
	}
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		int indexKey = 0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (auto &key : mapGraphSlots) {
			if (tid == (indexKey % nthreads)) {
				for (int numSlot = 0; numSlot < key.second.size(); numSlot++) {
					std::vector<std::vector<std::vector<double>>> totalRuns;
					totalRuns.push_back({{10,1}});
					for (int i=0;i<totalRuns.size();i++) {
						generateCSV(2,"Aleatoire","Recuit Simule Grille TME",key.first,key.second[numSlot],totalRuns[i],false,tid);
					}
				}
			}
			indexKey++;
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void allRunsSingleThread() {
	fillLogsVector();
	std::vector<std::pair<std::string, std::vector<std::string>>> mapGraphSlots;
	std::vector<std::string> methodesPlacement = { "Aleatoire" };
	std::vector<std::string> methodesAlgo = { "Aucun" };
	for (int i = 1; i <= 12; i++) {
		//mapGraphSlots.push_back({ "graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots", "2X-" + std::to_string(i) + "-input-slots", "3X-" + std::to_string(i) + "-input-slots", "GRID"} });
		mapGraphSlots.push_back({ "graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots"} });
	}
	int nbRuns = 5;
	std::cout << "Starting all run logs, nb run: " << nbRuns << std::endl;
	
	for (auto &key : mapGraphSlots) {
		for (int i = 0; i < key.second.size(); i++) {
			std::cout << "-----------------------------------------" << std::endl;
			std::cout << "Graphe: " << key.first << " " << key.second[i] << std::endl;
			for (int j = 0; j < methodesPlacement.size(); j++) {
				std::cout << "--------------------------" << std::endl;
				for (int k = 0; k < methodesAlgo.size(); k++) {
					std::cout << "Placement: " << methodesPlacement[j] << " Algo: " << methodesAlgo[k] << std::endl;
					generateCSV(nbRuns, methodesPlacement[j], methodesAlgo[k], key.first, key.second[i]);
				}
			}
		}
	}
}

// Multithreadé
void allRunsLogged() {
	fillLogsVector();
	std::unordered_map<std::string, std::vector<std::string>> mapGraphSlots;
	std::vector<std::string> methodesPlacement = { "Aleatoire"};
	std::vector<std::string> methodesAlgo = { "Rerecuit Simule Grille Best"};
	std::vector<int> runMethodesAlgo = { 1,1,1,1 };
	for (int i = 1; i <= 12; i++) {
		mapGraphSlots.insert({ "graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots","2X-" + std::to_string(i) + "-input-slots","3X-" + std::to_string(i) + "-input-slots","GRID"} });
	}
	std::cout << "Starting all run logs." << std::endl;
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		int indexKey = 0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		int chunk = std::ceil((double)mapGraphSlots.size() / (double)nthreads);
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
			printf("Chunk size: %d\n", chunk);
		}
		for (auto key : mapGraphSlots) {
			if (tid == (indexKey % nthreads)) {
				for (int i = 0; i < key.second.size(); i++) {
					for (int j = 0; j < methodesPlacement.size(); j++) {
						for (int k = 0; k < methodesAlgo.size(); k++) {
							generateCSV(runMethodesAlgo[k], methodesPlacement[j], methodesAlgo[k], key.first, key.second[i],{},tid);
						}
					}
				}
			}
			indexKey++;
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void allRunsBySlots() {
	fillLogsVector();
	std::vector<std::pair<std::string, std::vector<std::string>>> mapGraphSlots;
	//mapGraphSlots.push_back({"graph-8-input",{"8-input-slots","2X-8-input-slots","3X-8-input-slots","GRID"}});
	for (int i = 5; i <= 12; i++) {
		mapGraphSlots.push_back({"graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots","2X-" + std::to_string(i) + "-input-slots","3X-" + std::to_string(i) + "-input-slots","GRID"}});
	}
	std::cout << "Starting all run logs." << std::endl;
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		int chunk = std::ceil((double)mapGraphSlots.size() / (double)nthreads);
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
			printf("Chunk size: %d\n", chunk);
		}
		int indexKey = 0;
		std::vector<std::vector<double>> customParam;
		for (auto& key : mapGraphSlots) {
			if (tid == (indexKey % nthreads)) {
				startRunsForAllSlots(key,-1,"OGDFFMMM","Aucun",{{}},tid);
			}
			indexKey++;
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void allRunsBySlotsSecondRun() {
	fillLogsVector();
	std::vector<std::pair<std::string, std::vector<std::string>>> mapGraphSlots;
	//mapGraphSlots.push_back({"graph-8-input",{"8-input-slots","2X-8-input-slots","3X-8-input-slots","GRID"}});
	for (int i = 9; i <= 12; i++) {
		mapGraphSlots.push_back({"graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots","2X-" + std::to_string(i) + "-input-slots","3X-" + std::to_string(i) + "-input-slots","GRID"}});
	}
	std::cout << "Starting all run logs." << std::endl;
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		int chunk = std::ceil((double)mapGraphSlots.size() / (double)nthreads);
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
			printf("Chunk size: %d\n", chunk);
		}
		int indexKey = 0;
		std::vector<std::vector<std::vector<double>>> totalRuns;
		totalRuns.push_back({{9,0.99999}});
		totalRuns.push_back({{9,0.99999},{3,0,2}});
		totalRuns.push_back({{9,0.9999945}});
		totalRuns.push_back({{9,0.9999945},{3,0,2}});
		totalRuns.push_back({{9,0.999999}});
		totalRuns.push_back({{9,0.999999},{3,0,2}});
		for (auto& key : mapGraphSlots) {
			if (tid == (indexKey % nthreads)) {
				for (int taille=0;taille<totalRuns.size();taille++) {
					startRunsForAllSlots(key,-1,"Stress Dyn Stress","Rerecuit Simule Grille TME Custom",totalRuns[taille],tid);
					startRunsForAllSlots(key,-1,"OGDFFMMM","Rerecuit Simule Grille TME Custom",totalRuns[taille],tid);
				}
			}
			indexKey++;
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void allRunsBySlotsThirdRun() {
	fillLogsVector();
	std::vector<std::pair<std::string, std::vector<std::string>>> mapGraphSlots;
	for (int i = 5; i <= 12; i++) {
		mapGraphSlots.push_back({"graph-" + std::to_string(i) + "-input",{std::to_string(i) + "-input-slots","2X-" + std::to_string(i) + "-input-slots","3X-" + std::to_string(i) + "-input-slots","GRID"}});
	}
	std::cout << "Starting all run logs." << std::endl;
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		int chunk = std::ceil((double)mapGraphSlots.size() / (double)nthreads);
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
			printf("Chunk size: %d\n", chunk);
		}
		int indexKey = 0;
		std::vector<std::vector<std::vector<double>>> totalRuns;
		totalRuns.push_back({{1,5}});
		for (auto& key : mapGraphSlots) {
			if (tid == (indexKey % nthreads)) {
				for (int taille=0;taille<totalRuns.size();taille++) {
				}
			}
			indexKey++;
		}
		printf("Thread: %d done.\n",tid);
	}
	printf("All Threads done.\n");
}

void allRunsByOnFolder() {
	fillLogsVector();
	std::cout << "Starting all run logs." << std::endl;
	std::string path = chemin + "benchGraphs/runs/";
	std::string slots = "Grid";
	int nthreads, tid;
	std::vector<std::vector<std::vector<double>>> totalRuns;
	totalRuns.push_back({{9,0.99999}});
	totalRuns.push_back({{9,0.99999},{3,0,2}});
	totalRuns.push_back({{9,0.9999945}});
	totalRuns.push_back({{9,0.9999945},{3,0,2}});
	totalRuns.push_back({{9,0.999999}});
	totalRuns.push_back({{9,0.999999},{3,0,2}});
#pragma omp parallel private(tid)
	{
		int indexKey = 0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
			if (tid == (indexKey % nthreads)) {
				for (int numeroParam=0;numeroParam<totalRuns.size();numeroParam++) {
					generateCSV(-1, "Stress Dyn Stress", "Rerecuit Simule Grille TME Custom", dirEntry.path().string(), slots,totalRuns[numeroParam],tid);
				}
			}
			indexKey++;
		}
		printf("Thread %d done.\n", tid);
	}
}

// Multithreading sur un seul graphe pour differentes methodes
void specificGraphMulti(std::string fileGraph, std::string fileSlots, bool useSingleFile=false) {
	int nthreads, tid;
	int nbExec = 2;
	std::vector<std::string> methodesPlacement = { "Aleatoire", "Aucun" };
	std::vector<std::string> methodesAlgo = { "Recuit Simule", "Genetique Score" };
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid < nbExec) {
			Graphe G;
			if (!useSingleFile) {
				G.readFromJsonGraph(fileGraph);
				G.readFromJsonSlots(fileSlots);
			}
			else { 
				G.readFromJsonGraphAndSlot(fileGraph);
			}
			if (tid == 0) {
				printf("Number max of threads working on training data: %d\n", nthreads);
			}
			std::string nomFichierLog = "Graph-" + std::to_string(tid);
			//generateCSV(1, methodesPlacement[tid], methodesAlgo[tid], nomFichierLog, G, fileGraph, fileSlots, true);
		}
		std::cout << "Thread: " << tid << " done." << std::endl;
	}
	printf("All Threads done.\n");
}

void performanceTest() {
	int nombreMax = 1000;

	for (int n=1;n<nombreMax;n++) {

		// Vecteur d'id melange
		std::vector<int> shuffledIdVec;
		for (int i=0;i<5000*n;i++) {
			shuffledIdVec.push_back(i);
		}
		auto rng = std::default_random_engine {};
		std::shuffle(std::begin(shuffledIdVec), std::end(shuffledIdVec), rng);

		// Vecteur d'id melange
		std::vector<int> shuffledIdVec2;
		for (int i=0;i<5000*n;i++) {
			shuffledIdVec2.push_back(i);
		}
		std::shuffle(std::begin(shuffledIdVec2), std::end(shuffledIdVec2), rng);

		// Vecteur d'id genere au random
		std::vector<int> randomIdVec;
		for (int i=0;i<30000;i++) {
			randomIdVec.push_back(generateRand((5000*n)-1));
		}


		// Partie unordered set
		int nombreElem = n*50;
		std::cout << "Nombre element: " << nombreElem << std::endl;
		auto start = std::chrono::system_clock::now();
		std::unordered_set<int> nodeUSet;
		for (int i=0;i<nombreElem;i++) {
			nodeUSet.insert(shuffledIdVec[i]);
		}
		auto s1 = std::chrono::system_clock::now();
		for (int i=0;i<30000;i++) {
			nodeUSet.count(randomIdVec[i]);
		}
		auto s2 = std::chrono::system_clock::now();
		for (int i=0;i<nombreElem;i++) {
				nodeUSet.erase(shuffledIdVec2[i]);
		}
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> totalTime = end - start;
		std::chrono::duration<double> insertTime = s1-start;
		std::chrono::duration<double> findTime = s2-s1;
		std::chrono::duration<double> eraseTime = end-s2;
		std::cout << "Insert: " << insertTime.count() << "s.\n";
		std::cout << "Find: " << findTime.count() << "s.\n";
		std::cout << "Erase: " << eraseTime.count() << "s.\n";
		std::cout << "Total: " << totalTime.count() << "s.\n";
		std::cout << "-----------------------------------------------\n";

		std::string nomFichier = chemin + "/resultats/perfUSet.csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);

		resultats << std::fixed;
		resultats << std::setprecision(7) << nombreElem << ","
			<< insertTime.count() << ","
			<< eraseTime.count() << ","
			<< findTime.count() << ","
			<< totalTime.count() << "\n";
		resultats.close();
	}
	// Partie Vector
	std::vector<int> nodeVec;
}

void performanceTest2() {
		int nombreElem = 5000;
		// Vecteur d'id melange
		std::vector<int> shuffledIdVec;
		for (int i=0;i<nombreElem;i++) {
			shuffledIdVec.push_back(i);
		}
		auto rng = std::default_random_engine {};
		std::shuffle(std::begin(shuffledIdVec), std::end(shuffledIdVec), rng);

		// Vecteur d'id melange
		std::vector<int> shuffledIdVec2;
		for (int i=0;i<nombreElem;i++) {
			shuffledIdVec2.push_back(i);
		}
		std::shuffle(std::begin(shuffledIdVec2), std::end(shuffledIdVec2), rng);

		// Vecteur d'id genere au random
		std::vector<int> randomIdVec;
		for (int i=0;i<30000;i++) {
			randomIdVec.push_back(generateRand((nombreElem*2)-1));
		}


		std::cout << "Nombre element: " << nombreElem << std::endl;
		auto start = std::chrono::system_clock::now();
		std::unordered_set<int> nodeUSet;
		for (int i=0;i<nombreElem;i++) {
			nodeUSet.insert(shuffledIdVec[i]);
		}
		auto s1 = std::chrono::system_clock::now();
		for (int i=0;i<30000;i++) {
			nodeUSet.count(randomIdVec[i]);
		}
		auto s2 = std::chrono::system_clock::now();
		for (int i=0;i<nombreElem;i++) {
				nodeUSet.erase(shuffledIdVec2[i]);
		}
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> totalTime = end - start;
		std::chrono::duration<double> insertTime = s1-start;
		std::chrono::duration<double> findTime = s2-s1;
		std::chrono::duration<double> eraseTime = end-s2;
		std::cout << "Insert: " << insertTime.count() << "s.\n";
		std::cout << "Find: " << findTime.count() << "s.\n";
		std::cout << "Erase: " << eraseTime.count() << "s.\n";
		std::cout << "Total: " << totalTime.count() << "s.\n";
		std::cout << "-----------------------------------------------\n";

		std::string nomFichier = chemin + "/resultats/perfUSet.csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);

		resultats << std::fixed;
		resultats << std::setprecision(7) << nombreElem << ","
			<< insertTime.count() << ","
			<< eraseTime.count() << ","
			<< findTime.count() << ","
			<< totalTime.count() << "\n";
		resultats.close();
	// Partie Vector
	std::vector<int> nodeVec;
}

void testRomeGraphs() {
	std::string path = chemin + "benchGraphs/rome100/";
		int nthreads, tid;
#pragma omp parallel private(tid)
	{
		int i=0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
			if (i%nthreads == tid) {
				printf("Tid: %d | i: %d\n",tid,i);
				Graphe G;
				G.readFromGraphmlGraph(dirEntry.path().string());
				auto start = std::chrono::system_clock::now();
				double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit=0; 
				G.generateGrid();
				G.stressMajorization({},1);
				G.initGrille();
				G.registerSlotsAndEdgesInGrid();
				G.rerecuitSimule(tempsBest,nombreRecuit,start,{},-1,0.999999);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::string nomFichier = chemin + "resultats/resultatsRome/" + to_string(tid) + ".csv";
				std::ofstream resultats(nomFichier, std::ios_base::app);
				resultats << dirEntry.path().string() << "," << to_string(i) << "," << G.getNbCroisementDiff() << "," << tempsBest << "," << secondsTotal.count() << std::endl;
				resultats.close();
			}
			i++;
		}
		printf("Thread %d done.\n",tid);
	}
}

void cleanDimacsGraphs() {
	std::string path = chemin + "benchGraphs/dimacs/";
	for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
		if (!containsString(dirEntry.path().string(),"clean")) {
			Graphe G;
			G.readFromDimacsGraphClean(dirEntry.path().string());
			std::string output = dirEntry.path().string();
			output = output + "clean";
			G.writeToJsonComposanteConnexe(output,G.plusGrandeComposanteConnexe());
		}
	}
}

void testCleanGraphs() {
	std::string path = chemin + "benchGraphs/runs/";
		int nthreads, tid;
#pragma omp parallel private(tid)
	{
		int i=0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
			if (i%nthreads == tid) {
				printf("Tid: %d | i: %d | File: %s\n",tid,i,dirEntry.path().string().c_str());
				Graphe G;
				G.readFromJsonGraph(dirEntry.path().string());
				auto start = std::chrono::system_clock::now();
				double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit=0;
				int nbNoeud = 2 * G._noeuds.size();
				if (G._noeuds.size() > 3000) {
					nbNoeud = 6000;
				}
				G.generateGrid(nbNoeud,nbNoeud);
				G.stressMajorization({},1);
				auto finPlacement = std::chrono::system_clock::now();
				G.initGrille();
				G.registerSlotsAndEdgesInGrid();
				G.rerecuitSimule(tempsBest,nombreRecuit,start,{{9,0.9999945},{3,0,2}});
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::chrono::duration<double> secondsPlacement = finPlacement - start;
				std::string nomFichier = chemin + "resultats/resultatsBench/" + to_string(tid) + ".csv";
				std::ofstream resultats(nomFichier, std::ios_base::app);
				resultats << dirEntry.path().string() << "," << to_string(i) << "," << G.getNbCroisementDiff() << "," << tempsBest << "," << secondsTotal.count() << "," << secondsPlacement.count()<< std::endl;
				resultats.close();
			}
			i++;
		}
		printf("Thread %d done.\n",tid);
	}
}

void compareStressFMMM() {
	std::string path = chemin + "benchGraphs/runs/";
		int nthreads, tid;
#pragma omp parallel private(tid)
	{
		int i=0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
			if (i%nthreads == tid) {
				resetSeed(tid,true,true);
				printf("Tid: %d | i: %d | File: %s\n",tid,i,dirEntry.path().string().c_str());
				for (int methode=0;methode<=2;methode++) {
					for (int slots=-3;slots<=9;slots++) {
						Graphe G;
						G.readFromJsonGraph(dirEntry.path().string());
						int nbNoeud = std::min((int)G._noeuds.size()*2,6000);
						G.generateActivationGrid(nbNoeud,nbNoeud,9);
						G.gridWidth = nbNoeud;
						G.gridWidth = nbNoeud;
						if (slots < 0) {
							if (slots == -3) { G.generateEmplacements(G._noeuds.size()); }
							else if (slots == -2) { G.generateEmplacements(G._noeuds.size()*2); }
							else if (slots == -1) { G.generateEmplacements(G._noeuds.size()*3); }
						}
						else { G.activateSlotsGridUntil(slots); }
						auto start = std::chrono::system_clock::now();
						if (methode == 0) { G.stressMajorization({},1); }
						else if (methode == 1) { G.stressMajorization(); }
						else if (methode == 2) { ogdfFastMultipoleMultilevelEmbedder(G); }
						sched_setaffinity(0, sizeof(cpuset), &cpuset);
						auto finPlacement = std::chrono::system_clock::now();
						std::chrono::duration<double> secondsPlacement = finPlacement - start;
						std::string nomFichier = chemin + "resultats/resultatsBench/" + to_string(tid) + ".csv";
						std::ofstream resultats(nomFichier, std::ios_base::app);
						resultats << dirEntry.path().string() << "," << methode << "," << slots << "," << G._emplacements.size() << "," << G.getNbCroisementDiff() << "," << secondsPlacement.count() << "," << G.debugEverything() << std::endl;
						resultats.close();
					}
				}
			}
			i++;
		}
		printf("Thread %d done.\n",tid);
	}
}

void testGraphsReel() {
	std::string path = chemin + "benchGraphs/runs/";
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		int i = 0;
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(path)) {
			if (i % nthreads == tid) {
				printf("Tid: %d | i: %d | File: %s\n", tid, i, dirEntry.path().string().c_str());
				Graphe G;
				G.readFromJsonGraph(dirEntry.path().string());
				auto start = std::chrono::system_clock::now();
				double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit = 0;
				G.placementAleatoireReel();
				ogdfFastMultipoleMultilevelEmbedderReel(G);
				sched_setaffinity(0, sizeof(cpuset), &cpuset);
				G.translateGrapheToOriginReel(-1);
				G.initGrilleReel(); G.registerNodesAndEdgesInGrid();
				G.rerecuitSimuleReel(tempsBest, nombreRecuit, start, {},-1,0.999999,0.99,100.0,0.0001,1,0,2);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::string nomFichier = chemin + "resultats/resultatsBench/" + to_string(tid) + ".csv";
				std::ofstream resultats(nomFichier, std::ios_base::app);
				resultats << dirEntry.path().string() << "," << G.getNbCroisementDiffReel() << "," << tempsBest << "," << secondsTotal.count() << "," << std::endl;
				resultats.close();
			}
			i++;
		}
		printf("Thread %d done.\n", tid);
	}
}

void testGraphsCompletReel() {
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		for (int i = 32;i<100;i++) {
			if (i % nthreads == tid) {
				printf("Tid: %d | K%d\n", tid, i);
				Graphe G;
				G.initCompleteGraph(i);
				auto start = std::chrono::system_clock::now();
				double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit = 0;
				ogdfFastMultipoleMultilevelEmbedderReel(G);
				sched_setaffinity(0, sizeof(cpuset), &cpuset);
				G.translateGrapheToOriginReel(-1);
				G.rerecuitSimuleReel(tempsBest, nombreRecuit, start, {},-1,0.999999,0.99,100.0,0.0001,1,0,2,false,false,true);
				auto end = std::chrono::system_clock::now();
				std::chrono::duration<double> secondsTotal = end - start;
				std::string nomFichier = chemin + "resultats/resultatsBench/" + to_string(tid) + ".csv";
				std::ofstream resultats(nomFichier, std::ios_base::app);
				resultats << "K" << i << "," << G.getNbCroisementDiffReel() << "," << tempsBest << "," << secondsTotal.count() << std::endl;
				resultats.close();
			}
		}
		printf("Thread %d done.\n", tid);
	}
	printf("All Threads done.\n");
}

void testThreads() {
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		if (tid < 4) {
			printf("Tid: %d | CPU: %d\n", tid, sched_getcpu());
			std::string nomFichierGraph = "graph-10-input";
			std::string pathGraph = chemin + "exemple/Graphe/" + nomFichierGraph + ".json";
			Graphe G;
			G.readFromJsonGraph(pathGraph);
			auto start = std::chrono::system_clock::now();
			double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit = 0;
			//G.placementAleatoireReel();
			ogdfFastMultipoleMultilevelEmbedderReel(G);
			sched_setaffinity(0, sizeof(cpuset), &cpuset);
			printf("Placement done Tid: %d | CPU: %d\n", tid, sched_getcpu());
			G.translateGrapheToOriginReel(-1);
			G.initGrilleReel(); G.registerNodesAndEdgesInGrid();
			G.recuitSimuleReel(tempsBest, start, {},0.99999,100.0,0.0001,1,0,2,true,false,false);
			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> secondsTotal = end - start;
			std::string nomFichier = chemin + "resultats/" + to_string(tid) + ".csv";
			std::ofstream resultats(nomFichier, std::ios_base::app);
			resultats << tid << "," << G.getNbCroisementDiffReel() << "," << tempsBest << "," << secondsTotal.count() << std::endl;
			resultats.close();
		}
		printf("Thread %d done.\n", tid);
	}
	printf("All Threads done.\n");
}

void testThreads2() {
	int nthreads, tid;
#pragma omp parallel private(tid)
	{
		tid = ::omp_get_thread_num();
		nthreads = ::omp_get_num_threads();
		if (tid == 0) {
			printf("Number of threads working on training data: %d\n", nthreads);
		}
		std::string nomFichierGraph = "graph-10-input";
		std::string pathGraph = chemin + "exemple/Graphe/" + nomFichierGraph + ".json";
		Graphe G;
		G.readFromJsonGraph(pathGraph);
		G.generateGrid();
		auto start = std::chrono::system_clock::now();
		double tempsBest = -1; int bestIteration = -1; int lastIteration = -1; int nombreRecuit = 0;
		ogdfFastMultipoleMultilevelEmbedder(G);
		sched_setaffinity(0, sizeof(cpuset), &cpuset);
		G.initGrille(); G.registerSlotsAndEdgesInGrid();
		G.recuitSimule(tempsBest, start, {},0.99999,100.0,0.0001,1,0,2,true,false);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> secondsTotal = end - start;
		std::string nomFichier = chemin + "resultats/" + to_string(tid) + ".csv";
		std::ofstream resultats(nomFichier, std::ios_base::app);
		resultats << tid << "," << G.getNbCroisementDiff() << "," << tempsBest << "," << secondsTotal.count() << std::endl;
		resultats.close();
		printf("Thread %d done.\n", tid);
	}
	printf("All Threads done.\n");
}

#endif