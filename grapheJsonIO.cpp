#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "graphe.hpp"
#include <unordered_map>
#include <set>
#include <map>

using nlohmann::json;

bool DEBUG_JSON = false;

// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
// Precondition: Les id des noeuds sont ordonnees et commencent par 0
void Graphe::readFromJsonGraph(std::string input) {
	if (DEBUG_JSON) std::cout << "Fichier Graphe: " << input << std::endl;
	std::ifstream inp(input);
	json j;
	inp >> j;

	// Si le fichier ne contient pas de node
	if (j["nodes"] == nullptr) {
		exit(1);
	}
	int nodeNumber = static_cast<int>(j["nodes"].size());
	if (DEBUG_JSON) std::cout << "Nombre de noeud dans le json: " << nodeNumber << std::endl;
	_noeuds.reserve(nodeNumber*2);
	for (int i = 0; i < nodeNumber; i++) {
		_noeuds.push_back(Noeud(i));
	}
	int edgeNumber = static_cast<int>(j["edges"].size());
	if (DEBUG_JSON) std::cout << "Nombre d'arete dans le json: " << edgeNumber << std::endl;
	int id1, id2;
	_aretes.reserve(edgeNumber*2);
	for (int i = 0; i < edgeNumber; i++) {
		id1 = j["edges"][i]["source"];
		id2 = j["edges"][i]["target"];
		_aretes.push_back(Aretes(&_noeuds[id1], &_noeuds[id2],i));
	}
}

// Lecture des slots
// Precondition: Les id des slots sont ordonn�s et commencent par 0
void Graphe::readFromJsonSlots(std::string input) {
	if (DEBUG_JSON) std::cout << "Fichier Slot: " << input << std::endl;
	std::ifstream inp(input);
	json j;
	inp >> j;

	// Si le fichier ne contient pas de slots
	if (j["slots"] == nullptr) {
		exit(1);
	}

	int slotsNumber = static_cast<int>(j["slots"].size());
	if (DEBUG_JSON) std::cout << "Nombre de slots dans le fichier json: " << slotsNumber << std::endl;
	int x, y;
	_emplacements.reserve(slotsNumber*2);
	for (int i = 0; i < slotsNumber; i++) {
		x = j["slots"][i]["x"];
		y = j["slots"][i]["y"];
		_emplacements.push_back(Emplacement(x,y,i));
		if (x > gridWidth) { gridWidth = x; }
		if (y > gridHeight) { gridHeight = y; }
	}
}

// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
// Precondition: Les id des noeuds sont ordonn�es et commencent par 0
void Graphe::readFromJsonGraphAndSlot(std::string input) {
	if (DEBUG_JSON) std::cout << "Fichier Graphe: " << input << std::endl;
	std::ifstream inp(input);
	json j;
	inp >> j;

	// Si le fichier ne contient pas de node
	if (j["nodes"] == nullptr) {
		exit(1);
	}
	// Si le fichier ne contient pas de slots
	if (j["slots"] == nullptr) {
		exit(1);
	}

	int slotsNumber = static_cast<int>(j["slots"].size());
	if (DEBUG_JSON) std::cout << "Nombre de slots dans le fichier json: " << slotsNumber << std::endl;
	int x, y;
	_emplacements.reserve(slotsNumber*2);
	for (int i = 0; i < slotsNumber; i++) {
		x = j["slots"][i]["x"];
		y = j["slots"][i]["y"];
		_emplacements.push_back(Emplacement(x,y,i));
		if (x > gridWidth) { gridWidth = x; }
		if (y > gridHeight) { gridHeight = y; }
	}

	int nodeNumber = static_cast<int>(j["nodes"].size());
	if (DEBUG_JSON) std::cout << "Nombre de noeud dans le json: " << nodeNumber << std::endl;
	int id;
	_noeuds.reserve(nodeNumber*2);
	for (int i = 0; i < nodeNumber; i++) {
		_noeuds.push_back(Noeud(i));
	}
	for (int i=0;i<nodeNumber;i++) {
		id = j["nodes"][i]["id_slot"];
		if (id != -1) {
			_noeuds[i].setEmplacement(&_emplacements[id]);
		}
	}
	int edgeNumber = static_cast<int>(j["edges"].size());
	if (DEBUG_JSON) std::cout << "Nombre d'arete dans le json: " << edgeNumber << std::endl;
	int id1, id2;
	_aretes.reserve(edgeNumber*2);
	for (int i = 0; i < edgeNumber; i++) {
		id1 = j["edges"][i]["source"];
		id2 = j["edges"][i]["target"];
		_aretes.push_back(Aretes(&_noeuds[id1], &_noeuds[id2],i));
	}
}

// Sauvegarde des slots dans le fichier output
void Graphe::writeToJsonSlots(std::string output) {
	json j;
	j["width"] = gridWidth;
	j["height"] = gridHeight;

	int slotsNumber = _emplacements.size();
	for (int i = 0; i < slotsNumber; i++) {
		j["slots"][i]["id"] = i;
		j["slots"][i]["x"] = _emplacements[i].getX();
		j["slots"][i]["y"] = _emplacements[i].getY();
	}

	std::ofstream o(output);
	o << std::setw(4) << j << std::endl;
}

void Graphe::writeToJsonGraph(std::string output) {
	json j;

	int nodeNumber = _noeuds.size();
	for (int i = 0; i < nodeNumber; i++) {
		j["nodes"][i]["id"] = _noeuds[i].getId();
		if (_noeuds[i].getEmplacement() != nullptr)
			j["nodes"][i]["id_slot"] = _noeuds[i].getEmplacement()->getId();
		else
			j["nodes"][i]["id_slot"] = -1;
	}

	int edgeNumber = _aretes.size();
	for (int i = 0; i < edgeNumber; i++) {
		j["edges"][i]["source"] = _aretes[i].getNoeud1()->getId();
		j["edges"][i]["target"] = _aretes[i].getNoeud2()->getId();
	}

	std::ofstream o(output);
	o << std::setw(4) << j << std::endl;
}

void Graphe::writeToJsonComposanteConnexe(std::string output, std::vector<int> composante) {
	json j;
	std::sort(composante.begin(),composante.end());
	std::unordered_map<int,int> mapNewIndex;
	for (int i=0;i<composante.size();i++) {
		mapNewIndex.insert({composante[i],i});
	}
	std::set<std::pair<int,int>> aretesComposante;
	for (int i=0;i<composante.size();i++) {
		j["nodes"][i]["id"] = i;
		if (_noeuds[composante[i]].getEmplacement() != nullptr) { j["nodes"][i]["id_slot"] = _noeuds[composante[i]].getEmplacement()->getId(); }
		else { j["nodes"][i]["id_slot"] = -1; }
		for (const int& idArete : _noeuds[composante[i]]._aretes) {
			int idNoeud1 = mapNewIndex[_aretes[idArete]._noeud1->_id];
			int idNoeud2 = mapNewIndex[_aretes[idArete]._noeud2->_id];
			aretesComposante.insert({idNoeud1,idNoeud2});
		}
	}
	int numeroArete = 0;
	for (const std::pair<int,int>& arete : aretesComposante) {
		j["edges"][numeroArete]["source"] = arete.first;
		j["edges"][numeroArete]["target"] = arete.second;
		numeroArete++;
	}

	std::ofstream o(output);
	o << std::setw(4) << j << std::endl;
}

// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
// Creer les emplacements a partir des anciennes coord
// Les noeuds doivent avoir des coordonn�es
std::vector<int> Graphe::readFromJsonOldGraph(std::string input) {
	std::ifstream inp(input);
	json j;
	inp >> j;
	gridWidth = j["width"];
	gridHeight = j["height"];
	if (DEBUG_JSON) std::cout << "gridWidth: " << gridWidth << " gridHeight: " << gridHeight << std::endl;

	if (j["nodes"] == nullptr) {
		exit(1);
	}

	bool areNodePlaced = true;
	if ((j["nodes"][0]["x"] == j["nodes"][1]["x"])&&(j["nodes"][0]["y"] == j["nodes"][0]["y"])) {
		areNodePlaced = false;
	}

	int nodeNumber = static_cast<int>(j["nodes"].size());
	int x, y;
	_noeuds.reserve(nodeNumber*2);
	_emplacements.reserve(nodeNumber*2);
	for (int i = 0; i < nodeNumber; i++) {
		_noeuds.push_back(Noeud(i));
		x = j["nodes"][i]["x"];
		y = j["nodes"][i]["y"];
		if (areNodePlaced) { _emplacements.push_back(Emplacement(x,y,i)); }
	}

	int edgeNumber = static_cast<int>(j["edges"].size());
	int id1, id2;
	std::vector<int> tmpVec;
	int nbArete = 0;
	_aretes.reserve(edgeNumber*2);
	for (int i = 0; i < edgeNumber; i++) {
		id1 = j["edges"][i]["source"];
		tmpVec.push_back(id1);
		id2 = j["edges"][i]["target"];
		tmpVec.push_back(id2);
	}
	for (int i = 0; i < tmpVec.size(); i += 2) {
		_aretes.push_back(Aretes(&_noeuds[tmpVec[i]], &_noeuds[tmpVec[i + 1]], i / 2));
	}
	return tmpVec;
}

#if defined(PUGIXML_INSTALLED)
#include "pugixml.hpp"

void Graphe::readFromGraphmlGraph(std::string input) {
	pugi::xml_document doc;
	if (!doc.load_file(input.c_str())) { std::cout << "Fichier inexistant.\n"; exit(-1); }

	pugi::xml_node data = doc.child("graphml").child("graph");
    // tag::basic[]
	int i=0;
    for (pugi::xml_node tool = data.child("node"); tool; tool = tool.next_sibling("node"), i++) {
        _noeuds.push_back(Noeud(i));
    }
	i=0;
	for (pugi::xml_node tool = data.child("edge"); tool; tool = tool.next_sibling("edge"), i++) {
        std::string idSourceString = tool.attribute("source").value();
        std::string idTargetString = tool.attribute("target").value();
		int id1 = std::stoi(idSourceString.substr(1));
		int id2 = std::stoi(idTargetString.substr(1));
		_aretes.push_back(Aretes(&_noeuds[id1], &_noeuds[id2],i));
    }
}
#else
void Graphe::readFromGraphmlGraph(std::string input) {
	std::cout << "PUGIXML NOT INSTALLED.\n";
	exit(1);
}
#endif

void Graphe::readFromDimacsGraph(std::string input) {
	std::ifstream infile(input);
	std::string line;
	int numeroLigne = 0;
	int idArete = 0;
	bool unSurDeux = false;
	int idNoeud;
	while (std::getline(infile, line)) {
		idNoeud = numeroLigne - 1;
    	std::istringstream iss(line);
		int numeroMot = 0;
		std::string subs;
		iss >> subs;
		do {
			int idVoisin = std::stoi(subs);
			if (numeroLigne == 0) {
				if (numeroMot == 0) {
					for (int i=0;i<idVoisin;i++) {
						_noeuds.push_back(Noeud(i));
					}
				}
				else if (numeroMot == 2) {
					if (idVoisin == 1) { unSurDeux = true; }
				}
			}
			else {
				if ((!unSurDeux)||(numeroMot%2 == 0)) {
					idVoisin--;
					if (idVoisin > idNoeud) {
						_aretes.push_back(Aretes(&_noeuds[idNoeud],&_noeuds[idVoisin],idArete));
						idArete++;
					}
				}
			}
			iss >> subs;
			numeroMot++;
		} while (iss);
		numeroLigne++;
	}
}

void Graphe::readFromDimacsGraphClean(std::string input) {
	// Lecture de la premiere ligne
	std::vector<int> changeIdNode;
	bool unSurDeux = false;
	int finalNoeud;
	std::ifstream fichier(input);
	std::string ligne;
	std::getline(fichier, ligne);
	std::istringstream ligneStream(ligne);
	std::string mot;
	ligneStream >> mot;
	int nombreNoeud = std::stoi(mot);
	for (int i=0;i<nombreNoeud;i++) {
		changeIdNode.push_back(i);
		finalNoeud = nombreNoeud;
	}
	ligneStream >> mot;
	ligneStream >> mot;
	if (std::stoi(mot) == 1) { unSurDeux = true; }
	fichier.close();

	int increment = -1;
	while (increment != 0) {
		increment = 0;
		std::ifstream infile1(input);
		std::string line1;
		int numeroLigne1 = 0;
		int idNoeud1;
		while (std::getline(infile1, line1)) {
			if (numeroLigne1 > 0) {
				idNoeud1 = numeroLigne1 - 1;
				if (changeIdNode[idNoeud1] != -1) {
					changeIdNode[idNoeud1] -= increment;
					std::istringstream iss(line1);
					int nombreVoisin = 0, nombreMot = 0;
					if ((iss)&&(iss.rdbuf()->in_avail() > 0)) {
						std::string subs;
						iss >> subs;
						do {
							if ((!unSurDeux)||(nombreMot%2==0)) {
								int idVoisin = std::stoi(subs);
								idVoisin--;
								if (changeIdNode[idVoisin] != -1) {
									nombreVoisin++;
								}
							}
							nombreMot++;
							iss >> subs;
						} while (iss);
					}
					if (nombreVoisin <= 1) {
						changeIdNode[idNoeud1] = -1;
						increment++;
						finalNoeud--;
					}
				}
			}
			numeroLigne1++;
		}
		infile1.close();
	}

	for (int i=0;i<finalNoeud;i++) {
		_noeuds.push_back(Noeud(i));
	}
	
	std::ifstream infile(input);
	std::string line;
	int numeroLigne = 0;
	int idArete = 0;
	while (std::getline(infile, line)) {
		if (numeroLigne > 0) {
			int idNoeud = numeroLigne - 1;
			int idNoeudReel = changeIdNode[idNoeud];
			if (idNoeudReel >= 0) {
				std::istringstream iss(line);
				int numeroMot = 0;
				std::string subs;
				iss >> subs;
				do {
					if ((!unSurDeux)||(numeroMot%2 == 0)) {
						int idVoisin = std::stoi(subs);
						idVoisin--;
						int idVoisinReel = changeIdNode[idVoisin];
						if ((idVoisinReel >= 0)&&(idVoisinReel > idNoeudReel)) {
							_aretes.push_back(Aretes(&_noeuds[idNoeudReel],&_noeuds[idVoisinReel],idArete));
							idArete++;
						}
					}
					numeroMot++;
					iss >> subs;
				} while (iss);
			}
		}
		numeroLigne++;
	}
	infile.close();
}

void Graphe::writeToJsonCleanGraphe(std::string output) {
	json j;
	Graphe* G = this;
	if (!isGrapheConnected()) {
		std::vector<int> composante = plusGrandeComposanteConnexe();
		std::sort(composante.begin(),composante.end());
		std::unordered_map<int,int> mapNewIndex;
		G = new Graphe();
		for (int i=0;i<composante.size();i++) {
			mapNewIndex.insert({composante[i],i});
			G->_noeuds.push_back(Noeud(i));
		}
		std::set<std::pair<int,int>> aretesComposante;
		for (int i=0;i<composante.size();i++) {
			for (const int& idArete : _noeuds[composante[i]]._aretes) {
				int idNoeud = mapNewIndex[_aretes[idArete]._noeud1->_id];
				int idVoisin = mapNewIndex[_aretes[idArete]._noeud2->_id];
				if (idNoeud != idVoisin) {
					int idNoeud1, idNoeud2;
					if (idNoeud < idVoisin) { idNoeud1 = idNoeud; idNoeud2 = idVoisin; }
					else { idNoeud1 = idVoisin; idNoeud2 = idNoeud; }
					aretesComposante.insert({idNoeud1,idNoeud2});
				}
			}
		}
		int numeroArete = 0;
		for (const std::pair<int,int>& arete : aretesComposante) {
			G->_aretes.push_back(Aretes(&G->_noeuds[arete.first],&G->_noeuds[arete.second],numeroArete));
			numeroArete++;
		}
	}

	std::map<int,std::set<int>> mapNoeudVoisins;
	for (int i=0;i<G->_noeuds.size();i++) {
		std::set<int> voisins;
		for (const Noeud* voisinNoeud : G->_noeuds[i].voisins) {
			if (G->_noeuds[i]._id != voisinNoeud->_id) {
				voisins.insert(voisinNoeud->_id);
			}
		}
		mapNoeudVoisins.insert({i,voisins});
	}
	bool suppression = true;
	std::vector<int> toDelete;
	while (suppression) {
		toDelete.clear();
		suppression = false;
		for (auto const& [idNoeud, voisins] : mapNoeudVoisins) {
			if (voisins.size() <= 1) {
				suppression = true;
				for (const int& idVoisin : voisins) {
					mapNoeudVoisins[idVoisin].erase(idNoeud);
				}
				toDelete.push_back(idNoeud);
			}
		}
		for (const int& idNoeud : toDelete) {
			mapNoeudVoisins.erase(idNoeud);
		}
	}

	std::unordered_map<int,int> mapNewIndex;
	int i=0;
	for (auto const& [idNoeud, voisins] : mapNoeudVoisins) {
		mapNewIndex.insert({idNoeud,i});
		i++;
	}
	std::set<std::pair<int,int>> aretesComposante;
	i=0;
	for (auto const& [tmpIdNoeud, voisins] : mapNoeudVoisins) {
		int idNoeud = mapNewIndex[tmpIdNoeud];
		j["nodes"][i]["id"] = i;
		if (G->_noeuds[idNoeud].getEmplacement() != nullptr) { j["nodes"][i]["id_slot"] = _noeuds[idNoeud].getEmplacement()->getId(); }
		else { j["nodes"][i]["id_slot"] = -1; }
		for (const int& tmpIdVoisin : voisins) {
			int idVoisin = mapNewIndex[tmpIdVoisin];
			int idNoeud1, idNoeud2;
			if (idNoeud < idVoisin) { idNoeud1 = idNoeud; idNoeud2 = idVoisin; }
			else { idNoeud1 = idVoisin; idNoeud2 = idNoeud; }
			aretesComposante.insert({idNoeud1,idNoeud2});
		}
		i++;
	}
	int numeroArete = 0;
	for (const std::pair<int,int>& arete : aretesComposante) {
		j["edges"][numeroArete]["source"] = arete.first;
		j["edges"][numeroArete]["target"] = arete.second;
		numeroArete++;
	}
	std::ofstream o(output);
	o << std::setw(4) << j << std::endl;
}
