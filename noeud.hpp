#ifndef NOEUD_HPP
#define NOEUD_HPP

#include "emplacement.hpp"
#include <vector>
#include <iostream>
#include <sstream>
#include <mutex>

class Noeud {
public:
	Emplacement* _emplacement = nullptr; // L'emplacement sur lequel le noeud est placé, utile en coord entiere uniquement
	std::vector<int> _aretes; // Contient les indices des aretes contenant ce noeud
	int _id; // Id du noeud dans le tableau du graphe, unique et >= 0
	std::vector<Noeud*> voisins; // Voisins directs relié par une arete
	std::mutex* mutexAreteVoisin = nullptr;
	Noeud(int id) { _id = id; mutexAreteVoisin = new std::mutex(); }
	
	// Attention ce score n'est pas toujours a jour!
	long score = -1; // Score d'intersection du noeud

	int ogdfId = -1; // Id du noeud dans ogdf, utile uniquement dans l'algo ogdf.

	double stressX = -1;
	double stressY = -1;
	double pivotX = -1;
	double pivotY = -1;
	double _xreel = -12345;
	double _yreel = -12345;
	std::vector<int>* idCelluleVec = nullptr;
	
	int areteCommune(Noeud* noeudVoisin) {
		for (const int& idArete1 : _aretes) {
			for (const int& idArete2 : noeudVoisin->_aretes) {
				if (idArete1 == idArete2) { return idArete1; }
			}
		}
		return -1;
	}
	bool estPlace() const { return _emplacement != nullptr; }
	Emplacement* getEmplacement()  const { return _emplacement; }
	void connect(Noeud* noeud) { voisins.push_back(noeud); }
	std::vector<Noeud*> getVoisins() const { return voisins; }
	bool voisinsSontPlaces() const {
		for (int i=0;i<voisins.size();i++) {
			if (!voisins[i]->estPlace()) return false;
		}
		return true;
	}
	int getVoisinsPlaces() const {
		int nbVoisinPlaces = 0;
		for (int i=0;i<voisins.size();i++) {
			if (voisins[i]->estPlace()) ++nbVoisinPlaces;
		}
		return nbVoisinPlaces;
	}
	int getX()  const { return _emplacement->getX(); }
	int getY()  const { return _emplacement->getY(); }
	// Indique si deux noeuds ont les mêmes coordonnées
	bool compare(const Noeud* noeud) const {
		return ((getX() == noeud->getX()) && (getY() == noeud->getY()));
	}
	int getId() const { return _id; }

	void setEmplacement(Emplacement* emplacement) {
		if (_emplacement != nullptr) {
			_emplacement->removeNoeud();
		}
		_emplacement = emplacement;
		emplacement->setNoeud(this);
	}

	void setCoordReel(std::pair<double,double>& coord) {
		_xreel = coord.first;
		_yreel = coord.second;
	}

	// Force le noeud à l'emplacement en enlevant l'ancien noeud s'il y en avait un.
	// Renvoie l'id de l'ancien noeud présent sur l'emplacement ou -1 s'il n'y en avait pas.
	int ecraseNoeud(Emplacement& emplacement) {
		int oldNodeId = -1;
		if (_emplacement != nullptr) {
			_emplacement->removeNoeud();
		}
		if (emplacement._noeud != nullptr) {
			oldNodeId = emplacement._noeud->getId();
			emplacement._noeud->clearEmplacement();
		}
		_emplacement = &emplacement;
		emplacement.setNoeud(this);
		return oldNodeId;
	}

	void clearEmplacement() {
		if (_emplacement != nullptr) {
			_emplacement->removeNoeud();
		}
		_emplacement = nullptr;
	}

	void swap(Emplacement* emp2) {
		Noeud* n2 = emp2->_noeud;
		Emplacement* emp1 = this->_emplacement;
		this->clearEmplacement();
		n2->clearEmplacement();
		this->setEmplacement(emp2);
		n2->setEmplacement(emp1);
	}

	std::string voisinString() {
		std::stringstream paramStream;
		paramStream << "{";
		for (int i=0;i<voisins.size();i++) {
			paramStream << voisins[i]->_id;
			if (i<voisins.size()-1) { paramStream << " "; }
		}
		paramStream << "}";
		return paramStream.str();
	}

};


#endif