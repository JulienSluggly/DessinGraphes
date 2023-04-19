#ifndef ARETES_HPP
#define ARETES_HPP

#include "noeud.hpp"
#include <unordered_set>

class Aretes {
public:
	Noeud* _noeud1, *_noeud2;
	int _id;
	Aretes(Noeud* noeud1, Noeud* noeud2, int id) {
		_noeud1 = noeud1;
		_noeud2 = noeud2;
		_id = id;
		noeud1->_aretes.push_back(id);
		noeud2->_aretes.push_back(id);
		noeud1->connect(noeud2);
		noeud2->connect(noeud1);
	}
	// Vecteur contenant les id des aretes ayant une intersection legale avec celle-ci.
	// Attention ce vecteur n'est pas toujours a jour!
	std::unordered_set<int> intersections;
	// Vecteur contenant les id des aretes ne contenant pas de noeud commun ayant une intersection illegale avec celle-ci.
	// Attention ce vecteur n'est pas toujours a jour!
	std::unordered_set<int> intersectionsIll;
	// Vecteur contenant les id des aretes contenant un noeud commun ayant une intersection illegale avec celle-ci.
	// Attention ce vecteur n'est pas toujours a jour!
	std::unordered_set<int> intersectionsIllSelf;
	bool isUpdated = false;

	// Utile pour le calcul improve multi threadé. Prend la valeur 0, 1 ou 2.
	// 0 = Arete commune aux threads
	// 1 = ancienne arete, utile pour le thread improve avant deplacement
	// 2 = nouvelle arete, utile pour le thread improve apres deplacement
	int typeArrete = 0; 

	std::vector<int> vecIdCellules; // Contient les id des cellules dans la grille du graphe

	double edgeLength = 0.0; // Pour OGDFFMMM, a supprimer si pas utile

	Noeud* getNoeud1()  const { return _noeud1; }
	Noeud* getNoeud2()  const { return _noeud2; }

	void setNoeud1(Noeud* noeud1) { _noeud1 = noeud1; }
	void setNoeud2(Noeud* noeud2) { _noeud2 = noeud2; }

	bool contains(Noeud* noeud) const {
		return (_noeud1->getId() == noeud->getId() || _noeud2->getId() == noeud->getId());
	}

	bool contains(int id) {
		return (_noeud1->getId() == id || _noeud2->getId() == id);
	}

	// Renvoie le noeud qui n'est pas en commun avec l'arete passe en parametre
	Noeud* nodeNotInCommon(const Aretes* l) const {
		if (_noeud1->getId() == l->getNoeud1()->getId() || _noeud1->getId() == l->getNoeud2()->getId()) { return _noeud2; }
		return _noeud1;
	}

	// Retourne vrai si les deux noeuds sont placés
	bool estPlace() {
		return (_noeud1->getEmplacement() != nullptr && _noeud2->getEmplacement() != nullptr);
	}

	// Retourne vrai si au moin un noeud est placé
	bool unPlace() {
		return (_noeud1->getEmplacement() != nullptr || _noeud2->getEmplacement() != nullptr);
	}

	// Vide les sets d'intersections
	void clearIntersectionsVector() {
		intersections.clear();
		intersectionsIll.clear();
		intersectionsIllSelf.clear();
		isUpdated = false;
	}

};

#endif