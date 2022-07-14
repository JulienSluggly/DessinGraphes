#ifndef NODEMAP_H
#define NODEMAP_H
#include <vector>
#include "NodeBend.hpp"
extern std::vector<std::vector<bool>> mapPosNode;
extern std::vector<NodeBend*> vectorNodeBends;
extern std::vector<std::vector<std::set<NodeBend*>>> posVectorNodeBend;

// Vecteur qui indique la liste des NodeBend a ces coordonnées.
std::vector<std::vector<std::set<NodeBend*>>> posVectorNodeBend;
// Map qui indique si une node ou un bend se trouve a ses coordonnées.
// mapPosNode[Y][X]
std::vector<std::vector<bool>> mapPosNode;
// Vector qui contient la liste de tout les noeuds et bends d'un graphe
std::vector<NodeBend*> vectorNodeBends;

// Recupere le NodeBend associé à un node
NodeBend* getNodeBendFromNode(node n) {
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		if ((vectorNodeBends[i]->isNode) && (vectorNodeBends[i]->getNode() == n)) {
			return vectorNodeBends[i];
		}
	}
	return nullptr;
}

// Recupere le NodeBend associé à un bend
NodeBend* getNodeBendFromBend(IPoint* p) {
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		if ((!vectorNodeBends[i]->isNode) && (vectorNodeBends[i]->getPoint() == p)) {
			return vectorNodeBends[i];
		}
	}
	return nullptr;
}

#endif