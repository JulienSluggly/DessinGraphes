#ifndef NODEMAP_H
#define NODEMAP_H
#include <vector>
#include "NodeBend.hpp"
extern std::vector<std::vector<bool>> mapPosNode;
extern std::vector<NodeBend*> vectorNodeBends;
extern std::vector<std::vector<std::set<NodeBend*>>> posVectorNodeBend;
extern std::vector<Segment*> vectorSegments;
extern std::vector<std::vector<Segment*>> vectorFaceSegment;

// Vecteur qui indique la liste des NodeBend a ces coordonnées.
std::vector<std::vector<std::set<NodeBend*>>> posVectorNodeBend;
// Map qui indique si une node ou un bend se trouve a ses coordonnées.
// mapPosNode[Y][X]
std::vector<std::vector<bool>> mapPosNode;
// Vector qui contient la liste de tout les noeuds et bends d'un graphe
std::vector<NodeBend*> vectorNodeBends;
// Vecteur qui stocke tout les segments du graphe
std::vector<Segment*> vectorSegments;
// Vector qui associe l'index d'une face a un vecteur de segment
std::vector<std::vector<Segment*>> vectorFaceSegment;

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

// Renvoie le segment associé a deux nodebend s'il existe
Segment* getSegmentFromNodeBends(NodeBend* nb1, NodeBend* nb2) {
	for (int i = 0; i < vectorSegments.size(); i++) {
		if (((vectorSegments[i]->source->globalNum == nb1->globalNum) && (vectorSegments[i]->target->globalNum == nb2->globalNum)) || ((vectorSegments[i]->target->globalNum == nb1->globalNum) && (vectorSegments[i]->source->globalNum == nb2->globalNum))) {
			return vectorSegments[i];
		}
	}
	return nullptr;
}

#endif