#ifndef NODEMAP_H
#define NODEMAP_H
#include "NodeBend.hpp"
#include <vector>

extern std::vector<std::vector<std::list<NodeBend*>>> posVectorNodeBend;
extern std::vector<NodeBend*> vectorNodeBends;
extern std::vector<Segment*> vectorSegments;
extern std::vector<std::vector<Segment*>> vectorFaceSegment;
extern std::unordered_map<adjEntry, std::pair<int, int>> mapAdjEntryFaces;
extern std::unordered_map<adjEntry, NodeBend*> mapAdjEntryFirstNodeBend;

// Vecteur qui indique la liste des NodeBend a ces coordonnées.
std::vector<std::vector<std::list<NodeBend*>>> posVectorNodeBend;
// Vector qui contient la liste de tout les noeuds et bends d'un graphe
std::vector<NodeBend*> vectorNodeBends;
// Vecteur qui stocke tout les segments du graphe
std::vector<Segment*> vectorSegments;
// Vector qui associe l'index d'une face a un vecteur de segment
std::vector<std::vector<Segment*>> vectorFaceSegment;
// Map de l'adjentry aux numeros des faces gauches et droites de l'adjentry, utilisé pour les node
std::unordered_map<adjEntry, std::pair<int, int>> mapAdjEntryFaces;
// Map de l'adjEntry au premier bend voisin, utilisé pour les node
std::unordered_map<adjEntry, NodeBend*> mapAdjEntryFirstNodeBend;

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
	//std::cout << "Error n1: " << nb1->globalNum << " nb2: " << nb2->globalNum << std::endl;
	return nullptr;
}

// Utilisé par les node pour recuperer le premier segment non nul de l'adjEntry
std::pair<NodeBend*, NodeBend*> getFirstSegmentInAdjEntry(adjEntry adj, NodeBend* src) {
	auto it = mapAdjEntryFirstNodeBend.find(adj);
	NodeBend* nb1 = (*it).second;
	if ((nb1->getX() != src->getX()) || (nb1->getY() != src->getY())) {
		return std::pair<NodeBend*, NodeBend*>(src, nb1);
	}
	else {
		NodeBend* nb2;
		if (nb1->suivant->globalNum == src->globalNum) {
			nb2 = nb1->precedent;
			while ((nb1->getX() == nb2->getX()) && (nb1->getY() == nb2->getY())) {
				nb1 = nb2;
				nb2 = nb2->precedent;
			}
		}
		else {
			nb2 = nb1->suivant;
			while ((nb1->getX() == nb2->getX()) && (nb1->getY() == nb2->getY())) {
				nb1 = nb2;
				nb2 = nb2->suivant;
			}
		}
		return std::pair<NodeBend*, NodeBend*>(nb1, nb2);
	}
}

// Utilisé par les node pour retourner le premier nodebend non stacké sur le node de départ dans une adjentry
NodeBend* getFirstNonStackedNodeBendInAdjEntry(adjEntry adj, NodeBend* src) {
	auto it = mapAdjEntryFirstNodeBend.find(adj);
	NodeBend* nb = (*it).second;
	if (nb->isNode) {
		return nb;
	}
	else {
		bool searchSuivant = (nb->precedent->globalNum == src->globalNum);
		while ((nb->getX() == src->getX()) && (nb->getY() == src->getY())) {
			if (searchSuivant) {
				nb = nb->suivant;
			}
			else {
				nb = nb->precedent;
			}
		}
		return nb;
	}
}

// Recupere les numeros des faces liées a l'adjentry
std::pair<int, int> getAdjEntryFaces(adjEntry a) {
	auto it = mapAdjEntryFaces.find(a);
	return (it->second);
}

#endif