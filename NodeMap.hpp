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
extern std::unordered_map<int, int> mapIndexId;
extern std::unordered_map<int, node> mapIdNode;
extern std::vector<std::vector<int>> vectorSegmentBool;

// Vecteur qui indique la liste des NodeBend a ces coordonn�es.
std::vector<std::vector<std::list<NodeBend*>>> posVectorNodeBend;
// Vector qui contient la liste de tout les noeuds et bends d'un graphe
std::vector<NodeBend*> vectorNodeBends;
// Vecteur qui stocke tout les segments du graphe
std::vector<Segment*> vectorSegments;
// Vector qui associe l'index d'une face a un vecteur de segment
std::vector<std::vector<Segment*>> vectorFaceSegment;
// Map de l'adjentry aux numeros des faces gauches et droites de l'adjentry, utilis� pour les node
std::unordered_map<adjEntry, std::pair<int, int>> mapAdjEntryFaces;
// Map de l'adjEntry au premier bend voisin, utilis� pour les node
std::unordered_map<adjEntry, NodeBend*> mapAdjEntryFirstNodeBend;
// Map de l'index d'un node a son ID original
std::unordered_map<int, int> mapIndexId;
// Map de l'id original d'un node a la node
std::unordered_map<int, node> mapIdNode;
// Vecteur de vecteur d'entier indiquant la position d'un segment entre deux nodebend de globalnum diff�rents dans le vecteur global vectorSegments, -1 si ce segment n'existe pas.0
std::vector<std::vector<int>> vectorSegmentBool;

// Recupere le NodeBend associ� � un node
NodeBend* getNodeBendFromNode(node n) {
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		if ((vectorNodeBends[i]->isNode) && (vectorNodeBends[i]->getNode() == n)) {
			return vectorNodeBends[i];
		}
	}
	return nullptr;
}

// Recupere le NodeBend associ� � un bend
NodeBend* getNodeBendFromBend(IPoint* p) {
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		if ((!vectorNodeBends[i]->isNode) && (vectorNodeBends[i]->getPoint() == p)) {
			return vectorNodeBends[i];
		}
	}
	return nullptr;
}

// Renvoie le segment associ� a deux nodebend s'il existe
Segment* getSegmentFromNodeBends(NodeBend* nb1, NodeBend* nb2) {
	for (int i = 0; i < vectorSegments.size(); i++) {
		if (((vectorSegments[i]->source->globalNum == nb1->globalNum) && (vectorSegments[i]->target->globalNum == nb2->globalNum)) || ((vectorSegments[i]->target->globalNum == nb1->globalNum) && (vectorSegments[i]->source->globalNum == nb2->globalNum))) {
			return vectorSegments[i];
		}
	}
	std::cout << "OLD VESRION Error n1: " << nb1->globalNum << " nb2: " << nb2->globalNum << std::endl;
	return nullptr;
}

// Renvoie le segment associ� a deux nodebend s'il existe
Segment* getSegmentFromNodeBendsV2(NodeBend* nb1, NodeBend* nb2) {
	int numero = vectorSegmentBool[nb1->globalNum][nb2->globalNum];
	if (numero != -1) {
		return vectorSegments[numero];
	}
	std::cout << "Error n1: " << nb1->globalNum << " nb2: " << nb2->globalNum << std::endl;
	return nullptr;
}

// Utilis� par les node pour recuperer le premier segment non nul de l'adjEntry
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

// Utilis� par les node pour recuperer le premier segment non nul de l'adjEntry
Segment* getFirstSegmentInAdjEntryV2(adjEntry adj, NodeBend* src) {
	auto it = mapAdjEntryFirstNodeBend.find(adj);
	NodeBend* nb1 = (*it).second;
	if ((nb1->getX() != src->getX()) || (nb1->getY() != src->getY())) {
		return getSegmentFromNodeBendsV2(src, nb1);
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
		return getSegmentFromNodeBendsV2(nb1, nb2);
	}
}

// Utilis� par les node pour retourner le premier nodebend non stack� sur le node de d�part dans une adjentry
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

// Recupere les numeros des faces li�es a l'adjentry
std::pair<int, int> getAdjEntryFaces(adjEntry a) {
	auto it = mapAdjEntryFaces.find(a);
	return (it->second);
}

// Indique si le NodeBend n'est pas seul sur sa grille
bool isStackedInGrid(NodeBend* nb) {
	return (posVectorNodeBend[nb->getX()][nb->getY()].size() > 1);
}

#endif