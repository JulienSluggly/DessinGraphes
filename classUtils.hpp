#ifndef CLASSUTILS_HPP
#define CLASSUTILS_HPP

#include <ogdf/basic/Graph_d.h>

// Renvoie une map associant les NodeBend sur vecteur de nodebend passé en parametre a leur coordonnée actuelle
std::map<NodeBend*,std::pair<int,int>> copyGraph(std::vector<NodeBend*> &vectorN1) {
	std::map<NodeBend*, std::pair<int, int>> mapNodeCoord;
	for (int i = 0; i < vectorN1.size(); i++) {
		std::pair<int, int> tmpPair(vectorN1[i]->getX(), vectorN1[i]->getY());
		mapNodeCoord.insert(std::pair<NodeBend*, std::pair<int, int>>(vectorN1[i],tmpPair));
	}
	return mapNodeCoord;
}

// Applique la map de coordonnée passée en parametre
void applyGraph(std::map<NodeBend*, std::pair<int, int>> mapNodeCoord) {
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		NodeBend* nb = vectorNodeBends[i];
		auto it = mapNodeCoord.find(nb);
		int newX = it->second.first;
		int newY = it->second.second;
		removeNodeBendPos(nb, *nb->a_x, *nb->a_y);
		(*nb->a_x) = newX;
		(*nb->a_y) = newY;
		if (nb->isNode) {
			nb->recalculateAdjBendStack();
			posVectorNodeBend[newX][newY].push_front(nb);
		}
		else {
			nb->recalculateIsStacked();
			nb->precedent->recalculateIsStacked();
			nb->suivant->recalculateIsStacked();
			posVectorNodeBend[newX][newY].push_back(nb);
		}
	}
}
#endif