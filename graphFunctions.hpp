#ifndef GRAPHFUNCTIONS_HPP
#define GRAPHFUNCTIONS_HPP

#include <ogdf/basic/Graph_d.h>
#include <ogdf/basic/GridLayout.h>
#include <ogdf/basic/Layout.h>
#include <ogdf/basic/CombinatorialEmbedding.h>
#include <stdio.h>
#include "geometrie.hpp"
#include "optimAlg.hpp"
#include "intersection.hpp"
#include "calcEdgeLength.hpp"
#include "EdgeMap.hpp"
#include "NodeBend.hpp"
#include <random>

using namespace ogdf;

// Retourne une valeur entiere comprise dans [1,n]
int generateRand(int n) {
	std::random_device rd;  // Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<> dis(1, n);
	return dis(gen);
}

// Enleve le nodebend du set global pour les positions
void removeNodeBendPos(NodeBend* nb, int x, int y) {
	auto it = posVectorNodeBend[x][y].begin();
	it = posVectorNodeBend[x][y].find(nb);
	posVectorNodeBend[x][y].erase(it);
}

// Renvoie un vecteur composé de tout les segments qui composent les faces adjacentes a un adjentry
std::vector<Segment> getSegmentFromAdjFacesFromAdjEntry(adjEntry& adj, ConstCombinatorialEmbedding& ccem, GridLayout& GL) {
	std::set<edge> setAllEdges;
	std::set<face> setAdjFaces;
	std::vector<Segment> vectorSegment;
	// On recupere la face gauche et droite de l'adjentry
	setAdjFaces.insert(ccem.rightFace(adj));
	setAdjFaces.insert(ccem.leftFace(adj));
	// On ajoute tout les edge dans un set en premier temps pour eviter les doublons
	for (auto it = setAdjFaces.begin(); it != setAdjFaces.end(); it++) {
		adjEntry firstAdj = (*it)->firstAdj();
		adjEntry nextAdj = firstAdj;
		if (firstAdj != nullptr) {
			do {
				if (nextAdj->theEdge() != nullptr) {
					setAllEdges.insert(nextAdj->theEdge());
				}
				nextAdj = (*it)->nextFaceEdge(nextAdj);
			} while ((nextAdj != firstAdj) && (nextAdj != nullptr));
		}
	}
	// On parcour tout les edges et on les transforme en segments.
	int edgeSrcX, edgeSrcY, edgeTrgX, edgeTrgY;
	for (auto it = setAllEdges.begin(); it != setAllEdges.end(); it++) {
		edgeSrcX = GL.x((*it)->source());
		edgeSrcY = GL.y((*it)->source());
		IPolyline& bends = GL.bends((*it));
		for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
			edgeTrgX = (*i).m_x;
			edgeTrgY = (*i).m_y;
			//if ((edgeSrcX != edgeTrgX) || (edgeSrcY != edgeTrgY)) {
			Segment tmpSegmentEdge(&edgeSrcX, &edgeSrcY, &edgeTrgX, &edgeTrgY);
			vectorSegment.push_back(tmpSegmentEdge);
			//}
			edgeSrcX = edgeTrgX;
			edgeSrcY = edgeTrgY;
		}
		edgeTrgX = GL.x((*it)->target());
		edgeTrgY = GL.y((*it)->target());
		//if ((edgeSrcX != edgeTrgX) || (edgeSrcY != edgeTrgY)) {
		Segment tmpSegmentEdge(&edgeSrcX, &edgeSrcY, &edgeTrgX, &edgeTrgY);
		vectorSegment.push_back(tmpSegmentEdge);
		//}
	}
	return vectorSegment;
}

// Renvoie un set composé de tout les edges qui composent les faces adjacentes a un NodeBend
std::set<edge> getEdgesFromAdjFacesFromNodeBend(NodeBend* n, ConstCombinatorialEmbedding& ccem) {
	std::set<edge> setAllEdges;
	std::set<face> setAdjFaces;
	if (n->isNode) {
		SListPure<edge> edges;
		n->getNode()->adjEdges(edges);
		for (SListConstIterator<edge> i = edges.begin(); i.valid(); i++) {
			edge e = (*i);
			setAdjFaces.insert(ccem.leftFace(e->adjSource()));
			setAdjFaces.insert(ccem.rightFace(e->adjSource()));
		}
	}
	else {
		setAdjFaces.insert(ccem.rightFace(n->getEdge()->adjSource()));
		setAdjFaces.insert(ccem.leftFace(n->getEdge()->adjSource()));
	}
	for (auto it = setAdjFaces.begin(); it != setAdjFaces.end(); it++) {
		adjEntry firstAdj = (*it)->firstAdj();
		adjEntry nextAdj = firstAdj;
		if (firstAdj != nullptr) {
			do {
				if (nextAdj->theEdge() != nullptr) {
					setAllEdges.insert(nextAdj->theEdge());
				}
				nextAdj = (*it)->nextFaceEdge(nextAdj);
			} while ((nextAdj != firstAdj) && (nextAdj != nullptr));
		}
	}
	return setAllEdges;
}

std::vector<NodeBend*> getTargetNodeBends(NodeBend* n) {
	std::vector<NodeBend*> bendPrecSuiv;
	NodeBend* precedent = n->precedent;
	bool found = false;
	do {
		if ((precedent->getX() != n->getX()) || (precedent->getY() != n->getY())) {
			//std::cout << "Precedent x: " << *precedent->a_x << " y: " << *precedent->a_y << std::endl;
			bendPrecSuiv.push_back(precedent);
			found = true;
		}
		else {
			if (precedent->isNode) {
				found = true;
			}
			else {
				precedent = precedent->precedent;
			}
		}
	} while (!found);
	NodeBend* suivant = n->suivant;
	found = false;
	do {
		if ((*suivant->a_x != *n->a_x) || (*suivant->a_y != *n->a_y)) {
			//std::cout << "Suivant x: " << *suivant->a_x << " y: " << *suivant->a_y << std::endl;
			bendPrecSuiv.push_back(suivant);
			found = true;
		}
		else {
			if (suivant->isNode) {
				found = true;
			}
			else {
				suivant = suivant->suivant;
			}
		}
	} while (!found);
	return bendPrecSuiv;
}

// Recupere les coordonnées du points target (node ou bend) a partir d'un node source (ici contenu dans le adjEntry) et renvoie le nodebend associé
NodeBend* getTargetCoordAndNodeBend(GridLayout& GL, const adjEntry& adj, int& trgX, int& trgY) {
	edge tmpEdge = adj->theEdge();
	IPolyline& p = GL.bends(tmpEdge);
	// Si l'edge contient des bends
	if (p.size() > 0) {
		bool found = false;
		// Si le noeud source est le meme, on prend le premier bend
		if (tmpEdge->source() == adj->theNode()) {
			auto it = p.begin();
			while (it.valid() && !found) {
				if (((*it).m_x != GL.x(adj->theNode())) || ((*it).m_y != GL.y(adj->theNode()))) {
					trgX = (*it).m_x;
					trgY = (*it).m_y;
					found = true;
					return getNodeBendFromBend(&(*it));
				}
				it++;
			}
		}
		// Sinon on prend le dernier bend
		else {
			auto it = p.rbegin();
			while (it.valid() && !found) {
				if (((*it).m_x != GL.x(adj->theNode())) || ((*it).m_y != GL.y(adj->theNode()))) {
					trgX = (*it).m_x;
					trgY = (*it).m_y;
					found = true;
					return getNodeBendFromBend(&(*it));
				}
				it++;
			}
		}
		// Si tout les bends sont stackés
		if (!found) {
			node tmpNode = adj->twinNode();
			trgX = GL.x(tmpNode);
			trgY = GL.y(tmpNode);
			return getNodeBendFromNode(tmpNode);
		}
	}
	// Si pas de bends on prends les coordonnées du noeud
	else {
		node tmpNode = adj->twinNode();
		trgX = GL.x(tmpNode);
		trgY = GL.y(tmpNode);
		return getNodeBendFromNode(tmpNode);
	}
}

// Recupere les coordonnées du points target (node ou bend) a partir d'un node source (ici contenu dans le adjEntry)
void getTargetCoord(GridLayout& GL, const adjEntry& adj, int& trgX, int& trgY) {
	edge tmpEdge = adj->theEdge();
	IPolyline& p = GL.bends(tmpEdge);
	// Si l'edge contient des bends
	if (p.size() > 0) {
		bool found = false;
		// Si le noeud source est le meme, on prend le premier bend
		if (tmpEdge->source() == adj->theNode()) {
			auto it = p.begin();
			while (it.valid() && !found) {
				if (((*it).m_x != GL.x(adj->theNode())) || ((*it).m_y != GL.y(adj->theNode()))) {
					trgX = (*it).m_x;
					trgY = (*it).m_y;
					found = true;
				}
				it++;
			}
		}
		// Sinon on prend le dernier bend
		else {
			auto it = p.rbegin();
			while (it.valid() && !found) {
				if (((*it).m_x != GL.x(adj->theNode())) || ((*it).m_y != GL.y(adj->theNode()))) {
					trgX = (*it).m_x;
					trgY = (*it).m_y;
					found = true;
				}
				it++;
			}
		}
		// Si tout les bends sont stackés
		if (!found) {
			node tmpNode = adj->twinNode();
			trgX = GL.x(tmpNode);
			trgY = GL.y(tmpNode);
		}
	}
	// Si pas de bends on prends les coordonnées du noeud
	else {
		node tmpNode = adj->twinNode();
		trgX = GL.x(tmpNode);
		trgY = GL.y(tmpNode);
	}
}

// Renvoie l'ordre des adjentry autour d'un noeud apres déplacement d'un noeud adjacent
ListPure<adjEntry> orderAroundNodeAfterAdjNodeMove(node nsrc, GridLayout& GL, ListPure<adjEntry> adj, adjEntry moved, int newX, int newY) {
	ListPure<adjEntry> newOrder;
	// Coordonnées: s = source, t = target, n = node a ajouter
	int sx, sy, tx, ty, nx, ny;
	sx = GL.x(nsrc);
	sy = GL.y(nsrc);
	// Itérateur qui itere sur le tableau des adjacent non trié
	auto it = adj.begin();
	// On insere le premier element et on passe directement au prochain
	newOrder.pushBack((*it));
	it++;
	// On itere sur tout les éléments de la liste non triée
	for (; it.valid(); it++) {
		bool inserted = false;
		edge tmpEdge2;
		int qnewnode;
		if ((*it) == moved) {
			nx = newX;
			ny = newY;
		}
		else {
			getTargetCoord(GL, (*it), nx, ny);
		}
		qnewnode = quadrant(sx, sy, nx, ny);
		// On itere sur le deuxieme tableau tant qu'on est pas a la fin et tant qu'on a pas inseré
		for (auto it2 = newOrder.begin(); ((it2.valid()) && (!inserted)); it2++) {
			if ((*it2) == moved) {
				tx = newX;
				ty = newY;
			}
			else {
				getTargetCoord(GL, (*it2), tx, ty);
			}
			// Quadrant du noeud/premier bend
			int qtrg = quadrant(sx, sy, tx, ty);
			// Si le quadrant du point que l'on veut inserer est inférieur a celui qu'on compare
			if (qnewnode < qtrg) {
				// Si on compare au premier, on insere en premiere place
				if (it2 == newOrder.begin()) {
					newOrder.pushFront((*it));
				}
				else {
					it2--;
					newOrder.insertAfter((*it), it2);
					it2++;
				}
				inserted = true;
			}
			// Si les quadrants sont égaux
			else if (qnewnode == qtrg) {
				// Si on est a droite on insere apres it3
				if (!aGauche(sx, sy, tx, ty, nx, ny)) {
					if (it2 == newOrder.begin()) {
						newOrder.pushFront((*it));
					}
					else {
						it2--;
						newOrder.insertAfter((*it), it2);
						it2++;
					}
					inserted = true;
				}
			}
		}
		// Si on a toujours pas inseré, c'est qu'on est le dernier element
		if (!inserted) {
			newOrder.pushBack((*it));
		}
	}
	return newOrder;
}

// Renvoie l'ordre des adjentry autour d'un noeud apres déplacement de ce noeud
ListPure<adjEntry> orderAroundNodeAfterMove(node nsrc, GridLayout& GL, ListPure<adjEntry> adj, int newX, int newY) {
	ListPure<adjEntry> newOrder;
	// Coordonnées: s = source, t = target, n = node a ajouter
	int sx, sy, tx, ty, nx, ny;
	sx = newX;
	sy = newY;
	// Itérateur qui itere sur le tableau des adjacent non trié
	auto it = adj.begin();
	// On insere le premier element et on passe directement au prochain
	newOrder.pushBack((*it));
	it++;
	// On itere sur tout les éléments de la liste non triée
	for (; it.valid(); it++) {
		bool inserted = false;
		edge tmpEdge2;
		int qnewnode;
		getTargetCoord(GL, (*it), nx, ny);
		qnewnode = quadrant(sx, sy, nx, ny);
		// On itere sur le deuxieme tableau tant qu'on est pas a la fin et tant qu'on a pas inseré
		for (auto it2 = newOrder.begin(); ((it2.valid()) && (!inserted)); it2++) {
			getTargetCoord(GL, (*it2), tx, ty);
			// Quadrant du noeud/premier bend
			int qtrg = quadrant(sx, sy, tx, ty);
			// Si le quadrant du point que l'on veut inserer est inférieur a celui qu'on compare
			if (qnewnode < qtrg) {
				// Si on compare au premier, on insere en premiere place
				if (it2 == newOrder.begin()) {
					newOrder.pushFront((*it));
				}
				else {
					it2--;
					newOrder.insertAfter((*it), it2);
					it2++;
				}
				inserted = true;
			}
			// Si les quadrants sont égaux
			else if (qnewnode == qtrg) {
				// Si on est a droite on insere apres it3
				if (!aGauche(sx, sy, tx, ty, nx, ny)) {
					if (it2 == newOrder.begin()) {
						newOrder.pushFront((*it));
					}
					else {
						it2--;
						newOrder.insertAfter((*it), it2);
						it2++;
					}
					inserted = true;
				}
			}
		}
		// Si on a toujours pas inseré, c'est qu'on est le dernier element
		if (!inserted) {
			newOrder.pushBack((*it));
		}
	}
	return newOrder;
}

// Renvoie l'ordre des adjentry autour d'un noeud
ListPure<adjEntry> orderAroundNode(node nsrc, GridLayout& GL, ListPure<adjEntry> adj) {
	ListPure<adjEntry> newOrder;
	// Coordonnées: s = source, t = target, n = node a ajouter
	int sx, sy, tx, ty, nx, ny;
	sx = GL.x(nsrc);
	sy = GL.y(nsrc);
	// Itérateur qui itere sur le tableau des adjacent non trié
	auto it = adj.begin();
	// On insere le premier element et on passe directement au prochain
	newOrder.pushBack((*it));
	it++;
	// On itere sur tout les éléments de la liste non triée
	for (; it.valid(); it++) {
		bool inserted = false;
		edge tmpEdge2;
		int qnewnode;
		getTargetCoord(GL, (*it), nx, ny);
		qnewnode = quadrant(sx, sy, nx, ny);
		// On itere sur le deuxieme tableau tant qu'on est pas a la fin et tant qu'on a pas inseré
		for (auto it2 = newOrder.begin(); ((it2.valid()) && (!inserted)); it2++) {
			getTargetCoord(GL, (*it2), tx, ty);
			// Quadrant du noeud/premier bend
			int qtrg = quadrant(sx, sy, tx, ty);
			// Si le quadrant du point que l'on veut inserer est inférieur a celui qu'on compare
			if (qnewnode < qtrg) {
				// Si on compare au premier, on insere en premiere place
				if (it2 == newOrder.begin()) {
					newOrder.pushFront((*it));
				}
				else {
					it2--;
					newOrder.insertAfter((*it), it2);
					it2++;
				}
				inserted = true;
			}
			// Si les quadrants sont égaux
			else if (qnewnode == qtrg) {
				// Si on est a droite on insere apres it3
				if (!aGauche(sx, sy, tx, ty, nx, ny)) {
					if (it2 == newOrder.begin()) {
						newOrder.pushFront((*it));
					}
					else {
						it2--;
						newOrder.insertAfter((*it), it2);
						it2++;
					}
					inserted = true;
				}
			}
		}
		// Si on a toujours pas inseré, c'est qu'on est le dernier element
		if (!inserted) {
			newOrder.pushBack((*it));
		}
	}
	return newOrder;
}

// Renvoie vrai si les deux listes ont les memes suivants/precedent de leur contenu (ABC == BCA)
// Fonction utilisée pour des listes ayant les memes adjentry en contenu mais pas forcément dans le meme ordre
bool sameOrderList(ListPure<adjEntry> l1, ListPure<adjEntry> l2) {
	ListIterator<adjEntry> it = l1.begin();
	ListIterator<adjEntry> it2 = l2.begin();
	for (; (*it2) != (*it); it2++);
	while (l1.cyclicSucc(it) != l1.begin()) {
		it = l1.cyclicSucc(it);
		it2 = l2.cyclicSucc(it2);
		if ((*it) != (*it2)) {
			return false;
		}
	}
	return true;
}

bool hasBends(adjEntry adj, GridLayout& GL) {
	edge tmpEdge = adj->theEdge();
	IPolyline& p = GL.bends(tmpEdge);
	// Si l'edge contient des bends
	return (p.size() > 0);
}

// Renvoie vrai si le noeud n ou un de ses noeuds adjacent a un ordre différent apres le déplacement du node n
bool orderNodeAdjChanged(NodeBend* nb, GridLayout& GL, int newX, int newY) {
	if (nb->isNode) {
		int degre = nb->getNode()->degree();
		node n = nb->getNode();
		// On regarde l'ordre autour du noeud
		if (degre >= 3) {
			ListPure<adjEntry> nodeAdjEntries;
			n->allAdjEntries(nodeAdjEntries);
			ListPure<adjEntry> newAdjEntriesOrder;
			n->allAdjEntries(newAdjEntriesOrder);
			newAdjEntriesOrder = orderAroundNodeAfterMove(n, GL, newAdjEntriesOrder, newX, newY);
			if (!sameOrderList(nodeAdjEntries, newAdjEntriesOrder)) {
				return true;
			}
		}
		ListPure<adjEntry> nodeAdjEntries;
		n->allAdjEntries(nodeAdjEntries);
		ListPure<adjEntry> adjNodeAdjEntries;
		ListPure<adjEntry> adjNodeNewAdjEntriesOrder;
		// On regarde l'ordre de tout les noeuds adjacents
		for (auto it = nodeAdjEntries.begin(); it.valid(); it++) {
			node na = (*it)->twinNode();
			// Uniquement si le degree >=3 l'ordre peut changer et que l'edge ne contient pas de bend
			if ((!hasBends((*it), GL)) && (na->degree() >= 3)) {
				adjNodeAdjEntries.clear();
				adjNodeNewAdjEntriesOrder.clear();
				adjEntry oppose = (*it)->twin();
				adjNodeAdjEntries.pushBack(oppose->cyclicPred());
				adjNodeAdjEntries.pushBack(oppose);
				adjNodeAdjEntries.pushBack(oppose->cyclicSucc());
				adjNodeNewAdjEntriesOrder.pushBack(oppose->cyclicPred());
				adjNodeNewAdjEntriesOrder.pushBack(oppose);
				adjNodeNewAdjEntriesOrder.pushBack(oppose->cyclicSucc());
				adjNodeNewAdjEntriesOrder = orderAroundNodeAfterAdjNodeMove(na, GL, adjNodeNewAdjEntriesOrder, oppose, newX, newY);
				if (!sameOrderList(adjNodeAdjEntries, adjNodeNewAdjEntriesOrder)) {
					return true;
				}
			}
		}
	}
	// Si on est un bend on regarde si un noeud est directement adjacent ou non
	else {
		adjEntry adj = nb->getAdjEntry();
		edge e = nb->getEdge();
		IPolyline& bends = GL.bends(e);
		std::vector<node> adjNodes;
		// Numero 0 = on est adjacent au noeud source de l'edge
		if (nb->numero == 0) {
			adjNodes.push_back(e->source());
		}
		// Numero = bends.size() - 1 = on est adjacent au noeud target de l'edge
		if (nb->numero == bends.size() - 1) {
			adjNodes.push_back(e->target());
		}
		for (int i = 0; i < adjNodes.size(); i++) {
			// Pas de changement d'ordre possible si degre < 3
			if (adjNodes[i]->degree() >= 3) {
				// On recupere l'adjentry du bon coté
				if (adj->theNode() != adjNodes[i]) {
					adj = adj->twin();
				}
				ListPure<adjEntry> adjNodeAdjEntries;
				ListPure<adjEntry> adjNodeNewAdjEntriesOrder;
				adjNodeAdjEntries.clear();
				adjNodeNewAdjEntriesOrder.clear();
				adjNodeAdjEntries.pushBack(adj->cyclicPred());
				adjNodeAdjEntries.pushBack(adj);
				adjNodeAdjEntries.pushBack(adj->cyclicSucc());
				adjNodeNewAdjEntriesOrder.pushBack(adj->cyclicPred());
				adjNodeNewAdjEntriesOrder.pushBack(adj);
				adjNodeNewAdjEntriesOrder.pushBack(adj->cyclicSucc());
				adjNodeNewAdjEntriesOrder = orderAroundNodeAfterAdjNodeMove(adjNodes[i], GL, adjNodeNewAdjEntriesOrder, adj, newX, newY);
				if (!sameOrderList(adjNodeAdjEntries, adjNodeNewAdjEntriesOrder)) {
					return true;
				}
			}
		}
	}
	return false;
}

// Renvoie un vecteur de booléen de meme taille que "vectorMoveCoord". Ces booleen indiquent si le déplacement a la meme position dans ce vecteur est valide ou non.
std::vector<bool> getLegalMoves(NodeBend* n, GridLayout& GL, std::vector<std::pair<int, int>> vectorMoveCoord, ConstCombinatorialEmbedding& ccem) {
	// Vecteur qui indique si un déplacement est autorisé ou non
	std::vector<bool> vectorMoveAutorised;
	// On verifie en premier si le NodeBend peut se deplacer (on verifie le stacking)
	if (n->isNode) {
		if (n->nbDiffAdjStacked > 1) {
			for (int i = 0; i < vectorMoveCoord.size(); i++) {
				vectorMoveAutorised.push_back(false);
			}
			return vectorMoveAutorised;
		}
	}
	else {
		// On verifie que l'on ne soit pas stacké avec le précédent et le suivant
		if ((n->a_x == n->precedent->a_x) && (n->a_y == n->precedent->a_y) && (n->a_x == n->suivant->a_x) && (n->a_y == n->suivant->a_y)) {
			for (int i = 0; i < vectorMoveCoord.size(); i++) {
				vectorMoveAutorised.push_back(false);
			}
			return vectorMoveAutorised;
		}
	}
	int srcX = (*n->a_x);
	int srcY = (*n->a_y);
	int trgX, trgY;
	bool intersection;
	SList<adjEntry> nodeAdjEntries;
	// Node a toujours le meme nombre d'adjEntry
	if (n->isNode) {
		n->getNode()->allAdjEntries(nodeAdjEntries);
	}
	// Pour chaque déplacement, on regarde si il y a une intersection associé
	int segmentSourceTrgX, segmentSourceTrgY;
	// On recupere les segments des faces adjacentes du NodeBend
	std::vector<Segment> vectorAdjFaceSegments = n->adjFaceSegment;
	// Affichage debug
	for (int i = 0; i < vectorAdjFaceSegments.size(); i++) {
		//std::cout << "Segment: " << i << " x1: " << *vectorAdjFaceSegments[i].sourceX << " y1: " << *vectorAdjFaceSegments[i].sourceY << " x2: " << *vectorAdjFaceSegments[i].targetX << " y2: " << *vectorAdjFaceSegments[i].targetY << std::endl;
	}
	removeNodeBendPos(n, srcX, srcY);
	// Vecteur si le nodebend est un bend pour les potentiels suivants et précédents
	std::vector<NodeBend*> bendPrecSuiv;
	for (int i = 0; i < vectorMoveCoord.size(); i++) {
		posVectorNodeBend[vectorMoveCoord[i].first][vectorMoveCoord[i].second].insert(n);
		*n->a_x = vectorMoveCoord[i].first;
		*n->a_y = vectorMoveCoord[i].second;
		// On cherche si il y a 1 ou 2 target apres le déplacement
		if (!n->isNode) {
			nodeAdjEntries.clear();
			nodeAdjEntries.pushBack(n->getAdjEntry());
			bendPrecSuiv.clear();
			bendPrecSuiv = getTargetNodeBends(n);
			if (bendPrecSuiv.size() > 1) {
				nodeAdjEntries.pushBack(n->getAdjEntry());
			}
		}
		//std::cout << "Check Deplacement " << i << std::endl;
		intersection = false;
		// On parcour la liste des adjentry du point de départ
		int j = 0;
		NodeBend* target = nullptr;
		for (auto it = nodeAdjEntries.begin(); ((it != nodeAdjEntries.end()) && (!intersection)); it++, j++) {
			// On recupere les coordonnées du target du segment
			if (n->isNode) {
				//getTargetCoord(GL, (*it), segmentSourceTrgX, segmentSourceTrgY);
				target = getTargetCoordAndNodeBend(GL, (*it), segmentSourceTrgX, segmentSourceTrgY);
				//std::cout << "Target1 Coord x: " << segmentSourceTrgX << " y: " << segmentSourceTrgY << std::endl;
				//std::cout << "Target2 Coord x: " << *target->a_x << " y: " << *target->a_y << std::endl;
			}
			else {
				target = bendPrecSuiv[j];
				segmentSourceTrgX = *target->a_x;
				segmentSourceTrgY = *target->a_y;
			}
			// Si le segment est non nul on fait les tests d'intersection, sinon on passe au prochain
			if ((vectorMoveCoord[i].first != segmentSourceTrgX) || (vectorMoveCoord[i].second != segmentSourceTrgY)) {
				// Et on regarde si une intersection se créer avec la liste des segments non adjacents
				for (int k = 0; (k < vectorAdjFaceSegments.size()) && (!intersection); k++) {
					// On vérifie que le segment n'est pas nul
					if (!vectorAdjFaceSegments[k].isNull()) {
						// On regarde si un NodeBend est commun aux deux segments ou s'il y a stacking
						NodeBend* nodeBendCommun = nullptr;
						std::pair<NodeBend*, NodeBend*> nodeBendNonCommun;
						bool notStackable = false;
						bool is_in = posVectorNodeBend[vectorMoveCoord[i].first][vectorMoveCoord[i].second].find(vectorAdjFaceSegments[k].source) != posVectorNodeBend[vectorMoveCoord[i].first][vectorMoveCoord[i].second].end();
						if (is_in) {
							notStackable = !(n->isStackableWith(vectorAdjFaceSegments[k].source));
							//std::cout << "Stack Cas 1: " << notStackable << std::endl;
						}
						bool is_in2 = posVectorNodeBend[vectorMoveCoord[i].first][vectorMoveCoord[i].second].find(vectorAdjFaceSegments[k].target) != posVectorNodeBend[vectorMoveCoord[i].first][vectorMoveCoord[i].second].end();
						if (is_in2) {
							notStackable = notStackable||!(n->isStackableWith(vectorAdjFaceSegments[k].target));
							//std::cout << "Stack Cas 2: " << notStackable << std::endl;
						}
						bool is_in3 = posVectorNodeBend[*target->a_x][*target->a_y].find(vectorAdjFaceSegments[k].source) != posVectorNodeBend[*target->a_x][*target->a_y].end();
						if (is_in3) {
							notStackable = notStackable||!(target->isStackableWith(vectorAdjFaceSegments[k].source));
							//std::cout << "Stack Cas 3: " << notStackable << std::endl;
						}
						bool is_in4 = posVectorNodeBend[*target->a_x][*target->a_y].find(vectorAdjFaceSegments[k].target) != posVectorNodeBend[*target->a_x][*target->a_y].end();
						if (is_in4) {
							notStackable = notStackable||!(target->isStackableWith(vectorAdjFaceSegments[k].target));
							//std::cout << "Stack Cas 4: " << notStackable << std::endl;
						}
						bool same = ((is_in && (is_in3 || is_in4)) || (is_in2 && (is_in3 || is_in4)));
						bool stack = (is_in || is_in2 || is_in3 || is_in4);
						//std::cout << "Deplacement: " << i << " stack: " << stack << " same: " << same << " dx " << *n->a_x - srcX << " dy " << *n->a_y - srcY << std::endl;
						//std::cout << "x1: " << vectorMoveCoord[i].first << " y1: " << vectorMoveCoord[i].second << " x2: " << segmentSourceTrgX << " y2: " << segmentSourceTrgY << " x3: " << *vectorAdjFaceSegments[k].sourceX << " y3: " << *vectorAdjFaceSegments[k].sourceY << " x4: " << *vectorAdjFaceSegments[k].targetX << " y4: " << *vectorAdjFaceSegments[k].targetY << std::endl;
						// Si on compare deux segments identique on passe
						if (same) {
							// On verifie si on est le meme segment ou superposé completement
							if ((vectorAdjFaceSegments[k].target->globalNum == n->globalNum) && (vectorAdjFaceSegments[k].source->globalNum == target->globalNum)) {

							}
							else if ((vectorAdjFaceSegments[k].target->globalNum == target->globalNum) && (vectorAdjFaceSegments[k].source->globalNum == n->globalNum)) {

							}
							else {
								// On verifie que le segment existe
								if (!n->isNode) {
									if ((n->suivant->globalNum == target->globalNum) || (n->precedent->globalNum == target->globalNum)) {
										intersection = true;
										//std::cout << "Cas superposition complete" << std::endl;
										//std::cout << "n " << n->globalNum << " t " << target->globalNum << " v1 " << vectorAdjFaceSegments[k].source->globalNum << " v2 " << vectorAdjFaceSegments[k].target->globalNum << std::endl;
									}
								}
								else {
									for (int i = 0; (i <n->adjNodeBend.size())&&!intersection; i++) {
										if (n->adjNodeBend[i]->globalNum == target->globalNum) {
											intersection = true;
											//std::cout << "Cas superposition complete" << std::endl;
											//std::cout << "n " << n->globalNum << " t " << target->globalNum << " v1 " << vectorAdjFaceSegments[k].source->globalNum << " v2 " << vectorAdjFaceSegments[k].target->globalNum << std::endl;
										}
									}
								}
							}
						}
						else {
							// Deplacement interdit
							if (notStackable) {
								intersection = true;
								//std::cout << "Cas stack interdit" << std::endl;
								//std::cout << "n " << n->globalNum << " t " << target->globalNum << " v1 " << vectorAdjFaceSegments[k].source->globalNum << " v2 " << vectorAdjFaceSegments[k].target->globalNum << std::endl;
							}
							else {
								// Si on a un stacking on regarde uniquement la superposition
								if (stack) {
									if (is_in) {
										nodeBendCommun = vectorAdjFaceSegments[k].source;
										nodeBendNonCommun.first = target;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k].target;
									}
									else if (is_in2) {
										nodeBendCommun = vectorAdjFaceSegments[k].target;
										nodeBendNonCommun.first = target;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k].source;
									}
									else if (is_in3) {
										nodeBendCommun = vectorAdjFaceSegments[k].source;
										nodeBendNonCommun.first = n;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k].target;
									}
									else if (is_in4) {
										nodeBendCommun = vectorAdjFaceSegments[k].target;
										nodeBendNonCommun.first = n;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k].source;
									}
									// On teste si les 3 noeuds sont alignés
									if (aGaucheInt(*nodeBendCommun->a_x, *nodeBendCommun->a_y, *nodeBendNonCommun.first->a_x, *nodeBendNonCommun.first->a_y, *nodeBendNonCommun.second->a_x, *nodeBendNonCommun.second->a_y) == 0) {
										// On teste si le noeud en commun ne se trouve pas entre les deux autres noeuds, dans ce cas intersection
										if (!dansRectangle(*nodeBendNonCommun.first->a_x, *nodeBendNonCommun.first->a_y, *nodeBendNonCommun.second->a_x, *nodeBendNonCommun.second->a_y, *nodeBendCommun->a_x, *nodeBendCommun->a_y)) {
											intersection = true;
											//std::cout << "Cas: 1 i:" << i << " k: " << k << " j: " << j << std::endl;
											//std::cout << "x1: " << *nodeBendNonCommun.first->a_x << " y1: " << *nodeBendNonCommun.first->a_y << " x2: " << *nodeBendNonCommun.second->a_x << " y2: " << *nodeBendNonCommun.second->a_y << " x3: " << *nodeBendCommun->a_x << " y3: " << *nodeBendCommun->a_y << std::endl;
										}
									}
								}
								// Si aucun stacking on fait le test d'intersection
								else {
									// On regarde s'ils se croisent
									if (seCroisent(vectorMoveCoord[i].first, vectorMoveCoord[i].second, segmentSourceTrgX, segmentSourceTrgY, *vectorAdjFaceSegments[k].sourceX, *vectorAdjFaceSegments[k].sourceY, *vectorAdjFaceSegments[k].targetX, *vectorAdjFaceSegments[k].targetY)) {
										intersection = true;
										//std::cout << "Cas: 2 i:" << i << " k: " << k << " j: " << j << std::endl;
										//std::cout << "x1: " << vectorMoveCoord[i].first << " y1: " << vectorMoveCoord[i].second << " x2: " << segmentSourceTrgX << " y2: " << segmentSourceTrgY << " x3: " << *vectorAdjFaceSegments[k].sourceX << " y3: " << *vectorAdjFaceSegments[k].sourceY << " x4: " << *vectorAdjFaceSegments[k].targetX << " y4: " << *vectorAdjFaceSegments[k].targetY << std::endl;
									}
								}
							}
						}
					}
				}
			}
			// Sinon on vérifie que le déplacement n'est pas un node sur un autre node
			else {
				if (n->isNode) {
					if ((vectorMoveCoord[i].first == GL.x((*it)->twinNode())) && (vectorMoveCoord[i].second == GL.y((*it)->twinNode()))) {
						intersection = true;
						//std::cout << "Cas: 3 i:" << i << std::endl;
					}
				}
			}
		}
		// On regarde si la face s'inverse ou que l'ordre autour d'un noeud change
		if (!intersection) {
			// On regarde si les ordre du noeud source ou des adjacents ont changé
			if (orderNodeAdjChanged(n, GL, vectorMoveCoord[i].first, vectorMoveCoord[i].second)) {
				intersection = true;
				//std::cout << "Cas: 4 i:" << i << std::endl;
			}
			// Sinon on regarde le cas particulier si le point qui se déplace inverse une face sans changer l'ordre
			// Pour cela il doit avoir 2 voisins et ses deux voisins aussi
			else if (!n->isNode) {
				if (ccem.leftFace(n->getAdjEntry()) != ccem.rightFace(n->getAdjEntry())) {
					// A FAIRE
				}
			}
			else if (n->getNode()->degree() == 2) {
				if (ccem.leftFace(n->getNode()->firstAdj()) != ccem.rightFace(n->getNode()->firstAdj())) {
					// A FAIRE
				}
			}
		}
		// Intersection = déplacement pas autorisé
		vectorMoveAutorised.push_back(!intersection);
		removeNodeBendPos(n, vectorMoveCoord[i].first, vectorMoveCoord[i].second);
	}
	posVectorNodeBend[srcX][srcY].insert(n);
	*n->a_x = srcX;
	*n->a_y = srcY;
	return vectorMoveAutorised;
}

// Renvoie un vecteur qui attribue une probabilité a un déplacement
// Pour les déplacements: 0=droite(x+1) 1=haut(y+1) 2=gauche(x-1) 3=bas(y-1)
// Cette fonction doit etre appelée avant un déplacement
// Les poids assignés aux déplacements sont attribués en fonction de leur amélioration de l'écart-type
std::vector<std::pair<int, std::pair<int, int>>> rouletteRusseNodeMove(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	//std::cout << "Variance avant roulette: " << variance << std::endl;
	int nx = (*n->a_x);
	int ny = (*n->a_y);
	// On stocke les changements de variances apres un déplacement
	std::vector<double> vectorVarChangeMove;
	// On stocke les coordonnées d'arrivée qu'on aurait apres le déplacement
	std::vector<std::pair<int, int>> vectorMoveCoord;
	if ((nx + 1) <= gridWidth)
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny));
	if ((ny + 1) <= gridHeight)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny + 1));
	if ((nx - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny));
	if ((ny - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny - 1));
	if (((nx + 1) <= gridWidth) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny + 1));
	if (((nx + 1) <= gridWidth) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny - 1));
	if (((nx - 1) >= 0) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny + 1));
	if (((nx - 1) >= 0) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny - 1));
	// On stocke si les déplacements sont autorisés, donc s'il n'y a pas de node ou de bend a ces coordonnées
	std::vector<bool> vectorMoveAutorised = getLegalMoves(n, GL, vectorMoveCoord, ccem);
	SListPure<adjEntry> adjEntries;
	bool isNode = true;
	if (n->isNode) {
		n->getNode()->allAdjEntries(adjEntries);
	}
	else {
		adjEntries.pushBack(n->getAdjEntry());
		isNode = false;
	}
	// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
	double tmpMaxContribution = 0;
	int numberMoveAutorised = 1;
	// Boucle sur tout les déplacements possibles
	for (int i = 0; i < vectorMoveAutorised.size(); i++) {
		// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
		if (vectorMoveAutorised[i]) {
			numberMoveAutorised++;
			double tmpSommeLong = sommeLong;
			double tmpSommeLong2 = sommeLong2;
			double tmpVariance = variance;
			for (auto it = adjEntries.begin(); it.valid(); it++) {
				auto it2 = mapEdgeLength.find((*it)->theEdge());
				double tmpOldLength = it2->second;
				double tmpNewLength;
				if (isNode) {
					tmpNewLength = calcTmpEdgeLength((*it), vectorMoveCoord[i].first, vectorMoveCoord[i].second, GL);
				}
				else {
					tmpNewLength = calcTmpEdgeLengthBends((*it)->theEdge(), n, vectorMoveCoord[i].first, vectorMoveCoord[i].second, GL);
				}
				deleteEdgeNVar(tmpOldLength, tmpSommeLong, tmpSommeLong2);
				addEdgeNVar(tmpNewLength, tmpSommeLong, tmpSommeLong2);
			}
			tmpVariance = calcNVar(tmpSommeLong, tmpSommeLong2);
			double tmpContribution = tmpVariance - variance;
			vectorVarChangeMove.push_back(tmpContribution);
			if (tmpContribution > tmpMaxContribution) {
				tmpMaxContribution = tmpContribution;
			}
		}
		else {
			vectorVarChangeMove.push_back(-1);
		}
	}
	//pas de mouvement -> déplacement 4
	vectorMoveCoord.push_back(std::pair<int, int>(nx, ny));
	vectorMoveAutorised.push_back(true);
	vectorVarChangeMove.push_back(0);
	std::vector<std::pair<int, std::pair<int, int>>> vectorProbaMove;
	double tmpVarSomme = 0;
	// On soustrait a tout les valeurs la variance maximale
	for (int i = 0; i < vectorVarChangeMove.size(); i++) {
		if (vectorMoveAutorised[i]) {
			//std::cout << "Contribution Variance deplacement " << i << ": " << vectorVarChangeMove[i] << std::endl;
			vectorVarChangeMove[i] = (-vectorVarChangeMove[i]) + (2 * tmpMaxContribution);
			tmpVarSomme += vectorVarChangeMove[i];
		}
	}
	// On transforme les valeurs en proba
	int tmpSommeProba = 0;
	int size = vectorMoveAutorised.size() - 1;
	for (int i = 0; i < size; i++) {
		if (vectorMoveAutorised[i]) {
			if (tmpVarSomme == 0) {
				int tmpProba = round(100 / numberMoveAutorised);
				tmpSommeProba += tmpProba;
				std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
			}
			else {
				int tmpProba = round((vectorVarChangeMove[i] / tmpVarSomme) * 100);
				tmpSommeProba += tmpProba;
				std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
			}
			if (tmpSommeProba > 100)
				tmpSommeProba = 100;
			std::pair<int, std::pair<int, int>> tmpPair(tmpSommeProba, vectorMoveCoord[i]);
			vectorProbaMove.push_back(tmpPair);
		}
	}
	std::cout << "Deplacement " << size << " Proba: " << 100 - tmpSommeProba << " SommeProba: " << 100 << std::endl;
	std::pair<int, std::pair<int, int>> tmpPair(100, vectorMoveCoord[size]);
	vectorProbaMove.push_back(tmpPair);
	return vectorProbaMove;
}

// Change les valeurs dans mapEdgeLength et mapLengthEdgeSet
void changeEdgeLengthInMaps(edge e, double oldEdgeLength, double newEdgeLength) {
	if (oldEdgeLength != newEdgeLength) {
		auto it = mapEdgeLength.find(e);
		it->second = newEdgeLength;
		auto it2 = mapLengthEdgeSet.find(newEdgeLength);
		if (it2 != mapLengthEdgeSet.end()) {
			it2->second.insert(e);
		}
		else {
			std::set<edge> tmpSet;
			tmpSet.insert(e);
			mapLengthEdgeSet.insert(std::pair<double, std::set<edge>>(newEdgeLength, tmpSet));
		}
		auto it3 = mapLengthEdgeSet.find(oldEdgeLength);
		it3->second.erase(e);
		if (it3->second.empty()) {
			mapLengthEdgeSet.erase(oldEdgeLength);
		}
	}
}

// A appelé avant le déplacement avec les nouvelles coordonnées du nodebend
// Change la variance du graphe en fonction du déplacement choisi
// Modifie les maps utilisant les longueurs des egdes
void changeVariance(NodeBend* n, GridLayout& GL, int newSrcX, int newSrcY, double& sommeLong, double& sommeLong2, double& variance) {
	if (n->isNode) {
		List<adjEntry> nodeAdjEntries;
		n->getNode()->allAdjEntries(nodeAdjEntries);
		double oldEdgeLength, newEdgeLength;
		for (auto it = nodeAdjEntries.begin(); it != nodeAdjEntries.end(); it++) {
			edge e = (*it)->theEdge();
			auto it2 = mapEdgeLength.find(e);
			double oldEdgeLength = it2->second;
			double newEdgeLength = calcTmpEdgeLength((*it), newSrcX, newSrcY, GL);
			deleteEdgeNVar(oldEdgeLength, sommeLong, sommeLong2);
			addEdgeNVar(newEdgeLength, sommeLong, sommeLong2);
			changeEdgeLengthInMaps(e, oldEdgeLength, newEdgeLength);
		}
	}
	else {
		edge e = n->getEdge();
		auto it = mapEdgeLength.find(e);
		double oldEdgeLength = it->second;
		double newEdgeLength = calcTmpEdgeLengthBends(e, n, newSrcX, newSrcY, GL);
		deleteEdgeNVar(oldEdgeLength, sommeLong, sommeLong2);
		addEdgeNVar(newEdgeLength, sommeLong, sommeLong2);
		changeEdgeLengthInMaps(e, oldEdgeLength, newEdgeLength);
	}
	variance = calcNVar(sommeLong, sommeLong2);
}

// Deplace un NodeBend aux coordonnées newX et newY et fait les vérifications sur le stacking.
void moveNodeBend(NodeBend* nb, int newX, int newY) {
	removeNodeBendPos(nb, *nb->a_x, *nb->a_y);
	(*nb->a_x) = newX;
	(*nb->a_y) = newY;
	if (nb->isNode) {
		nb->recalculateAdjBendStack();
	}
	else {
		nb->recalculateIsStacked();
		nb->precedent->recalculateIsStacked();
		nb->suivant->recalculateIsStacked();
	}
	posVectorNodeBend[*nb->a_x][*nb->a_y].insert(nb);
}

// Fonction de déplacement utilisée avec les raccourcis openGL, pas utilisée pour les tests finaux
void move(NodeBend* n, GridLayout& GL, int dx, int dy, double& sommeLong, double& sommeLong2, double& variance) {
	removeNodeBendPos(n,*n->a_x,*n->a_y);
	int newX = (*n->a_x) + dx;
	int newY = (*n->a_y) + dy;
	changeVariance(n, GL, newX, newY, sommeLong, sommeLong2, variance);
	(*n->a_x) = newX;
	(*n->a_y) = newY;
	if (n->isNode) {
		n->recalculateAdjBendStack();
	}
	else {
		n->recalculateIsStacked();
		n->precedent->recalculateIsStacked();
		n->suivant->recalculateIsStacked();
	}
	posVectorNodeBend[*n->a_x][*n->a_y].insert(n);
}

// Demarre l'algorithme de roulette russe sur le NodeBend choisi a l'avance
void specificRouletteRusse(int numero, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	//std::cout << "Numero selectionne: " << numero << std::endl;
	NodeBend* nb = vectorNodeBends[numero];
	std::vector<std::pair<int, std::pair<int, int>>> probaDeplacement = rouletteRusseNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
	if (probaDeplacement.size() > 0) {
		int randomChoice = generateRand(100);
		bool moved = false;
		for (int i = 0; ((i < probaDeplacement.size()) && (!moved)); i++) {
			if (randomChoice <= probaDeplacement[i].first) {
				//std::cout << "Nombre aleatoire: " << randomChoice << " Deplacement choisi : " << i << std::endl;
				changeVariance(nb, GL, probaDeplacement[i].second.first, probaDeplacement[i].second.second, sommeLong, sommeLong2, variance);
				moveNodeBend(nb, probaDeplacement[i].second.first, probaDeplacement[i].second.second);
				moved = true;
			}
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
}

// Demarre l'algorithme de roulette russe sur le graphe
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
int startRouletteRusse(GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	// On choisis au hasard un NodeBend
	int randomNum = generateRand(vectorNodeBends.size()) - 1;
	//std::cout << "Numero selectionne: " << randomNum << std::endl;
	NodeBend* nb = vectorNodeBends[randomNum];
	std::vector<std::pair<int, std::pair<int, int>>> probaDeplacement = rouletteRusseNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
	if (probaDeplacement.size() > 0) {
		int randomChoice = generateRand(100);
		bool moved = false;
		for (int i = 0; ((i < probaDeplacement.size()) && (!moved)); i++) {
			if (randomChoice <= probaDeplacement[i].first) {
				//std::cout << "Nombre aleatoire: " << randomChoice << " Deplacement choisi : " << i << std::endl;
				changeVariance(nb, GL, probaDeplacement[i].second.first, probaDeplacement[i].second.second, sommeLong, sommeLong2, variance);
				moveNodeBend(nb, probaDeplacement[i].second.first, probaDeplacement[i].second.second);
				moved = true;
			}
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return randomNum;
}

// Renvoie un vecteur qui attribue une probabilité a un déplacement
// Cette fonction doit etre appelée avant un déplacement
// Les poids assignés aux déplacements sont attribués en fonction de leur amélioration de l'écart-type et du coefficient de recuit simulé
std::vector<std::pair<int, std::pair<int, int>>> recuitSimuleNodeMove(NodeBend* n, double coeff, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	int nx = (*n->a_x);
	int ny = (*n->a_y);
	// On stocke les changements de variances apres un déplacement
	std::vector<double> vectorVarChangeMove;
	// On stocke les coordonnées d'arrivée qu'on aurait apres le déplacement
	std::vector<std::pair<int, int>> vectorMoveCoord;
	if ((nx + 1) <= gridWidth)
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny));
	if ((ny + 1) <= gridHeight)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny + 1));
	if ((nx - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny));
	if ((ny - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny - 1));
	if (((nx + 1) <= gridWidth) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny + 1));
	if (((nx + 1) <= gridWidth) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny - 1));
	if (((nx - 1) >= 0) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny + 1));
	if (((nx - 1) >= 0) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny - 1));
	// On stocke si les déplacements sont autorisés, donc s'il n'y a pas de node ou de bend a ces coordonnées
	std::vector<bool> vectorMoveAutorised = getLegalMoves(n, GL, vectorMoveCoord, ccem);
	SListPure<adjEntry> adjEntries;
	bool isNode = true;
	if (n->isNode) {
		n->getNode()->allAdjEntries(adjEntries);
	}
	else {
		adjEntries.pushBack(n->getAdjEntry());
		isNode = false;
	}
	// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
	double tmpMaxContribution = 0;
	int numberMoveAutorised = 1;
	// Boucle sur tout les déplacements possibles
	for (int i = 0; i < vectorMoveAutorised.size(); i++) {
		// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
		if (vectorMoveAutorised[i]) {
			numberMoveAutorised++;
			double tmpSommeLong = sommeLong;
			double tmpSommeLong2 = sommeLong2;
			double tmpVariance = variance;
			for (auto it = adjEntries.begin(); it.valid(); it++) {
				auto it2 = mapEdgeLength.find((*it)->theEdge());
				double tmpOldLength = it2->second;
				double tmpNewLength;
				if (isNode) {
					tmpNewLength = calcTmpEdgeLength((*it), vectorMoveCoord[i].first, vectorMoveCoord[i].second, GL);
				}
				else {
					tmpNewLength = calcTmpEdgeLengthBends((*it)->theEdge(), n, vectorMoveCoord[i].first, vectorMoveCoord[i].second, GL);
				}
				deleteEdgeNVar(tmpOldLength, tmpSommeLong, tmpSommeLong2);
				addEdgeNVar(tmpNewLength, tmpSommeLong, tmpSommeLong2);
			}
			tmpVariance = calcNVar(tmpSommeLong, tmpSommeLong2);
			double tmpContribution = tmpVariance - variance;
			vectorVarChangeMove.push_back(tmpContribution);
			if (tmpContribution > tmpMaxContribution) {
				tmpMaxContribution = tmpContribution;
			}

		}
		else {
			vectorVarChangeMove.push_back(-1);
		}
	}
	//pas de mouvement -> déplacement 4
	vectorMoveCoord.push_back(std::pair<int, int>(nx, ny));
	vectorMoveAutorised.push_back(true);
	vectorVarChangeMove.push_back(0);
	std::vector<std::pair<int, std::pair<int, int>>> vectorProbaMove;
	double tmpVarSomme = 0;
	// On soustrait a tout les valeurs la variance maximale
	for (int i = 0; i < vectorVarChangeMove.size(); i++) {
		if (vectorMoveAutorised[i]) {
			//std::cout << "Contribution Variance deplacement " << i << ": " << vectorVarChangeMove[i] << std::endl;
			vectorVarChangeMove[i] = (-vectorVarChangeMove[i]) + (2 * tmpMaxContribution);
			tmpVarSomme += vectorVarChangeMove[i];
		}
	}
	// Si la somme des variance est égale a 0, alors tout les déplacements ont la meme proba, pas besoin d'appliquer le coeff
	int size = vectorMoveAutorised.size() - 1;
	int tmpSommeProba = 0;
	if (tmpVarSomme == 0) {
		for (int i = 0; i < size; i++) {
			if (vectorMoveAutorised[i]) {
				int tmpProba = round(100 / numberMoveAutorised);
				tmpSommeProba += tmpProba;
				//std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
				if (tmpSommeProba > 100)
					tmpSommeProba = 100;
				std::pair<int, std::pair<int, int>> tmpPair(tmpSommeProba, vectorMoveCoord[i]);
				vectorProbaMove.push_back(tmpPair);
			}
		}
	}
	// Il y a au moin un déplacement qui change la variance
	else {
		// On applique le coefficient aux probabilités
		double tmpSommeProbaFraction = 0;
		std::vector<double> vectorTmpProba;
		for (int i = 0; i < size; i++) {
			if (vectorMoveAutorised[i]) {
				double tmpProbaFraction = vectorVarChangeMove[i] / tmpVarSomme;
				tmpProbaFraction = pow(tmpProbaFraction, coeff);
				tmpSommeProbaFraction += tmpProbaFraction;
				vectorTmpProba.push_back(tmpProbaFraction);
			}
			else {
				vectorTmpProba.push_back(-1.0);
			}
		}
		// On transforme ces valeurs en probabilité
		for (int i = 0; i < size; i++) {
			if (vectorMoveAutorised[i]) {
				int tmpProba = round((vectorTmpProba[i] / tmpSommeProbaFraction) * 100);
				tmpSommeProba += tmpProba;
				//std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
				if (tmpSommeProba > 100)
					tmpSommeProba = 100;
				std::pair<int, std::pair<int, int>> tmpPair(tmpSommeProba, vectorMoveCoord[i]);
				vectorProbaMove.push_back(tmpPair);
			}
		}
	}

	//std::cout << "Deplacement " << size << " Proba: " << 100 - tmpSommeProba << " SommeProba: " << 100 << std::endl;
	std::pair<int, std::pair<int, int>> tmpPair(100, vectorMoveCoord[size]);
	vectorProbaMove.push_back(tmpPair);
	return vectorProbaMove;
}

// Demarre l'algorithme de recuit simulé sur le graphe
// Les probas sont calculés avec l'algorithme roulette russe et modifiée avec un coefficient évoluant avec le temps
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
int startRecuitSimule(double coeff, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	// On choisis au hasard un NodeBend
	int randomNum = generateRand(vectorNodeBends.size()) - 1;
	//std::cout << "Numero selectionne: " << randomNum << std::endl;
	NodeBend* nb = vectorNodeBends[randomNum];
	std::vector<std::pair<int, std::pair<int, int>>> probaDeplacement = recuitSimuleNodeMove(nb, coeff, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
	std::vector<double> tmpProba;
	double tmpSommeProba = 0;
	if (probaDeplacement.size() > 0) {
		int randomChoice = generateRand(100);
		bool moved = false;
		// Le déplacement est choisi aléatoirement
		for (int i = 0; ((i < probaDeplacement.size()) && (!moved)); i++) {
			if (randomChoice <= probaDeplacement[i].first) {
				//std::cout << "Nombre aleatoire: " << randomChoice << " Deplacement choisi : " << i << std::endl;
				changeVariance(nb, GL, probaDeplacement[i].second.first, probaDeplacement[i].second.second, sommeLong, sommeLong2, variance);
				moveNodeBend(nb, probaDeplacement[i].second.first, probaDeplacement[i].second.second);
				moved = true;
			}
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return randomNum;
}

// Renvoie un vecteur composé des meilleurs déplacements améliorant la variance
// Cette fonction doit etre appelée avant un déplacement
std::vector<std::pair<int, int>> bestVarianceNodeMove(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	int nx = (*n->a_x);
	int ny = (*n->a_y);
	// On stocke les changements de variances apres un déplacement
	std::vector<double> vectorVarChangeMove;
	// On stocke les coordonnées d'arrivée qu'on aurait apres le déplacement
	std::vector<std::pair<int, int>> vectorMoveCoord;
	if ((nx + 1) <= gridWidth)
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny));
	if ((ny + 1) <= gridHeight)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny + 1));
	if ((nx - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny));
	if ((ny - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny - 1));
	if (((nx + 1) <= gridWidth) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny + 1));
	if (((nx + 1) <= gridWidth) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny - 1));
	if (((nx - 1) >= 0) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny + 1));
	if (((nx - 1) >= 0) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny - 1));
	// On stocke si les déplacements sont autorisés, donc s'il n'y a pas de node ou de bend a ces coordonnées
	std::vector<bool> vectorMoveAutorised = getLegalMoves(n, GL, vectorMoveCoord, ccem);
	SListPure<adjEntry> adjEntries;
	bool isNode = true;
	if (n->isNode) {
		n->getNode()->allAdjEntries(adjEntries);
	}
	else {
		adjEntries.pushBack(n->getAdjEntry());
		isNode = false;
	}
	// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
	double tmpMinContribution = 0;
	// Boucle sur tout les déplacements possibles
	for (int i = 0; i < vectorMoveAutorised.size(); i++) {
		// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
		if (vectorMoveAutorised[i]) {
			double tmpSommeLong = sommeLong;
			double tmpSommeLong2 = sommeLong2;
			double tmpVariance = variance;
			for (auto it = adjEntries.begin(); it.valid(); it++) {
				auto it2 = mapEdgeLength.find((*it)->theEdge());
				double tmpOldLength = it2->second;
				double tmpNewLength;
				if (isNode) {
					tmpNewLength = calcTmpEdgeLength((*it), vectorMoveCoord[i].first, vectorMoveCoord[i].second, GL);
				}
				else {
					tmpNewLength = calcTmpEdgeLengthBends((*it)->theEdge(), n, vectorMoveCoord[i].first, vectorMoveCoord[i].second, GL);
				}
				deleteEdgeNVar(tmpOldLength, tmpSommeLong, tmpSommeLong2);
				addEdgeNVar(tmpNewLength, tmpSommeLong, tmpSommeLong2);
			}
			tmpVariance = calcNVar(tmpSommeLong, tmpSommeLong2);
			double tmpContribution = tmpVariance - variance;
			vectorVarChangeMove.push_back(tmpContribution);
			if (tmpContribution < tmpMinContribution) {
				tmpMinContribution = tmpContribution;
			}

		}
		else {
			vectorVarChangeMove.push_back(-1);
		}
	}
	std::vector<std::pair<int, int>> vectorMove;
	// S'il y a au moin un déplacement améliorant la variance
	if (tmpMinContribution < -0.00001) {
		// On stocke ces déplacements
		for (int i = 0; i < vectorVarChangeMove.size(); i++) {
			if (vectorMoveAutorised[i]) {
				if ((vectorVarChangeMove[i] < tmpMinContribution + 0.000001) && (vectorVarChangeMove[i] > tmpMinContribution - 0.000001)) {
					vectorMove.push_back(vectorMoveCoord[i]);
				}
			}
		}
	}
	return vectorMove;
}

// Demarre l'algorithme de best variance
// On choisis uniquement les déplacements qui améliorent le plus la variance, s'ils sont egaux on tire au hasard.
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
// On itere sur les noeuds dans l'ordre
int startBestVariance(GridLayout& GL, ConstCombinatorialEmbedding& ccem, int numCourant, int& numLastMoved, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	NodeBend* nb = vectorNodeBends[numCourant];
	std::vector<std::pair<int, int>> deplacements = bestVarianceNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
	std::vector<double> tmpProba;
	double tmpSommeProba = 0;
	int size = deplacements.size();
	if (size > 0) {
		numLastMoved = numCourant;
		int randomChoice = generateRand(size) - 1;
		changeVariance(nb, GL, deplacements[randomChoice].first, deplacements[randomChoice].second, sommeLong, sommeLong2, variance);
		moveNodeBend(nb, deplacements[randomChoice].first, deplacements[randomChoice].second);
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return numCourant;
}

// Renvoie un vecteur composé des meilleurs déplacements réduisant les longueurs autour d'un noeud
// Cette fonction doit etre appelée avant un déplacement
std::vector<std::pair<int, int>> shortestLengthNodeMove(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	int nx = (*n->a_x);
	int ny = (*n->a_y);
	// On stocke les coordonnées d'arrivée qu'on aurait apres le déplacement
	std::vector<std::pair<int, int>> vectorMoveCoord;
	if ((nx + 1) <= gridWidth)
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny));
	if ((ny + 1) <= gridHeight)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny + 1));
	if ((nx - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny));
	if ((ny - 1) >= 0)
		vectorMoveCoord.push_back(std::pair<int, int>(nx, ny - 1));
	if (((nx + 1) <= gridWidth) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny + 1));
	if (((nx + 1) <= gridWidth) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx + 1, ny - 1));
	if (((nx - 1) >= 0) && ((ny + 1) <= gridHeight))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny + 1));
	if (((nx - 1) >= 0) && ((ny - 1) >= 0))
		vectorMoveCoord.push_back(std::pair<int, int>(nx - 1, ny - 1));
	// On stocke si les déplacements sont autorisés, donc s'il n'y a pas de node ou de bend a ces coordonnées
	std::vector<bool> vectorMoveAutorised = getLegalMoves(n, GL, vectorMoveCoord, ccem);
	SListPure<adjEntry> adjEntries;
	bool isNode = true;
	if (n->isNode) {
		n->getNode()->allAdjEntries(adjEntries);
	}
	else {
		adjEntries.pushBack(n->getAdjEntry());
		isNode = false;
	}
	// On stocke les changements de longueurs apres un déplacement
	std::vector<double> vectorLengthChangeMove;
	// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
	double longueurActuelle = totalLengthAroundNodeBend(n, GL, nx, ny);
	double minLongeurs = longueurActuelle;
	int numberMoveAutorised = 1;
	// Boucle sur tout les déplacements possibles
	for (int i = 0; i < vectorMoveAutorised.size(); i++) {
		// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
		if (vectorMoveAutorised[i]) {
			numberMoveAutorised++;
			double tmpLongueur = totalLengthAroundNodeBend(n, GL, vectorMoveCoord[i].first, vectorMoveCoord[i].second);
			vectorLengthChangeMove.push_back(tmpLongueur);
			if (tmpLongueur < minLongeurs) {
				minLongeurs = tmpLongueur;
			}
		}
		else {
			vectorLengthChangeMove.push_back(-1);
		}
	}
	std::vector<std::pair<int, int>> vectorMove;
	// S'il y a au moin un déplacement améliorant la variance
	if (minLongeurs < longueurActuelle) {
		// On stocke ces déplacements
		for (int i = 0; i < vectorLengthChangeMove.size(); i++) {
			if (vectorMoveAutorised[i]) {
				if ((vectorLengthChangeMove[i] < minLongeurs + 0.000001) && (vectorLengthChangeMove[i] > minLongeurs - 0.000001)) {
					vectorMove.push_back(vectorMoveCoord[i]);
				}
			}
		}
	}
	return vectorMove;
}

// Demarre l'algorithme de longueur minimale
// On choisis uniquement les déplacements qui réduisent les longueurs autour d'un node, s'ils sont egaux on tire au hasard.
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
// On itere sur les noeuds dans l'ordre
int startShortestLength(GridLayout& GL, ConstCombinatorialEmbedding& ccem, int numCourant, int& numLastMoved, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	NodeBend* nb = vectorNodeBends[numCourant];
	std::vector<std::pair<int, int>> deplacements = shortestLengthNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth);
	std::vector<double> tmpProba;
	double tmpSommeProba = 0;
	int size = deplacements.size();
	if (size > 0) {
		numLastMoved = numCourant;
		int randomChoice = generateRand(size) - 1;
		changeVariance(nb, GL, deplacements[randomChoice].first, deplacements[randomChoice].second, sommeLong, sommeLong2, variance);
		moveNodeBend(nb, deplacements[randomChoice].first, deplacements[randomChoice].second);
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return numCourant;
}

// Calcul le ratio edge/length. longueur la plus grande divisé par la longueur la plus courte.
double calcEdgeLengthRatio() {
	double ratio = (mapLengthEdgeSet.rbegin()->first / mapLengthEdgeSet.begin()->first);
	return ratio;
}

// Renvoie l'ordre trigonométrique de tout les adjEntry autour d'un noeud
ListPure<adjEntry> embedNode(Graph& G, GridLayout& GL, node nsrc) {
	ListPure<adjEntry> adj;
	nsrc->allAdjEntries(adj);
	return orderAroundNode(nsrc, GL, adj);
}

// Creer l'embedding du graphe avec le concept de carte que l'on a vu au premier semestre.
void embedderCarte(Graph& G, GridLayout& GL) {
	ListPure<adjEntry> newOrder;
	node nsrc = G.firstNode();
	while (nsrc != nullptr) {
		newOrder = embedNode(G, GL, nsrc);
		G.sort(nsrc, newOrder);
		nsrc = nsrc->succ();
	}
}

#endif