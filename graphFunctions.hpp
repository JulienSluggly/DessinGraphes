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

std::vector<std::pair<int, int>> vecteurDeplacements; // Vecteur des déplacements, tout algos
std::vector<bool> vecteurLegalDeplacements; // A un emplacement de plus que vecteurDeplacements, pour tout les algos
std::vector<double> vecteurVarChangeMove; // A autant de size que vecteurLegalDeplacements, utilisé par recuit simulé,roulette russe,bestvariance,shortestLength
std::vector<double> vecteurProbaMove; // A autant de size que vecteurVarChangeMove, utilisé par recuit simulé et roulette russe
std::vector<double> vecteurProbaCoeff; // A autant de size que vecteurVarChangeMove, utilisé par le recuit simulé
std::vector<int> vectorBestVarianceMove; // A une taille dynamique et différente a chaque tour, utilisé par best variance et shortest length
bool atLeastOneMove = false; // Booleen indiquant si vecteurLegalDeplacements a au moin une valeur a vraie
bool moveBendsWithNode = true; // Booleen qui indique si un node deplace tout ses bends stackés avec lui

// Renvoie le NodeBend passé en argument s'il peut se déplacer, sinon renvoie un de ses suivant ou précédent qui peut se déplacer
NodeBend* movableNodeBend(NodeBend* nb) {
	NodeBend* tmpNb = nb;
	// Si on est un node on est jamais stuck si on deplace les bends avec nous
	if (tmpNb->isNode) {
		if (!moveBendsWithNode) {
			// Si on ne deplace pas les bends avec nous, on prend un bend de sa premiere adjentry
			if (tmpNb->nbDiffAdjStacked > 1) {
				return movableNodeBend(tmpNb->adjNodeBend[0]);
			}
		}
	}
	else {
		// Tant qu'on est stuck on prend le precedent jusqu'a ne plus etre stuck ou atteindre un noeud
		while ((!tmpNb->isNode) && (tmpNb->isStuck())) {
			tmpNb = tmpNb->precedent;
		}
		// Si on a atteind un noeud on cherche un bend de l'autre coté
		if (tmpNb->isNode) {
			tmpNb = nb->suivant;
			while (tmpNb->isStuck()) {
				tmpNb = tmpNb->suivant;
			}
		}
	}
	return tmpNb;
}

// Initialise le vecteur de deplacement avec les differents deplacements voulu
// Les paires correspondent aux deplacements en X et Y du NodeBend choisi
// Initialise le vecteur de booleen indiquant si le deplacement au meme index dans vecteurDeplacements est legal ou non
void initListeDeplacements() {
	vecteurDeplacements.push_back(std::pair<int, int>(1, 0)); // Deplacement 0: DROITE
	vecteurDeplacements.push_back(std::pair<int, int>(0, 1)); // Deplacement 1: HAUT
	vecteurDeplacements.push_back(std::pair<int, int>(-1, 0)); // Deplacement 2: GAUCHE
	vecteurDeplacements.push_back(std::pair<int, int>(0, -1)); // Deplacement 3: BAS
	vecteurDeplacements.push_back(std::pair<int, int>(1, 1)); // Deplacement 4: HAUT-DROITE
	vecteurDeplacements.push_back(std::pair<int, int>(-1, 1)); // Deplacement 5: HAUT-GAUCHE
	vecteurDeplacements.push_back(std::pair<int, int>(-1, -1)); // Deplacement 6: BAS-GAUCHE
	vecteurDeplacements.push_back(std::pair<int, int>(1, -1)); // Deplacement 7: BAS-DROITE
	// Un de plus pour le deplacement pas bouger
	for (int i = 0; i <= vecteurDeplacements.size(); i++) {
		vecteurLegalDeplacements.push_back(true);
		vecteurVarChangeMove.push_back(0.0);
		vecteurProbaMove.push_back(100.0);
		vecteurProbaCoeff.push_back(0.0);
	}
}

// Fonction qui modifie les booleen de vecteurLegalDeplacements en fonction de si le deplacement fait sortir le NodeBend de la grille
void checkDebordement(NodeBend* n, int gridWidth, int gridHeight) {
	int srcX = n->getX();
	int srcY = n->getY();
	for (int i = 0; i < vecteurDeplacements.size(); i++) {
		vecteurLegalDeplacements[i] = ((srcX + vecteurDeplacements[i].first >= 0) && (srcX + vecteurDeplacements[i].first <= gridWidth) && (srcY + vecteurDeplacements[i].second >= 0) && (srcY + vecteurDeplacements[i].second <= gridHeight));
		//std::cout << "sx: " << srcX << " sy: " << srcY << " nx: " << srcX + vecteurDeplacements[i].first << " ny: " << srcY + vecteurDeplacements[i].second << " autorised: " << vecteurLegalDeplacements[i] << std::endl;
	}
}

// Fonction qui modifie les booleen de vecteurLegalDeplacements en fonction de si le deplacement provoque un empilement non autorisé du NodeBend sur un autre NodeBend
// Cette fonction utilise la grille de NodeBend et ne doit etre utilisée uniquement si le graphe est en place et le stacking autorisé
// Appeler cette fonction APRES avoir appeller checkDebordement();
void checkStacking(NodeBend* n) {
	for (int i = 0; i < vecteurDeplacements.size(); i++) {
		bool autorisedStacking = false;
		if (vecteurLegalDeplacements[i]) {
			int newX = n->getX() + vecteurDeplacements[i].first;
			int newY = n->getY() + vecteurDeplacements[i].second;
			std::list<NodeBend*>& setNodeBend = posVectorNodeBend[newX][newY];
			if (setNodeBend.size() > 0) {
				if (n->isNode) {
					auto it = setNodeBend.begin();
					if (!(*it)->isNode) {
						for (; ((it != setNodeBend.end()) && (!autorisedStacking)); it++) {
							for (int j = 0; ((j < n->adjNodeBend.size()) && (!autorisedStacking)); j++) {
								// Soit si un des nodebends de la liste est un adjacent du noeud
								if ((*it)->globalNum == n->adjNodeBend[j]->globalNum) {
									autorisedStacking = true;
								}
								// Soit si un précédent ou un suivant d'un de la liste est déja stacké avec le noeud
								else if ((((*it)->suivant->getX() == n->getX()) && ((*it)->suivant->getY() == n->getY()))|| (((*it)->precedent->getX() == n->getX()) && ((*it)->precedent->getY() == n->getY()))) {
									autorisedStacking = true;
								}
							}
						}
					}
				}
				else {
					for (auto it = setNodeBend.begin(); ((it != setNodeBend.end()) && (!autorisedStacking)); it++) {
						if (((*it)->globalNum == n->precedent->globalNum) || ((*it)->globalNum == n->suivant->globalNum)) {
							autorisedStacking = true;
						}
					}
				}
			}
			else {
				autorisedStacking = true;
			}
			vecteurLegalDeplacements[i] = autorisedStacking;
		}
		//std::cout << "Stack check: i: " << i << " autorised: " << vecteurLegalDeplacements[i] << std::endl;
	}
}

// Appelle checkDebordement et checkStacking puis renvoie un booleen indiquant si au moin un deplacement pourrait etre effectué
bool checkAtLeastOneMove(NodeBend* nb, int gridWidth, int gridHeight) {
	checkDebordement(nb, gridWidth, gridHeight);
	checkStacking(nb);
	// On prend la taille de vecteurDeplacements pour eviter le slot de 'pas bouger' dans vecteurLegalDeplacements
	for (int i = 0; i < vecteurDeplacements.size(); i++) {
		if (vecteurLegalDeplacements[i]) {
			return true;
		}
	}
	return false;
}

// Retourne une valeur réelle comprise dans [0.1,n]
double generateDoubleRand(double n) {
	std::random_device rd;  // Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(0.1, n);
	return dis(gen);
}

// Retourne une valeur entiere comprise dans [1,n]
int generateRand(int n) {
	std::random_device rd;  // Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<> dis(1, n);
	return dis(gen);
}

// Enleve le nodebend du set global pour les positions
void removeNodeBendPos(NodeBend* nb, int x, int y) {
	posVectorNodeBend[x][y].remove(nb);
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

// Modifie le vecteur de booléen global. Ces booleen indiquent si le déplacement a la meme position dans ce vecteur est valide ou non.
bool getLegalMoves(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem) {
	int srcX = n->getX();
	int srcY = n->getY();
	bool oneCanMove = false;
	bool intersection;
	SList<adjEntry> nodeAdjEntries;
	// Node a toujours le meme nombre d'adjEntry
	if (n->isNode) {
		n->getNode()->allAdjEntries(nodeAdjEntries);
	}
	std::list<NodeBend*>& listeNodeBends = posVectorNodeBend[srcX][srcY];
	// Pour chaque déplacement, on regarde si il y a une intersection associé
	int segmentSourceTrgX, segmentSourceTrgY;
	// Vecteur si le nodebend est un bend pour les potentiels suivants et précédents
	std::vector<NodeBend*> bendPrecSuiv;
	for (int i = 0; i < vecteurDeplacements.size(); i++) {
		if (vecteurLegalDeplacements[i]) {
			int newSrcX = srcX + vecteurDeplacements[i].first;
			int newSrcY = srcY + vecteurDeplacements[i].second;
			if ((n->isNode) && (moveBendsWithNode)) {
				for (auto it = listeNodeBends.begin(); it != listeNodeBends.end(); it++) {
					(*(*it)->a_x) = newSrcX;
					(*(*it)->a_y) = newSrcY;
				}
			}
			else {
				*n->a_x = newSrcX;
				*n->a_y = newSrcY;
			}
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
				std::pair<int, int> facesNum = n->getAdjEntryFaces((*it));
				int nombreFace = 1;
				if (facesNum.first != facesNum.second) {
					nombreFace++;
				}
				// Pour chaque adjEntry on parcours les deux faces si elles sont différentes
				for (int numFace = 0; numFace < nombreFace; numFace++) {
					int indexFace = facesNum.first;
					if (numFace == 1) {
						indexFace = facesNum.second;
					}
					// On vérifie que la deuxieme face est différente de la premiere
					std::vector<Segment*>& vectorAdjFaceSegments = vectorFaceSegment[indexFace];
					// On recupere la liste des segments grace au numero de face
					// Et on regarde si une intersection se créer avec la liste des segments non adjacents
					for (int k = 0; (k < vectorAdjFaceSegments.size()) && (!intersection); k++) {
						// On vérifie que le segment n'est pas nul
						if (!vectorAdjFaceSegments[k]->isNull()) {
							// On regarde si un NodeBend est commun aux deux segments ou s'il y a stacking
							NodeBend* nodeBendCommun = nullptr;
							std::pair<NodeBend*, NodeBend*> nodeBendNonCommun;
							bool is_in = ((newSrcX == vectorAdjFaceSegments[k]->source->getX()) && (newSrcY == vectorAdjFaceSegments[k]->source->getY()));
							bool is_in2 = ((newSrcX == vectorAdjFaceSegments[k]->target->getX()) && (newSrcY == vectorAdjFaceSegments[k]->target->getY()));
							bool is_in3 = ((target->getX() == vectorAdjFaceSegments[k]->source->getX()) && (target->getY() == vectorAdjFaceSegments[k]->source->getY()));
							bool is_in4 = ((target->getX() == vectorAdjFaceSegments[k]->target->getX()) && (target->getY() == vectorAdjFaceSegments[k]->target->getY()));
							bool same = ((is_in && (is_in3 || is_in4)) || (is_in2 && (is_in3 || is_in4)));
							bool stack = (is_in || is_in2 || is_in3 || is_in4);
							//std::cout << "Deplacement: " << i << " stack: " << stack << " same: " << same << " dx " << *n->a_x - srcX << " dy " << *n->a_y - srcY << std::endl;
							//std::cout << "x1: " << vectorMoveCoord[i].first << " y1: " << vectorMoveCoord[i].second << " x2: " << segmentSourceTrgX << " y2: " << segmentSourceTrgY << " x3: " << *vectorAdjFaceSegments[k]->sourceX << " y3: " << *vectorAdjFaceSegments[k]->sourceY << " x4: " << *vectorAdjFaceSegments[k]->targetX << " y4: " << *vectorAdjFaceSegments[k]->targetY << std::endl;
							// Si on compare deux segments identique on passe
							if (same) {
								// On verifie si on est le meme segment ou superposé completement
								// Si on passe ce if alors on est superposé mais le segment n'est pas le meme
								if (!(((vectorAdjFaceSegments[k]->target->globalNum == n->globalNum) && (vectorAdjFaceSegments[k]->source->globalNum == target->globalNum)) || ((vectorAdjFaceSegments[k]->target->globalNum == target->globalNum) && (vectorAdjFaceSegments[k]->source->globalNum == n->globalNum)))) {
									// On verifie que le segment existe
									if (!n->isNode) {
										if ((n->suivant->globalNum == target->globalNum) || (n->precedent->globalNum == target->globalNum)) {
											intersection = true;
											//std::cout << "Cas superposition complete" << std::endl;
											//std::cout << "n " << n->globalNum << " t " << target->globalNum << " v1 " << vectorAdjFaceSegments[k]->source->globalNum << " v2 " << vectorAdjFaceSegments[k]->target->globalNum << std::endl;
										}
									}
									else {
										for (int i = 0; (i < n->adjNodeBend.size()) && !intersection; i++) {
											if (n->adjNodeBend[i]->globalNum == target->globalNum) {
												intersection = true;
												//std::cout << "Cas superposition complete" << std::endl;
												//std::cout << "n " << n->globalNum << " t " << target->globalNum << " v1 " << vectorAdjFaceSegments[k]->source->globalNum << " v2 " << vectorAdjFaceSegments[k]->target->globalNum << std::endl;
											}
										}
									}
								}
							}
							else {
								// Si on a un stacking, il est autorisé donc on regarde uniquement la superposition
								if (stack) {
									if (is_in) {
										nodeBendCommun = vectorAdjFaceSegments[k]->source;
										nodeBendNonCommun.first = target;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k]->target;
									}
									else if (is_in2) {
										nodeBendCommun = vectorAdjFaceSegments[k]->target;
										nodeBendNonCommun.first = target;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k]->source;
									}
									else if (is_in3) {
										nodeBendCommun = vectorAdjFaceSegments[k]->source;
										nodeBendNonCommun.first = n;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k]->target;
									}
									else if (is_in4) {
										nodeBendCommun = vectorAdjFaceSegments[k]->target;
										nodeBendNonCommun.first = n;
										nodeBendNonCommun.second = vectorAdjFaceSegments[k]->source;
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
									if (seCroisent(newSrcX, newSrcY, segmentSourceTrgX, segmentSourceTrgY, *vectorAdjFaceSegments[k]->sourceX, *vectorAdjFaceSegments[k]->sourceY, *vectorAdjFaceSegments[k]->targetX, *vectorAdjFaceSegments[k]->targetY)) {
										intersection = true;
										//std::cout << "Cas: 2 i:" << i << " k: " << k << " j: " << j << std::endl;
										//std::cout << "x1: " << vectorMoveCoord[i].first << " y1: " << vectorMoveCoord[i].second << " x2: " << segmentSourceTrgX << " y2: " << segmentSourceTrgY << " x3: " << *vectorAdjFaceSegments[k]->sourceX << " y3: " << *vectorAdjFaceSegments[k]->sourceY << " x4: " << *vectorAdjFaceSegments[k]->targetX << " y4: " << *vectorAdjFaceSegments[k]->targetY << std::endl;
									}
								}
							}
						}
					}
				}
			}
			// On regarde si la face s'inverse ou que l'ordre autour d'un noeud change
			if (!intersection) {
				// On regarde si les ordre du noeud source ou des adjacents ont changé
				if (orderNodeAdjChanged(n, GL, newSrcX, newSrcY)) {
					intersection = true;
					//std::cout << "Cas: 4 i:" << i << std::endl;
				}
				// Sinon on regarde le cas particulier si le point qui se déplace inverse une face sans changer l'ordre
				// Pour cela il doit avoir 2 voisins et ses deux voisins aussi
				else if (!n->isNode) {
					// Les faces gauches et droites sont différentes, donc possibilité d'inversion de face sans changement d'ordre
					if (ccem.leftFace(n->getAdjEntry()) != ccem.rightFace(n->getAdjEntry())) {
						// On regarde si on est un triangle ou non
					}
				}
				else if (n->getNode()->degree() == 2) {
					if (ccem.leftFace(n->getNode()->firstAdj()) != ccem.rightFace(n->getNode()->firstAdj())) {
						// A FAIRE
					}
				}
			}
			// Intersection = déplacement pas autorisé
			vecteurLegalDeplacements[i] = !intersection;
			if (!intersection) {
				oneCanMove = true;
			}
		}
	}
	if ((n->isNode) && (moveBendsWithNode)) {
		for (auto it = listeNodeBends.begin(); it != listeNodeBends.end(); it++) {
			(*(*it)->a_x) = srcX;
			(*(*it)->a_y) = srcY;
		}
	}
	else {
		*n->a_x = srcX;
		*n->a_y = srcY;
	}
	return oneCanMove;
}

// Renvoie un vecteur qui attribue une probabilité a un déplacement
// Pour les déplacements: 0=droite(x+1) 1=haut(y+1) 2=gauche(x-1) 3=bas(y-1)
// Cette fonction doit etre appelée avant un déplacement
// Les poids assignés aux déplacements sont attribués en fonction de leur amélioration de l'écart-type
bool rouletteRusseNodeMove(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	atLeastOneMove = getLegalMoves(n, GL, ccem);
	if (atLeastOneMove) {
		// On stocke si les déplacements sont autorisés, donc s'il n'y a pas de node ou de bend a ces coordonnées
		SListPure<adjEntry> adjEntries;
		if (n->isNode) {
			n->getNode()->allAdjEntries(adjEntries);
		}
		else {
			adjEntries.pushBack(n->getAdjEntry());
		}
		// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
		double tmpMaxContribution = 0.0;
		double tmpMinContribution = 0.0;
		int numberMoveAutorised = 1; // 1 Car on ajoute le deplacement "par bouger"
		vecteurVarChangeMove[vecteurDeplacements.size()] = 0.0;
		// Boucle sur tout les déplacements possibles
		for (int i = 0; i < vecteurDeplacements.size(); i++) {
			// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
			if (vecteurLegalDeplacements[i]) {
				int newX = n->getX() + vecteurDeplacements[i].first;
				int newY = n->getY() + vecteurDeplacements[i].second;
				numberMoveAutorised++;
				double tmpSommeLong = sommeLong;
				double tmpSommeLong2 = sommeLong2;
				double tmpVariance = variance;
				for (auto it = adjEntries.begin(); it.valid(); it++) {
					auto it2 = mapEdgeLength.find((*it)->theEdge());
					double tmpOldLength = it2->second;
					double tmpNewLength;
					if (n->isNode) {
						tmpNewLength = calcTmpEdgeLength((*it), newX, newY, GL);
					}
					else {
						tmpNewLength = calcTmpEdgeLengthBends((*it)->theEdge(), n, newX, newY, GL);
					}
					deleteEdgeNVar(tmpOldLength, tmpSommeLong, tmpSommeLong2);
					addEdgeNVar(tmpNewLength, tmpSommeLong, tmpSommeLong2);
				}
				tmpVariance = calcNVar(tmpSommeLong, tmpSommeLong2);
				double tmpContribution = tmpVariance - variance;
				vecteurVarChangeMove[i] = tmpContribution;
				if (tmpContribution > tmpMaxContribution) {
					tmpMaxContribution = tmpContribution;
				}
				if (tmpContribution < tmpMinContribution) {
					tmpMinContribution = tmpContribution;
				}
			}
		}
		std::vector<std::pair<int, std::pair<int, int>>> vectorProbaMove;
		double tmpVarSomme = 0;
		// On soustrait a tout les valeurs la variance maximale
		for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
			if (vecteurLegalDeplacements[i]) {
				//std::cout << "Contribution Variance deplacement " << i << ": " << vectorVarChangeMove[i] << std::endl;
				if (tmpMaxContribution > 0.0) {
					vecteurVarChangeMove[i] = (-vecteurVarChangeMove[i]) + (2 * tmpMaxContribution);
				}
				else {
					vecteurVarChangeMove[i] = vecteurVarChangeMove[i] + (2 * (-tmpMinContribution));
				}
				tmpVarSomme += vecteurVarChangeMove[i];
			}
		}
		double tmpSommeProba = 0.0;
		// Si aucun deplacement ne modifie la variance
		if (tmpVarSomme == 0) {
			double tmpProba = 100.0 / (double)numberMoveAutorised;
			for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					tmpSommeProba += tmpProba;
					//std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
				}
				vecteurProbaMove[i] = tmpSommeProba;
			}
		}
		else {
			// On transforme les valeurs en proba
			for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					double tmpProba = (vecteurVarChangeMove[i] / tmpVarSomme) * 100.0;
					tmpSommeProba += tmpProba;
					//std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
				}
				vecteurProbaMove[i] = tmpSommeProba;
			}
		}
	}
	return atLeastOneMove;
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
	int srcX = (*nb->a_x);
	int srcY = (*nb->a_y);
	if ((moveBendsWithNode)&&(nb->isNode)) {
		std::list<NodeBend*>& listeNodeBends = posVectorNodeBend[nb->getX()][nb->getY()];
		for (auto it = listeNodeBends.begin(); it != listeNodeBends.end(); it++) {
			(*(*it)->a_x) = newX;
			(*(*it)->a_y) = newY;
			if ((*it)->isNode) {
				posVectorNodeBend[newX][newY].push_front((*it));
			}
			else {
				posVectorNodeBend[newX][newY].push_back((*it));
			}
		}
		listeNodeBends = posVectorNodeBend[newX][newY];
		for (auto it = listeNodeBends.begin(); it != listeNodeBends.end(); it++) {
			if ((*it)->isNode) {
				(*it)->recalculateAdjBendStack();
			}
			else {
				(*it)->recalculateIsStacked();
			}
		}
		posVectorNodeBend[srcX][srcY].clear();
	}
	else {
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

// Fonction de déplacement utilisée avec les raccourcis openGL, pas utilisée pour les tests finaux
void move(NodeBend* nb, GridLayout& GL, int dx, int dy, double& sommeLong, double& sommeLong2, double& variance) {
	int srcX = (*nb->a_x);
	int srcY = (*nb->a_y);
	int newX = (*nb->a_x) + dx;
	int newY = (*nb->a_y) + dy;
	if ((moveBendsWithNode) && (nb->isStacked) && (nb->isNode)) {
		std::list<NodeBend*>& listeNodeBends = posVectorNodeBend[nb->getX()][nb->getY()];
		int i = 0;
		for (auto it = listeNodeBends.begin(); it != listeNodeBends.end(); it++) {
			(*(*it)->a_x) = newX;
			(*(*it)->a_y) = newY;
			if ((*it)->isNode) {
				posVectorNodeBend[newX][newY].push_front((*it));
			}
			else {
				posVectorNodeBend[newX][newY].push_back((*it));
			}
			i++;
		}
		listeNodeBends = posVectorNodeBend[newX][newY];
		i = 0;
		for (auto it = listeNodeBends.begin(); it != listeNodeBends.end(); it++) {
			if ((*it)->isNode) {
				(*it)->recalculateAdjBendStack();
			}
			else {
				(*it)->recalculateIsStacked();
			}
			i++;
		}
		posVectorNodeBend[srcX][srcY].clear();
	}
	else {
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
	changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
}

// Demarre l'algorithme de roulette russe sur le NodeBend choisi a l'avance
void specificRouletteRusse(int numero, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	//std::cout << "Numero selectionne: " << numero << std::endl;
	NodeBend* nb = movableNodeBend(vectorNodeBends[numero]);
	//std::cout << "Chosen: " << numero << " new: " << nb->globalNum << std::endl;
	// Si au moin un deplacement possible
	if (checkAtLeastOneMove(nb, gridWidth, gridHeight)) {
		// Si apres tests au moin un deplacement possible
		if (rouletteRusseNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth)) {
			double randomChoice = generateDoubleRand(100.0);
			for (int i = 0; (i < vecteurDeplacements.size()); i++) {
				if (vecteurLegalDeplacements[i]) {
					if (randomChoice <= vecteurProbaMove[i]) {
						int newX = nb->getX() + vecteurDeplacements[i].first;
						int newY = nb->getY() + vecteurDeplacements[i].second;
						//std::cout << "Nombre aleatoire: " << randomChoice << " Deplacement choisi : " << i << std::endl;
						changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
						moveNodeBend(nb, newX, newY);
						break;
					}
				}
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
	NodeBend* nb = movableNodeBend(vectorNodeBends[randomNum]);
	// Si au moin un deplacement possible
	if (checkAtLeastOneMove(nb, gridWidth, gridHeight)) {
		// Si apres tests au moin un deplacement possible
		if (rouletteRusseNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth)) {
			double randomChoice = generateDoubleRand(100.0);
			for (int i = 0; (i < vecteurDeplacements.size()); i++) {
				if (vecteurLegalDeplacements[i]) {
					if (randomChoice <= vecteurProbaMove[i]) {
						int newX = nb->getX() + vecteurDeplacements[i].first;
						int newY = nb->getY() + vecteurDeplacements[i].second;
						//std::cout << "Nombre aleatoire: " << randomChoice << " Deplacement choisi : " << i << std::endl;
						changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
						moveNodeBend(nb, newX, newY);
						break;
					}
				}
			}
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return randomNum;
}

// Renvoie un booleen indiquant si au moin un deplacement peut etre effectué
// Modifie un vecteur qui attribue une probabilité au meme index que le deplacement global
// Cette fonction doit etre appelée avant un déplacement
// Les poids assignés aux déplacements sont attribués en fonction de leur amélioration de l'écart-type et du coefficient de recuit simulé
bool recuitSimuleNodeMove(NodeBend* n, double coeff, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	// On regarde les déplacements autorisés
	atLeastOneMove = getLegalMoves(n, GL, ccem);
	if (atLeastOneMove) {
		SListPure<adjEntry> adjEntries;
		if (n->isNode) {
			n->getNode()->allAdjEntries(adjEntries);
		}
		else {
			adjEntries.pushBack(n->getAdjEntry());
		}
		// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
		double tmpMaxContribution = 0.0;
		double tmpMinContribution = 0.0;
		int numberMoveAutorised = 1;
		vecteurVarChangeMove[vecteurDeplacements.size()] = 0.0;
		// Boucle sur tout les déplacements possibles
		// On stocke les changements de variances apres un déplacement, ce vecteur aura une taille de 1 de plus du vecteur global pour le deplacement pas bouger
		for (int i = 0; i < vecteurDeplacements.size(); i++) {
			// On regarde si le déplacement est autorisé
			if (vecteurLegalDeplacements[i]) {
				int newX = n->getX() + vecteurDeplacements[i].first;
				int newY = n->getY() + vecteurDeplacements[i].second;
				numberMoveAutorised++;
				double tmpSommeLong = sommeLong;
				double tmpSommeLong2 = sommeLong2;
				double tmpVariance = variance;
				for (auto it = adjEntries.begin(); it.valid(); it++) {
					auto it2 = mapEdgeLength.find((*it)->theEdge());
					double tmpOldLength = it2->second;
					double tmpNewLength;
					if (n->isNode) {
						tmpNewLength = calcTmpEdgeLength((*it), newX, newY, GL);
					}
					else {
						tmpNewLength = calcTmpEdgeLengthBends((*it)->theEdge(), n, newX, newY, GL);
					}
					deleteEdgeNVar(tmpOldLength, tmpSommeLong, tmpSommeLong2);
					addEdgeNVar(tmpNewLength, tmpSommeLong, tmpSommeLong2);
				}
				tmpVariance = calcNVar(tmpSommeLong, tmpSommeLong2);
				double tmpContribution = tmpVariance - variance;
				vecteurVarChangeMove[i] = tmpContribution;
				if (tmpContribution > tmpMaxContribution) {
					tmpMaxContribution = tmpContribution;
				}
				if (tmpContribution < tmpMinContribution) {
					tmpMinContribution = tmpContribution;
				}
			}
		}
		double tmpVarSomme = 0;
		// contribution negative = amélioration
		// On soustrait a tout les valeurs la variance maximale
		for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
			if (vecteurLegalDeplacements[i]) {
				//std::cout << "Contribution Variance deplacement " << i << ": " << vectorVarChangeMove[i] << std::endl;
				if (tmpMaxContribution > 0.0) {
					vecteurVarChangeMove[i] = (-1)*vecteurVarChangeMove[i] + (2 * tmpMaxContribution);
				}
				else {
					vecteurVarChangeMove[i] = vecteurVarChangeMove[i] + (2 * (abs(tmpMinContribution)));
				}
				tmpVarSomme += vecteurVarChangeMove[i];
			}
		}
		// Si la somme des contribution est égale a 0, alors tout les déplacements ont la meme proba, pas besoin d'appliquer le coeff
		double tmpSommeProba = 0.0;
		if (tmpVarSomme == 0) {
			double tmpProba = 100.0 / (double)numberMoveAutorised;
			for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					tmpSommeProba += tmpProba;
					//std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
				}
				vecteurProbaMove[i] = tmpSommeProba;
			}
		}
		// Il y a au moin un déplacement qui change la variance
		else {
			// On applique le coefficient aux probabilités
			double tmpSommeProbaFraction = 0;
			for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					double tmpProbaFraction = vecteurVarChangeMove[i] / tmpVarSomme;
					tmpProbaFraction = pow(tmpProbaFraction, coeff);
					tmpSommeProbaFraction += tmpProbaFraction;
					vecteurProbaCoeff[i] = tmpProbaFraction;
				}
			}
			// On transforme ces valeurs en probabilité
			for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					double tmpProba = (vecteurProbaCoeff[i] / tmpSommeProbaFraction) * 100.0;
					tmpSommeProba += tmpProba;
					//std::cout << "Deplacement " << i << " Proba: " << tmpProba << " SommeProba: " << tmpSommeProba << std::endl;
				}
				vecteurProbaMove[i] = tmpSommeProba;
			}
		}
		//std::cout << "Deplacement " << size << " Proba: " << 100 - tmpSommeProba << " SommeProba: " << 100 << std::endl;
	}
	return atLeastOneMove;
}

void specificRecuitSimule(int selectedNodeBendNum,double coeff, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	// On choisis au hasard un NodeBend
	//std::cout << "Numero selectionne: " << randomNum << std::endl;
	NodeBend* nb = movableNodeBend(vectorNodeBends[selectedNodeBendNum]);
	// Si au moin un deplacement possible
	if (checkAtLeastOneMove(nb, gridWidth, gridHeight)) {
		// Si apres tests au moin un deplacement possible
		if (recuitSimuleNodeMove(nb, coeff, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth)) {
			for (int i = 0; i < vecteurProbaMove.size(); i++) {
				//std::cout << "Deplacement: " << i << ": " << vecteurLegalDeplacements[i] << " Proba: " << vecteurProbaMove[i] << std::endl;
			}
			double randomChoice = generateDoubleRand(100.0);
			// Le déplacement est choisi aléatoirement
			for (int i = 0; (i < vecteurDeplacements.size()); i++) {
				if (vecteurLegalDeplacements[i]) {
					if (randomChoice <= vecteurProbaMove[i]) {
						int newX = nb->getX() + vecteurDeplacements[i].first;
						int newY = nb->getY() + vecteurDeplacements[i].second;
						//std::cout << "New Coord x: " << newX << " y: " << newY << std::endl;
						//std::cout << "Nombre aleatoire: " << randomChoice << " Deplacement choisi : " << i << std::endl;
						changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
						moveNodeBend(nb, newX, newY);
						break;
					}
				}
			}
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
}

// Demarre l'algorithme de recuit simulé sur le graphe
// Les probas sont calculés avec l'algorithme roulette russe et modifiée avec un coefficient évoluant avec le temps
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
int startRecuitSimule(double coeff, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	// On choisis au hasard un NodeBend
	int randomNum = generateRand(vectorNodeBends.size()) - 1;
	//std::cout << "Numero selectionne: " << randomNum << std::endl;
	NodeBend* nb = movableNodeBend(vectorNodeBends[randomNum]);
	// Si au moin un deplacement possible
	if (checkAtLeastOneMove(nb, gridWidth, gridHeight)) {
		// Si apres tests au moin un deplacement possible
		if (recuitSimuleNodeMove(nb, coeff, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth)) {
			//for (int i = 0; i < vecteurProbaMove.size(); i++) {
				//std::cout << "Deplacement: " << i << ": " << vecteurLegalDeplacements[i] << " Proba: " << vecteurProbaMove[i] << std::endl;
			//}
			double randomChoice = generateDoubleRand(100.0);
			// Le déplacement est choisi aléatoirement
			//std::cout << "Nombre aleatoire: " << randomChoice;
			for (int i = 0; (i < vecteurDeplacements.size()); i++) {
				if (vecteurLegalDeplacements[i]) {
					if (randomChoice <= vecteurProbaMove[i]) {
						int newX = nb->getX() + vecteurDeplacements[i].first;
						int newY = nb->getY() + vecteurDeplacements[i].second;
						//std::cout << " Deplacement choisi : " << i << std::endl;
						changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
						moveNodeBend(nb, newX, newY);
						break;
					}
				}
			}
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return randomNum;
}

// Renvoie un vecteur composé des meilleurs déplacements améliorant la variance
// Cette fonction doit etre appelée avant un déplacement
bool bestVarianceNodeMove(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	// On regarde les déplacements autorisés
	atLeastOneMove = getLegalMoves(n, GL, ccem);
	if (atLeastOneMove) {
		SListPure<adjEntry> adjEntries;
		if (n->isNode) {
			n->getNode()->allAdjEntries(adjEntries);
		}
		else {
			adjEntries.pushBack(n->getAdjEntry());
		}
		// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
		double tmpMinContribution = 0.0;
		// Boucle sur tout les déplacements possibles
		for (int i = 0; i < vecteurDeplacements.size(); i++) {
			// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
			if (vecteurLegalDeplacements[i]) {
				int newX = n->getX() + vecteurDeplacements[i].first;
				int newY = n->getY() + vecteurDeplacements[i].second;
				double tmpSommeLong = sommeLong;
				double tmpSommeLong2 = sommeLong2;
				double tmpVariance = variance;
				for (auto it = adjEntries.begin(); it.valid(); it++) {
					auto it2 = mapEdgeLength.find((*it)->theEdge());
					double tmpOldLength = it2->second;
					double tmpNewLength;
					if (n->isNode) {
						tmpNewLength = calcTmpEdgeLength((*it), newX, newY, GL);
					}
					else {
						tmpNewLength = calcTmpEdgeLengthBends((*it)->theEdge(), n, newX, newY, GL);
					}
					deleteEdgeNVar(tmpOldLength, tmpSommeLong, tmpSommeLong2);
					addEdgeNVar(tmpNewLength, tmpSommeLong, tmpSommeLong2);
				}
				tmpVariance = calcNVar(tmpSommeLong, tmpSommeLong2);
				double tmpContribution = tmpVariance - variance;
				vecteurVarChangeMove[i] = tmpContribution;
				if (tmpContribution < tmpMinContribution) {
					tmpMinContribution = tmpContribution;
				}

			}
		}

		// S'il y a au moin un déplacement améliorant la variance
		if (tmpMinContribution < -0.000001) {
			vectorBestVarianceMove.clear();
			// On stocke ces déplacements
			for (int i = 0; i < vecteurDeplacements.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					if ((vecteurVarChangeMove[i] < tmpMinContribution + 0.000001) && (vecteurVarChangeMove[i] > tmpMinContribution - 0.000001)) {
						vectorBestVarianceMove.push_back(i);
					}
				}
			}
		}
		else {
			return false;
		}
	}
	return atLeastOneMove;
}

// Demarre l'algorithme de best variance
// On choisis uniquement les déplacements qui améliorent le plus la variance, s'ils sont egaux on tire au hasard.
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
// On itere sur les noeuds dans l'ordre
int startBestVariance(GridLayout& GL, ConstCombinatorialEmbedding& ccem, int numCourant, int& numLastMoved, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	NodeBend* nb = vectorNodeBends[numCourant];
	if (checkAtLeastOneMove(nb, gridWidth, gridHeight)) {
		if (bestVarianceNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth)) {
			numLastMoved = numCourant;
			int randomChoice = generateRand(vectorBestVarianceMove.size()) - 1;
			randomChoice = vectorBestVarianceMove[randomChoice];
			int newX = nb->getX() + vecteurDeplacements[randomChoice].first;
			int newY = nb->getY() + vecteurDeplacements[randomChoice].second;
			changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
			moveNodeBend(nb, newX, newY);
		}
	}
	//std::cout << "Nouvelle variance " << variance << std::endl;
	return numCourant;
}

// Renvoie un vecteur composé des meilleurs déplacements réduisant les longueurs autour d'un noeud
// Cette fonction doit etre appelée avant un déplacement
bool shortestLengthNodeMove(NodeBend* n, GridLayout& GL, ConstCombinatorialEmbedding& ccem, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	atLeastOneMove = getLegalMoves(n, GL, ccem);
	if (atLeastOneMove) {
		SListPure<adjEntry> adjEntries;
		if (n->isNode) {
			n->getNode()->allAdjEntries(adjEntries);
		}
		else {
			adjEntries.pushBack(n->getAdjEntry());
		}
		// Minimum et Maximum des variances des différents déplacements pour calcul de probabilité plus tard
		double longueurActuelle = totalLengthAroundNodeBend(n, GL, n->getX(), n->getY());
		double minLongeurs = longueurActuelle;
		int numberMoveAutorised = 1;
		// Boucle sur tout les déplacements possibles
		for (int i = 0; i < vecteurDeplacements.size(); i++) {
			// On regarde si le déplacement est autorisé (si on ne se déplace par sur une node ou un bend)
			if (vecteurLegalDeplacements[i]) {
				int newX = n->getX() + vecteurDeplacements[i].first;
				int newY = n->getY() + vecteurDeplacements[i].second;
				numberMoveAutorised++;
				double tmpLongueur = totalLengthAroundNodeBend(n, GL, newX, newY);
				vecteurVarChangeMove.push_back(tmpLongueur);
				if (tmpLongueur < minLongeurs) {
					minLongeurs = tmpLongueur;
				}
			}
		}
		// S'il y a au moin un déplacement améliorant la variance
		if (minLongeurs < longueurActuelle) {
			// On stocke ces déplacements
			for (int i = 0; i < vecteurVarChangeMove.size(); i++) {
				if (vecteurLegalDeplacements[i]) {
					if ((vecteurVarChangeMove[i] < minLongeurs + 0.000001) && (vecteurVarChangeMove[i] > minLongeurs - 0.000001)) {
						vectorBestVarianceMove.push_back(i);
					}
				}
			}
		}
		else {
			return false;
		}
	}
	return atLeastOneMove;
}

// Demarre l'algorithme de longueur minimale
// On choisis uniquement les déplacements qui réduisent les longueurs autour d'un node, s'ils sont egaux on tire au hasard.
// retourne le numero du nodebend choisi, uniquement utile pour l'affichage opengl
// On itere sur les noeuds dans l'ordre
int startShortestLength(GridLayout& GL, ConstCombinatorialEmbedding& ccem, int numCourant, int& numLastMoved, double& sommeLong, double& sommeLong2, double& variance, int gridHeight, int gridWidth) {
	NodeBend* nb = vectorNodeBends[numCourant];
	if (checkAtLeastOneMove(nb, gridWidth, gridHeight)) {
		if (shortestLengthNodeMove(nb, GL, ccem, sommeLong, sommeLong2, variance, gridHeight, gridWidth)) {
			numLastMoved = numCourant;
			int randomChoice = generateRand(vectorBestVarianceMove.size()) - 1;
			randomChoice = vectorBestVarianceMove[randomChoice];
			int newX = nb->getX() + vecteurDeplacements[randomChoice].first;
			int newY = nb->getY() + vecteurDeplacements[randomChoice].second;
			changeVariance(nb, GL, newX, newY, sommeLong, sommeLong2, variance);
			moveNodeBend(nb, newX, newY);
		}
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