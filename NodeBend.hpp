#ifndef NODEBEND_HPP
#define NODEBEND_HPP
#include <ogdf/basic/GridLayout.h>
#include <ogdf/basic/EdgeArray.h>
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/planarlayout/PlanarStraightLayout.h>
#include <ogdf/planarity/EmbedderMinDepth.h>
#include <ogdf/planarity/SimpleEmbedder.h>
#include <ogdf/basic/simple_graph_alg.h>
#include "geometrie.hpp"
using namespace ogdf;

class NodeBend {
public:
	// Nombre de bend de cette adjentry stacké sur la node
	//std::unordered_map<adjEntry, int> adjBendsStack; 
	// Booléen indiquant si le nodebend est un node ou non
	bool isNode;
	// Pointeur sur les coordonnée du node ou bend dans l'object Graph
	int* a_x; int* a_y;
	// Numéro du bend dans l'edge, -1 pour les nodes
	int numero;
	// Numero du NodeBend dans le vecteur global de NodeBend
	int globalNum = 0;
	bool isStacked = false;
	// Indique si le NodeBend est dans la grille ou non
	bool isInGrille;
	// Nombre de nodebend stacké avec celui ci
	int stack = 0;
	// indique si des bends de differents adjEntry sont stacké sur un node
	int nbDiffAdjStacked = 0;
	// parent1 et 2 pour les bends uniquement, ce sont les nodebend source et target de l'edge associé au nodebend
	NodeBend* parent1 = nullptr; NodeBend* parent2 = nullptr;
	// suivant et précédent pour les bends uniquements
	NodeBend* suivant = nullptr; NodeBend* precedent = nullptr;
	// adjNodeBend pour les node uniquement
	std::vector<NodeBend*> adjNodeBend;
	// Constructeur pour les nodes
	NodeBend(node n, GridLayout& GL) {
		isNode = true;
		m_n = n;
		a_x = &GL.x(n);
		a_y = &GL.y(n);
		numero = -1;
	}
	// Constructeur pour les bends
	NodeBend(IPoint& p, edge e, int num) {
		isNode = false;
		m_p = &p;
		m_e = e;
		a_x = &p.m_x;
		a_y = &p.m_y;
		numero = num;
	}
	// Uniquement pour les node
	inline node getNode() {
		return m_n;
	}
	// Uniquement pour les bends
	inline IPoint* getPoint() {
		return m_p;
	}
	inline edge getEdge() {
		return m_e;
	}
	// Uniquement pour les bends, envoie l'adjEntry associé au point source de l'edge qui est associé au nodebend
	adjEntry getAdjEntry() {
		if (!isNode) {
			return m_e->adjSource();
		}
		return nullptr;
	}
	inline int getX() {
		return *a_x;
	}
	inline int getY() {
		return *a_y;
	}
	// Numéro global unique du nodebend
	void assignGlobalNum(int i) {
		globalNum = i;
	}
	// A appeler sur les node uniquement, indique le nombre d'adjentry différente stacké sur le nodebend
	void initStackCheck() {
		nbDiffAdjStacked = 0;
		for (int i = 0; i < adjNodeBend.size(); i++) {
			if ((*adjNodeBend[i]->a_x == *this->a_x) && (*adjNodeBend[i]->a_y == *this->a_y)) {
				nbDiffAdjStacked++;
			}
		}
	}
	// Recalcule si le nodebend est stacké ou non, pour un node bien appeler initStackCheck() ou recalculateAdjBendStack() avant a la création/copie de graphe
	void recalculateIsStacked() {
		if (this->isNode) {
			this->isStacked = nbDiffAdjStacked > 0;
		}
		else {
			this->isStacked = (((*precedent->a_x == *this->a_x) && (*precedent->a_y == *this->a_y)) || ((*suivant->a_x == *this->a_x) && (*suivant->a_y == *this->a_y)));
		}
	}
	// A appeler sur les node uniquement, recalcule le nombre d'adjentry différente stacké sur le nodebend en recalculant le stacking des adjentry
	void recalculateAdjBendStack() {
		nbDiffAdjStacked = 0;
		for (int i = 0; i < adjNodeBend.size(); i++) {
			adjNodeBend[i]->recalculateIsStacked();
			if ((*adjNodeBend[i]->a_x == *this->a_x) && (*adjNodeBend[i]->a_y == *this->a_y)) {
				nbDiffAdjStacked++;
			}
		}
		this->isStacked = nbDiffAdjStacked > 0;
	}
	// Indique si le nodebend est stacké avec le précédent et le suivant, uniquement pour les bends
	bool isStuck() {
		return ((this->getX() == this->precedent->getX()) && (this->getY() == this->precedent->getY()) && (this->getX() == this->suivant->getX()) && (this->getY() == this->suivant->getY()));
	}
	// Indique si le nodebend peut se stack avec le nodebend nb
	bool isStackableWith(NodeBend* nb) {
		// Sois meme
		if (this->globalNum == nb->globalNum) {
			return true;
		}
		// Node
		if (this->isNode) {
			// Node avec Node
			if (nb->isNode) {
				return false;
			}
			// Node avec Bend
			else {
				// Node est suivant ou precedent direct du Bend
				if ((nb->precedent->globalNum == this->globalNum) || (nb->suivant->globalNum == this->globalNum)) {
					return true;
				}
				// Bend est stacké sur un des premiers d'une adjentry
				else {
					for (int i = 0; i < this->adjNodeBend.size(); i++) {
						if ((*nb->a_x == *this->adjNodeBend[i]->a_x) && (*nb->a_y == *this->adjNodeBend[i]->a_y)) {
							return true;
						}
					}
					return false;
				}
			}
		}
		// Bend
		else {
			// NB est un precedent ou suivant direct
			if ((this->precedent->globalNum == nb->globalNum) || (this->suivant->globalNum == nb->globalNum)) {
				return true;
			}
			// Pas precedent direct, on regarde si le precedent ou suivant du Bend est stacké avec NB
			if (((this->precedent->getX() == nb->getX()) && (this->precedent->getY() == nb->getY())) || ((this->suivant->getX() == nb->getX()) && (this->suivant->getY() == nb->getY()))) {
				return true;
			}
			return false;
		}
	}

	// Indique si le nodebend est stacké avec ce nodebend et si ce stacking est autorisé
	bool isAutorisedStacked(NodeBend* nb) {
		if ((this->getX() == nb->getX()) && (this->getY() == nb->getY())) {
			if (this->isStackableWith(nb)) {
				return true;
			}
		}
		return false;
	}

	// Indique si le bend est stacké sur l'un de ses parents
	bool isStackedOnParent() {
		return (((this->getX() == this->parent1->getX()) && (this->getY() == this->parent1->getY())) || ((this->getX() == this->parent2->getX()) && (this->getY() == this->parent2->getY())));
	}

private:
	node m_n = nullptr;
	edge m_e = nullptr;
	IPoint* m_p = nullptr;
};
#endif