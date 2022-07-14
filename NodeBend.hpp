#ifndef NODEBEND_HPP
#define NODEBEND_HPP
#include <ogdf/basic/GridLayout.h>
#include <ogdf/basic/EdgeArray.h>
#include <ogdf/fileformats/GraphIO.h>
#include "geometrie.hpp"
using namespace ogdf;

class NodeBend {
public:
	std::map<adjEntry, int> adjBendsStack; // Nombre de bend de cette adjentry stack� sur la node
	bool isNode;
	int* a_x;
	int* a_y;
	int numero; // Num�ro du bend dans l'edge
	int globalNum = 0; // Numero du NodeBend dans le vecteur global de NodeBend
	bool isStacked = false;
	int stack = 0;
	// indique si des bends de differents adjEntry sont stack� sur un node
	int nbDiffAdjStacked = 0;
	// parent1 et 2 pour les bends uniquement
	NodeBend* parent1 = nullptr;
	NodeBend* parent2 = nullptr;
	NodeBend* suivant = nullptr;
	NodeBend* precedent = nullptr;
	// adjNodeBend pour les node uniquement
	std::vector<NodeBend*> adjNodeBend;
	// Liste de tout les segments des faces adjacentes au NodeBend
	std::vector<Segment> adjFaceSegment;
	// Map des segments pouvant avoir une intersection avec une adjEntry
	std::map<adjEntry, std::vector<Segment>> mapSegmentInter;
	NodeBend(node n, GridLayout& GL) {
		isNode = true;
		m_n = n;
		a_x = &GL.x(n);
		a_y = &GL.y(n);
		numero = -1;
		ListPure<adjEntry> nodeAdjEntries;
		n->allAdjEntries(nodeAdjEntries);
		for (auto it = nodeAdjEntries.begin(); it.valid(); it++) {
			adjBendsStack.insert(std::pair<adjEntry, int>((*it), 0));
			std::vector<Segment> tmpVector;
			mapSegmentInter.insert(std::pair<adjEntry, std::vector<Segment>>((*it), tmpVector));
		}
	}
	NodeBend(IPoint& p, edge e, int num) {
		isNode = false;
		m_p = &p;
		m_e = e;
		a_x = &p.m_x;
		a_y = &p.m_y;
		numero = num;
	}
	node getNode() {
		return m_n;
	}
	IPoint* getPoint() {
		return m_p;
	}
	edge getEdge() {
		return m_e;
	}
	adjEntry getAdjEntry() {
		if (!isNode) {
			return m_e->adjSource();
		}
		return nullptr;
	}
	int getX() {
		return *a_x;
	}
	int getY() {
		return *a_y;
	}
	void assignGlobalNum(int i) {
		globalNum = i;
	}
	void recalculateIsStacked() {
		if (this->isNode) {
			this->isStacked = nbDiffAdjStacked > 0;
		}
		else {
			this->isStacked = (((precedent->a_x == this->a_x) && (precedent->a_y == this->a_y)) || ((suivant->a_x == this->a_x) && (suivant->a_y == this->a_y)));
		}
	}
	void recalculateAdjBendStack() {
		nbDiffAdjStacked = 0;
		for (int i = 0; i < adjNodeBend.size(); i++) {
			adjNodeBend[i]->recalculateIsStacked();
			if ((adjNodeBend[i]->a_x == this->a_x) && (adjNodeBend[i]->a_y == this->a_y)) {
				nbDiffAdjStacked++;
			}
		}
	}
	void addAdjNodeBend(NodeBend* nb) {
		adjNodeBend.push_back(nb);
	}
	void addAdjFaceSegment(Segment s) {
		adjFaceSegment.push_back(s);
	}
	void insertSegmentToAdjEntry(adjEntry adj, Segment s) {
		auto it = mapSegmentInter.find(adj);
		if (it != mapSegmentInter.end())
			it->second.push_back(s);
	}
	// Indique si le nodebend peut se stack avec le nodebend nb
	bool isStackableWith(NodeBend* nb) {
		if (this->globalNum == nb->globalNum) {
			return true;
		}
		if (this->isNode) {
			if (nb->isNode) {
				return false;
			}
			else {
				if ((nb->precedent->globalNum == this->globalNum) || (nb->suivant->globalNum == this->globalNum)) {
					return true;
				}
				else {
					for (int i = 0; i < this->adjNodeBend.size(); i++) {
						if ((*nb->a_x == *this->adjNodeBend[i]->a_x)&&(*nb->a_y == *this->adjNodeBend[i]->a_y)) {
							return true;
						}
					}
					return false;
				}
			}
		}
		else {
			if ((this->precedent->globalNum == nb->globalNum) || (this->suivant->globalNum == nb->globalNum)) {
				return true;
			}
			else {
				NodeBend* prec = this->precedent;
				int precX = *this->precedent->a_x;
				int precY = *this->precedent->a_y;
				// Recherche si nb est un precedent
				while (!prec->isNode) {
					prec = prec->precedent;
					if ((*prec->a_x != precX) || (*prec->a_y != precY)) {
						break;
					}
					if (prec->globalNum == nb->globalNum) {
						return true;
					}
				}
				NodeBend* suiv = this->suivant;
				int suivX = *this->suivant->a_x;
				int suivY = *this->suivant->a_y;
				// Recherche si nb est un suivant
				while (!suiv->isNode) {
					suiv = suiv->suivant;
					if ((*suiv->a_x != suivX) || (*suiv->a_y != suivY)) {
						break;
					}
					if (suiv->globalNum == nb->globalNum) {
						return true;
					}
				}
				// On regarde si ils sont stack�s sur un noeud en commun
				bool diffBendStacked = false;
				if (!nb->isNode) {
					if ((this->parent1->globalNum == nb->parent1->globalNum) || (this->parent1->globalNum == nb->parent2->globalNum)) {
						diffBendStacked = diffBendStacked || ((*this->parent1->a_x == *this->a_x) && (*this->parent1->a_y == *this->a_y) && (*this->parent1->a_x == *nb->a_x) && (*this->parent1->a_y == *nb->a_y));
					}
					else if ((this->parent2->globalNum == nb->parent1->globalNum) || (this->parent2->globalNum == nb->parent2->globalNum)) {
						diffBendStacked = diffBendStacked || ((*this->parent2->a_x == *this->a_x) && (*this->parent2->a_y == *this->a_y) && (*this->parent2->a_x == *nb->a_x) && (*this->parent2->a_y == *nb->a_y));
					}
				}
				return diffBendStacked;
			}
		}
	}
private:
	node m_n = nullptr;
	edge m_e = nullptr;
	IPoint* m_p = nullptr;
};
#endif