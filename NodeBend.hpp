#ifndef NODEBEND_HPP
#define NODEBEND_HPP
#include <ogdf/basic/GridLayout.h>
#include <ogdf/basic/EdgeArray.h>
#include <ogdf/fileformats/GraphIO.h>
#include "geometrie.hpp"
using namespace ogdf;
class NodeBend {
public:
	std::map<adjEntry, int> adjBendsStack; // Nombre de bend de cette adjentry stacké sur la node
	bool isNode;
	int* a_x;
	int* a_y;
	int numero; // Numéro du bend dans l'edge
	int globalNum = 0; // Numero du NodeBend dans le vecteur global de NodeBend
	bool isStacked = false;
	int stack = 0;
	// indique si des bends de differents adjEntry sont stacké sur un node
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
private:
	node m_n = nullptr;
	edge m_e = nullptr;
	IPoint* m_p = nullptr;
};
#endif