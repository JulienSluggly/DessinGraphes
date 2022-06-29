#ifndef NODEBEND_HPP
#define NODEBEND_HPP
#include <ogdf/basic/GridLayout.h>
#include <ogdf/basic/EdgeArray.h>
#include <ogdf/fileformats/GraphIO.h>
using namespace ogdf;
class NodeBend {
public:
	bool isNode;
	int* a_x;
	int* a_y;
	int numero;
	NodeBend(node n, GridLayout& GL) {
		isNode = true;
		m_n = n;
		a_x = &GL.x(n);
		a_y = &GL.y(n);
		numero = -1;
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
private:
	node m_n = nullptr;
	edge m_e = nullptr;
	IPoint* m_p = nullptr;
};
#endif