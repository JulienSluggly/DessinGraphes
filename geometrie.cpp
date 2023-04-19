#include "geometrie.hpp"
#include <algorithm>

double double_epsilon = 0.000001;

//renvoie 1,2,3 ou 4 si lpoint t est 1: en haut � droite, 2: en haut � gauche, 3: en bas � gauche, 4: en bas � droite du point s
//on consid�re s != t
int quadrant(int sx, int sy, int tx, int ty) {
	if (tx > sx) {
		if (ty >= sy) {
			return 1;
		}
		return 4;
	}
	if (ty > sy) {
		return 2;
	}
	return 3;
}

//s= source, t=target, c=comp=nouveau noeud à ajouter, renvoie vrai si c est à gauche de (s;t) faux sinon
bool aGauche(int sx, int sy, int tx, int ty, int cx, int cy) {
	return ((static_cast<long long>(tx) - sx) * (static_cast<long long>(cy) - sy) -
		(static_cast<long long>(ty) - sy) * (static_cast<long long>(cx) - sx)) > 0;
}
//si 'c' est à gauche de (s;t) renvoie 1, si 'c' est à droite de (s;t) renvoie -1, si 'c','s' et 't' sont alignés renvoie 0
int aGaucheInt(int sx, int sy, int tx, int ty, int cx, int cy) {
	long long det = ((static_cast<long long>(tx) - sx) * (static_cast<long long>(cy) - sy) -
		(static_cast<long long>(ty) - sy) * (static_cast<long long>(cx) - sx));
	if (det > 0) return 1;
	if (det < 0) return -1;
	return 0;
}

//si c est à gauche de (s;t) renvoie 1, si c est à droite de (s;t) renvoie -1, si c,s et t sont alignés renvoie 0
int aGaucheIntReel(double sx, double sy, double tx, double ty, double cx, double cy) {
	double det = ((tx - sx) * (cy - sy) - (ty - sy) * (cx - sx));
	if (det > double_epsilon) return 1;
	if (det < -double_epsilon) return -1;
	return 0;
}

// Renvoie vrai si c est dans le rectangle form" par st
bool dansRectangle(int sx,int sy,int tx,int ty,int cx,int cy) {
	return (cx <= std::max(sx, tx) && cx >= std::min(sx, tx) && cy <= std::max(sy, ty) && cy >= std::min(sy, ty));
}

bool dansRectangle(Emplacement* corner1, Emplacement* corner2, Emplacement* point) {
	return dansRectangle(corner1->getX(), corner1->getY(), 
		corner2->getX(), corner2->getY(), point->getX(), point->getY());
}

bool dansRectangle(const Noeud &noeud1, const Noeud &noeud2, const Noeud &noeud) {
	return dansRectangle(noeud1.getX(), noeud1.getY(),
		noeud2.getX(), noeud2.getY(), noeud.getX(), noeud.getY());
}

bool dansRectangle(const Aretes &aretes, const Noeud &noeud) {
	return dansRectangle(aretes.getNoeud1()->getEmplacement(), 
		aretes.getNoeud2()->getEmplacement(), noeud.getEmplacement());
}

// Renvoie vrai si c est sur le segment st
bool oldSurSegment(int sx, int sy, int tx, int ty, int cx, int cy) {
	return (dansRectangle(sx, sy, tx, ty, cx, cy) && (aGaucheInt(sx, sy, tx, ty, cx, cy) == 0));
}

bool oldSurSegment(Emplacement* s, Emplacement* t, Emplacement* c) {
	return oldSurSegment(s->getX(), s->getY(), t->getX(), t->getY(), c->getX(), c->getY());
}

bool oldSurSegment(const Aretes& lien, const Noeud& noeud) {
	return oldSurSegment(lien.getNoeud1()->getEmplacement(),
		lien.getNoeud2()->getEmplacement(), noeud.getEmplacement());
}
