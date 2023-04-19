#include "intersection.hpp"
#include "geometrie.hpp"

using std::min;
using std::max;

int area2(int ax, int ay, int bx, int by, int cx, int cy) {
	return (bx-ax)*(cy-ay)-(cx-ax)*(by-ay);
}

bool left(int ax, int ay, int bx, int by, int cx, int cy) {
	return area2(ax,ay,bx,by,cx,cy) > 0;
}

bool leftOn(int ax,int ay, int bx, int by, int cx, int cy) {
	return area2(ax,ay,bx,by,cx,cy) >= 0;
}

bool collinear(int ax,int ay,int bx, int by, int cx, int cy) {
	return area2(ax,ay,bx,by,cx,cy) == 0;
}

bool xorBool(bool x, bool y) {
	return !x ^ !y;
}

bool intersectProp(int ax, int ay, int bx, int by, int cx, int cy,int dx,int dy) {
	if (collinear(ax,ay,bx,by,cx,cy)||collinear(ax,ay,bx,by,dx,dy)||collinear(cx,cy,dx,dy,ax,ay)||collinear(cx,cy,dx,dy,bx,by))
		return false;
	return xorBool(left(ax,ay,bx,by,cx,cy),left(ax,ay,bx,by,dx,dy))&&xorBool(left(cx,cy,dx,dy,ax,ay),left(cx,cy,dx,dy,bx,by));
}

bool between(int ax,int ay,int bx, int by, int cx, int cy) {
	if (!collinear(ax,ay,bx,by,cx,cy))
		return false;
	if (ax != bx)
		return ((ax <= cx)&&(cx <= bx))||((ax >= cx)&&(cx >= bx));
	else
		return ((ay <= cy)&&(cy <= by)||((ay >= cy)&&(cy >= by)));
}

bool seCroisent(int ax, int ay,int bx,int by,int cx,int cy,int dx,int dy, bool& isIllegal) {
	if (intersectProp(ax,ay,bx,by,cx,cy,dx,dy))
		return true;
	else if (between(ax,ay,bx,by,cx,cy)||between(ax,ay,bx,by,dx,dy)||between(cx,cy,dx,dy,ax,ay)||between(cx,cy,dx,dy,bx,by)) {
		isIllegal = true;
		return true;
	}
	return false;
}

bool seCroisent(Emplacement *p, Emplacement *q, Emplacement *r, Emplacement *s, bool& isIllegal) {
	return seCroisent(p->getX(), p->getY(), q->getX(), q->getY(), r->getX(), r->getY(), s->getX(), s->getY(),isIllegal);
}

bool seCroisent(const Aretes &aretes1, const Aretes &aretes2, bool& isIllegal) {
	return seCroisent(aretes1.getNoeud1()->getEmplacement(), aretes1.getNoeud2()->getEmplacement(), aretes2.getNoeud1()->getEmplacement(), aretes2.getNoeud2()->getEmplacement(),isIllegal);
}

//Pas sur du nom
double calculNormalisation(Emplacement *p1, Emplacement *p2)
{
	double y1 = p1->getY(), y2 = p2->getY();
	double x1 = p1->getX(), x2 = p2->getX();
	return (y2 - y1) / (x2 - x1);
}

bool sontAlignes(Emplacement* p1, Emplacement* p2, Emplacement* p3)
{
	return calculNormalisation(p1, p3) == calculNormalisation(p1, p2)
		&& calculNormalisation(p2, p3) == calculNormalisation(p1, p2);
	//Seconde ligne n�cessaire ?
}

// Renvoie vrai si c est sur le segment ab
bool surSegment(int ax, int ay, int bx, int by, int cx, int cy) {
	if (!collinear(ax,ay,bx,by,cx,cy))
		return false;
	if (ax != bx)
		return ((ax <= cx)&&(cx <= bx))||((ax >= cx)&&(cx >= bx));
	else
		return ((ay <= cy)&&(cy <= by)||((ay >= cy)&&(cy >= by)));
}

bool surSegment(Emplacement* s, Emplacement* t, Emplacement* c)
{
	return between(s->getX(), s->getY(), t->getX(), t->getY(), c->getX(), c->getY());
}

bool surSegment(const Aretes& lien, const Noeud& noeud)
{
	return surSegment(lien.getNoeud1()->getEmplacement(),
		lien.getNoeud2()->getEmplacement(), noeud.getEmplacement());
}

//renvoie vrai si les segments [p,q] et [r,s] se croisent
bool oldSeCroisent(int px, int py, int qx, int qy, int rx, int ry, int sx, int sy) {

	//[P;Q] est a l'ouest de [R;S]
	if (max(px, qx) < min(rx, sx)) {
		return false;
	}
	//[P;Q] est a l'est de [R;S]
	if (min(px, qx) > max(rx, sx)) {
		return false;
	}
	//[P;Q] est au sud de [R;S]
	if (max(py, qy) < min(ry, sy)) {
		return false;
	}
	//[P;Q] est au sud de [R;S]
	if (min(py, qy) > max(ry, sy)) {
		return false;
	}

	//R est a gauche, droite ou aligné a [P;Q]
	int ag1 = aGaucheInt(px, py, qx, qy, rx, ry);
	//S est a gauche, droite ou aligné a [P;Q]
	int ag2 = aGaucheInt(px, py, qx, qy, sx, sy);
	//P est a gauche, droite ou aligné a [R;S]
	int ag3 = aGaucheInt(rx, ry, sx, sy, px, py);
	//Q est a gauche, droite ou aligné a [R;S]
	int ag4 = aGaucheInt(rx, ry, sx, sy, qx, qy);

	//R et S sont du meme cote par rapport a PQ
	//OU P et Q sont du meme cote par rapport a RS
	if (ag1 * ag2 == 1 || ag3 * ag4 == 1) {
		return false;
	}
	//Il restetrois cas, SOIT
	//R et S sont de cotes opposes par rapport a PQ ET P et Q sont de cotes opposes par rapport a RS
	//SOIT
	//les quatre points sont alignes
	//SOIT
	//il ne reste plus que les cas ou un produit est à 0 et l'autre à -1
	//trois points sont alignes
	//deux points sont de cotes opposes par rapport a un segment
	//les segments se croisent forcement
	return true;
}

bool oldSeCroisent(Emplacement *p, Emplacement *q, Emplacement *r, Emplacement *s)
{
	return oldSeCroisent(p->getX(), p->getY(), q->getX(), q->getY(), r->getX(), r->getY(), s->getX(), s->getY());
}

bool oldSeCroisent(const Aretes &aretes1, const Aretes &aretes2) {
	return oldSeCroisent(aretes1.getNoeud1()->getEmplacement(), aretes1.getNoeud2()->getEmplacement(), aretes2.getNoeud1()->getEmplacement(), aretes2.getNoeud2()->getEmplacement());
}

double area2Reel(double ax, double ay, double bx, double by, double cx, double cy) {
	return (bx-ax)*(cy-ay)-(cx-ax)*(by-ay);
}

bool leftReel(double ax, double ay, double bx, double by, double cx, double cy) {
	return area2Reel(ax,ay,bx,by,cx,cy) > 0;
}

bool collinearReel(double ax,double ay,double bx, double by, double cx, double cy) {
	return area2Reel(ax,ay,bx,by,cx,cy) == 0;
}

bool intersectPropReel(double ax, double ay, double bx, double by, double cx, double cy,double dx,double dy) {
	if (collinearReel(ax,ay,bx,by,cx,cy)||collinearReel(ax,ay,bx,by,dx,dy)||collinearReel(cx,cy,dx,dy,ax,ay)||collinearReel(cx,cy,dx,dy,bx,by))
		return false;
	return xorBool(leftReel(ax,ay,bx,by,cx,cy),leftReel(ax,ay,bx,by,dx,dy))&&xorBool(leftReel(cx,cy,dx,dy,ax,ay),leftReel(cx,cy,dx,dy,bx,by));
}

bool betweenReel(double ax,double ay,double bx, double by, double cx, double cy) {
	if (!collinearReel(ax,ay,bx,by,cx,cy))
		return false;
	if (ax != bx)
		return ((ax <= cx)&&(cx <= bx))||((ax >= cx)&&(cx >= bx));
	else
		return ((ay <= cy)&&(cy <= by)||((ay >= cy)&&(cy >= by)));
}

bool seCroisentReel(double ax, double ay,double bx,double by,double cx,double cy,double dx,double dy, bool& isIllegal) {
	if (intersectPropReel(ax,ay,bx,by,cx,cy,dx,dy))
		return true;
	else if (betweenReel(ax,ay,bx,by,cx,cy)||betweenReel(ax,ay,bx,by,dx,dy)||betweenReel(cx,cy,dx,dy,ax,ay)||betweenReel(cx,cy,dx,dy,bx,by)) {
		isIllegal = true;
		return true;
	}
	return false;
}

bool seCroisentReel(const Aretes &aretes1, const Aretes &aretes2, bool& isIllegal) {
	return seCroisentReel(aretes1.getNoeud1()->_xreel,aretes1.getNoeud1()->_yreel, aretes1.getNoeud2()->_xreel,aretes1.getNoeud2()->_yreel, aretes2.getNoeud1()->_xreel,aretes2.getNoeud1()->_yreel, aretes2.getNoeud2()->_xreel,aretes2.getNoeud2()->_yreel,isIllegal);
}

// Renvoie vrai si c est sur le segment ab
bool surSegmentReel(double ax, double ay, double bx, double by, double cx, double cy) {
	if (!collinearReel(ax,ay,bx,by,cx,cy))
		return false;
	if (ax != bx)
		return ((ax <= cx)&&(cx <= bx))||((ax >= cx)&&(cx >= bx));
	else
		return ((ay <= cy)&&(cy <= by)||((ay >= cy)&&(cy >= by)));
}

bool surSegmentReel(const Aretes& lien, const Noeud& noeud) {
	return surSegmentReel(lien.getNoeud1()->_xreel,lien.getNoeud1()->_yreel,lien.getNoeud2()->_xreel,lien.getNoeud2()->_yreel, noeud._xreel,noeud._yreel);
}

// compute the intersection point of two edges
bool seCroisentForce(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy, double& ix, double& iy) {
    double denominator = (bx - ax)*(dy - cy) - (by - ay)*(dx - cx);
    if (denominator == 0) {
        return false;
    }
    double t = ((cy - ay)*(dx - cx) - (cx - ax)*(dy - cy)) / denominator;
    double u = ((by - ay)*(dx - cx) - (bx - ax)*(dy - cy)) / denominator;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        ix = ax + t*(bx - ax);
        iy = ay + t*(by - ay);
        return true;
    } else {
        return false;
    }
}