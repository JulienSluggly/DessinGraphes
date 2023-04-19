#ifndef INTERSECTION_HPP
#define INTERSECTION_HPP

#include "emplacement.hpp"
#include "aretes.hpp"

//renvoie vrai si les segments [p,q] et [r,s] se croisent
bool seCroisent(int px, int py, int qx, int qy, int rx, int ry, int sx, int sy, bool& isIllegal);

bool seCroisent(Emplacement* p, Emplacement* q, Emplacement* r, Emplacement* s, bool& isIllegal);

bool seCroisent(const Aretes &aretes1, const Aretes &aretes2, bool& isIllegal);

//Pas sur du nom
double calculNormalisation(const Emplacement &p1, const Emplacement &p2);

bool sontAlignes(Emplacement* p1, Emplacement* p2, Emplacement* p3);

bool oldSeCroisent(int px, int py, int qx, int qy, int rx, int ry, int sx, int sy);

bool oldSeCroisent(Emplacement *p, Emplacement *q, Emplacement *r, Emplacement *s);

bool oldSeCroisent(const Aretes &aretes1, const Aretes &aretes2);

bool surSegment(int sx, int sy, int tx, int ty, int cx, int cy);

bool surSegment(Emplacement* s, Emplacement* t, Emplacement* c);

bool surSegment(const Aretes& lien, const Noeud& noeud);

bool seCroisentReel(double px, double py, double qx, double qy, double rx, double ry, double sx, double sy, bool& isIllegal);

bool seCroisentReel(const Aretes &aretes1, const Aretes &aretes2, bool& isIllegal);

bool surSegmentReel(double sx, double sy, double tx, double ty, double cx, double cy);

bool surSegmentReel(const Aretes& lien, const Noeud& noeud);

bool seCroisentForce(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy, double& ix, double& iy);

#endif