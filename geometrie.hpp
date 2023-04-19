#ifndef GEOMETRIE_HPP
#define GEOMETRIE_HPP
#include "aretes.hpp"

//renvoie 1,2,3 ou 4 si lpoint t est 1: en haut � droite, 2: en haut � gauche, 3: en bas � gauche, 4: en bas � droite du point s
//on consid�re s != t
int quadrant(int sx, int sy, int tx, int ty);

//s= source, t=target, c=comp=nouveau noeud � ajouter, renvoie vrai si c est � gauche de (s;t) faux sinon
bool aGauche(int sx, int sy, int tx, int ty, int cx, int cy);

//si c est � gauche de (s;t) renvoie 1, si c est � droite de (s;t) renvoie -1, si c,s et t sont align�s renvoie 0
int aGaucheInt(int sx, int sy, int tx, int ty, int cx, int cy);

//si c est � gauche de (s;t) renvoie 1, si c est � droite de (s;t) renvoie -1, si c,s et t sont align�s renvoie 0
int aGaucheIntReel(double sx, double sy, double tx, double ty, double cx, double cy);

// Renvoie vrai si c est dans le rectangle form� par st
bool dansRectangle(int sx,int sy,int tx,int ty,int cx,int cy);

bool dansRectangle(Emplacement* corner1, Emplacement* corner2, Emplacement* point);

bool dansRectangle(const Noeud &noeud1, const Noeud &noeud2, const Noeud &noeud);

bool dansRectangle(const Aretes &aretes, const Noeud &noeud);

// Renvoie vrai si c est sur le segment st
bool oldSurSegment(int sx, int sy, int tx, int ty, int cx, int cy);

bool oldSurSegment(Emplacement* s, Emplacement* t, Emplacement* c);

bool oldSurSegment(const Aretes& lien, const Noeud& noeud);


#endif