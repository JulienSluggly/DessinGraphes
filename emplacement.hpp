#ifndef EMPLACEMENT_HPP
#define EMPLACEMENT_HPP

#include <math.h>
#include <utility>
#include <vector>

class Noeud;
class DemiCote;

class Emplacement {
	friend class Carte;
	friend class DemiCote;
public:
	Noeud* _noeud = nullptr;
	int _id;
	int _x, _y;

	// Id des emplacements dans le tableau _emplacementsPossibles
	std::vector<int>* voisinsDelaunay = nullptr;

	// Id des cellules par rapport a la grille dans le graphe, max 4 cellules par emplacement
	int idCellule = -1;

	std::vector<int>* idCelluleVec = nullptr;

	Emplacement(int x, int y, int id) {
		_x = x;
		_y = y;
		_id = id;
	}

	~Emplacement() {
		delete voisinsDelaunay;
		delete idCelluleVec;
	}

	int getId() const { return _id; }
	bool estDisponible() { return _noeud == nullptr; }

	// NE PAS APPELER CES FONCTIONS
	void setNoeud(Noeud* noeud){
		_noeud = noeud;
	}
	void removeNoeud(){
		_noeud = nullptr;
	}

	int getX() const { return _x; }
	int getY() const { return _y; }
	int x() const {return _x;}
	int y() const {return _y;}

	int distance(Emplacement* emp) const {
		int xValue = _x - emp->getX();
		int yValue = _y - emp->getY();
		return sqrt(xValue * xValue + yValue * yValue);
	}
	int distance(std::pair<int,int>& p) const {
		int xValue = _x - p.first;
		int yValue = _y - p.second;
		return sqrt(xValue * xValue + yValue * yValue);
	}

	void setX(int x) { _x = x; }
	void setY(int y) { _y = y; }

	// renvoie 1, -1 ou 0 selon que le point auquel la méthode est appliquée est
	// à gauche de, à droite de, ou sur la droite (ab) orientée de a vers b.
	int aGauche(Emplacement *a, Emplacement *b) const {
		long long det = (b->x()-a->x())*(y()-a->y()) - (x()-a->x())*(b->y()-a->y());
		return det > 0 ? 1 : (det < 0 ? -1 : 0);
	}


	// renvoie 1, -1 ou 0 selon que le point auquel la méthode est appliquée est
	// à lintérieur du, à lextérieur du, ou sur le cercle qui passe par les points a, b, c.
	// Les points a, b, c doivent être donnés dans le sens trigonométrique.
	int dansCercle(Emplacement *a, Emplacement *b, Emplacement *c) const {
		long long apx = _x-a->_x,
				apy = _y-a->_y,
				bpx = _x-b->_x,
				bpy = _y-b->_y,
				cpx = _x-c->_x,
				cpy = _y-c->_y;

		long long det = (apx*apx+apy*apy)*(bpx*cpy-bpy*cpx)
			-(bpx*bpx+bpy*bpy)*(apx*cpy-apy*cpx)
			+(cpx*cpx+cpy*cpy)*(apx*bpy-apy*bpx);

		return det > 0 ? 1 : (det < 0 ? -1 : 0);
	}


	DemiCote* d_demiCote=nullptr; // l'un des demi-c�t�s issus du sommet
	int d_indice=-1;       // la position du sommet dans le tableau d_tabSommet

	// Renvoie un demi-c�t� issu du sommet
    DemiCote* demiCote() const {return d_demiCote;}

    // Renvoie l'indice du sommet
    int indice() const {return d_indice;}

};

#endif