#include "carte.h"
#include "triangulation.h"

#include <vector>
#include <algorithm>
#include "utilitaire.hpp"

using namespace std;

void triangulation(vector<Emplacement*>&T, Carte &C) {
    sort(T.begin(),T.end(),comparePtrEmplacementTri);

	DemiCote* b = C.ajouteCote(T[1],T[0]);

	for (int i = 2; i < T.size(); i++) {
		DemiCote* b1 = b->suivant()->oppose();
		DemiCote* bp = C.ajouteCote(T[i],b);
		while (T[i]->aGauche(b1->oppose()->sommet(), b1->sommet()) < 0)
		{
			b1 = C.ajouteCote(b1,bp);
			b1 = b1->suivant()->oppose();
		}
		while (T[i]->aGauche(b->oppose()->sommet(), b->sommet()) < 0)
		{
			b = b->oppose()->precedent();
			bp = C.ajouteCote(bp,b);
		}
		b = bp;
	}
	C.changeDemiCoteParticulier(b);
}

void delaunay(Carte &C)
{
	DemiCote* b = C.demiCoteParticulier();
	do
	{
		b->changeMarque(1);
		b->oppose()->changeMarque(1);
		b = b->oppose()->precedent();
	}
	while (b != C.demiCoteParticulier());

	vector<DemiCote*> P;
	for (int i=0; i < C.nbDemiCote(); i++)
		if (C.demiCote(i)->marque() == 0)
		{
			C.demiCote(i)->changeMarque(1);
			C.demiCote(i)->oppose()->changeMarque(1);
			P.push_back(C.demiCote(i));
		}

	while (P.size() != 0)
	{
		DemiCote* a = P.back();
		P.pop_back();
		a->changeMarque(0);
		a->oppose()->changeMarque(0);
		if (a->precedent()->oppose()->sommet()->dansCercle(a->sommet(), a->oppose()->sommet(), a->suivant()->oppose()->sommet()) > 0)
		{
			DemiCote* dc1 = a->precedent(),
                    * dc2 = dc1->oppose()->precedent(),
                    * dc3 = dc2->oppose()->precedent()->precedent(),
                    * dc4 = dc3->oppose()->precedent();
			for (DemiCote *dc : {dc1,dc2,dc3,dc4})
			{
				if (dc->marque() == 0)
				{
					dc->changeMarque(1);
					dc->oppose()->changeMarque(1);
					P.push_back(dc);
				}
			}
			C.flip(a);
		}
	}

	do
	{
		b->changeMarque(0);
		b->oppose()->changeMarque(0);
		b = b->oppose()->precedent();
	}
	while (b != C.demiCoteParticulier());

}



