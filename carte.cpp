#include <vector>
using namespace std;

#include "carte.h"


// Destructeur
Carte::~Carte() {
	for (int i=0; i<d_tabDemiCote.size(); i++)
		delete d_tabDemiCote[i];

}

// Ajoute dans la carte un nouveau c�t� compos� de deux demi-c�t�s dc1 et dc2 dont
// precedent1 et precedent2 seront les demi-c�t�s pr�c�dents respectifs.
// precedent1 et precedent2 doivent �tre effectivement pr�sents dans la carte.
// Renvoie l'adresse de dc1.
DemiCote* Carte::ajouteCote(DemiCote* precedent1, DemiCote* precedent2)
{
	DemiCote* dc1 = ajouteDemiCote(precedent1);
	ajouteDemiCote(precedent2,dc1);
	return dc1;
}

// Ajoute dans la carte un nouveau c�t� compos� de deux demi-c�t�s dc1 et dc2.
// precedent1 sera le demi-c�t� pr�c�dent de dc1 et dc2 sera issu d�un
// nouveau sommet (� cr�er) dont les coordonn�es sont celles du point p2.
// precedent1 doit �tre effectivement pr�sent dans la carte.
// Renvoie l'adresse de dc1.
DemiCote* Carte::ajouteCote(DemiCote* precedent1, Emplacement* p2)
{
	DemiCote* dc1 = ajouteDemiCote(precedent1);
	ajouteDemiCote(p2,dc1);
	return dc1;
}

// Ajoute dans la carte un nouveau c�t� compos� de deux demi-c�t�s dc1 et dc2.
// precedent2 sera le demi-c�t� pr�c�dent de dc2 et dc1 sera issu d�un
// nouveau sommet (� cr�er) dont les coordonn�es sont celles du point p1.
// precedent2 doit �tre effectivement pr�sent dans la carte.
// Renvoie l'adresse de dc1.
DemiCote* Carte::ajouteCote(Emplacement* p1, DemiCote* precedent2)
{
	DemiCote* dc1 = ajouteDemiCote(p1);
	ajouteDemiCote(precedent2,dc1);
	return dc1;
}

// Ajoute dans la carte un nouveau c�t� compos� de deux demi-c�t�s dc1 et dc2.
// dc1 et dc2 seront issus de deux nouveaux sommets (� cr�er) dont les coordonn�es
// sont celles des points p1 et p2 respectivement.
// Renvoie l'adresse de dc1.
DemiCote* Carte::ajouteCote(Emplacement* p1, Emplacement* p2)
{
	DemiCote* dc1 = ajouteDemiCote(p1);
	ajouteDemiCote(p2,dc1);
	return dc1;
}

// Ajoute un nouveau demi-c�t� dans la carte dont precedent sera le demi-c�t� pr�c�dent et
// oppose sera le demi-c�t� oppos�. On suppose que le demi-c�t� precedent est d�j� pr�sent dans la carte.
// Le demi-c�t� oppose est soit d�j� pr�sent dans la carte soit nul (valeur par d�faut).
// Renvoie l�adresse du nouveau demi-c�t�.
DemiCote* Carte::ajouteDemiCote(DemiCote* precedent, DemiCote* oppose)
{
	DemiCote* dc = new DemiCote(precedent->d_suivant, precedent, oppose, precedent->d_sommet, d_tabDemiCote.size());
	d_tabDemiCote.push_back(dc);
	precedent->d_suivant = dc;
	dc->d_suivant->d_precedent = dc;
	if (oppose != nullptr)
		oppose->d_oppose = dc;
	return dc;
}

// Ajoute un nouveau demi-c�t� dans la carte qui sera issu d�un nouveau sommet (� cr�er)
// dont les coordonn�es sont celles du point p.	oppose sera le demi-c�t� oppos� du nouveau demi-c�t�.
// oppose est soit d�j� pr�sent dans la carte soit nul (valeur par d�faut).
// Renvoie l�adresse du nouveau demi-c�t�.
DemiCote* Carte::ajouteDemiCote(Emplacement* s, DemiCote* oppose) {
	s->d_demiCote = nullptr;
	s->d_indice = d_tabSommet.size();
    d_tabSommet.push_back(s);
	DemiCote* dc = new DemiCote(nullptr, nullptr, oppose, s, d_tabDemiCote.size());
	d_tabDemiCote.push_back(dc);
	s->d_demiCote=dc;
	dc->d_suivant = dc->d_precedent = dc;
	if (oppose != nullptr)
		oppose->d_oppose = dc;
	return dc;
}

// Effectue dans la carte le flip du demi-c�t� d.
// On suppose que d est bien un demi-c�t� de la carte et que le flip est r�alisable.
void Carte::flipDemiCote(DemiCote *d)
{
	DemiCote *nouveauPrecedent = d->d_suivant->d_oppose;
	d->d_suivant->d_precedent = d->d_precedent;
	d->d_precedent->d_suivant = d->d_suivant;
	d->d_suivant = nouveauPrecedent->d_suivant;
	d->d_precedent = nouveauPrecedent;
	nouveauPrecedent->d_suivant = d;
	d->d_suivant->d_precedent = d;
	d->d_sommet = nouveauPrecedent->d_sommet;
	d->d_sommet->d_demiCote = d;
}

// Effectue dans la carte le flip du c�t� form� par le demi-c�t� d et son demi-c�t� oppos�.
// On suppose que d est bien un demi-c�t� de la carte et que le flip est r�alisable.
// � la sortie de la m�thode, le demi-c�t� d et son oppos� sont les deux demi-c�t�s du c�t� cr��.
void Carte::flip(DemiCote *d)
{
	flipDemiCote(d);
	flipDemiCote(d->d_oppose);
}


