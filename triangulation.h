#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include "emplacement.hpp"
#include "carte.h"
#include <vector>

using namespace std;

void triangulation(vector<Emplacement*>&T, Carte &C);
void delaunay(Carte &C);


#endif
