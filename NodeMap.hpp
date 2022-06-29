#ifndef NODEMAP_H
#define NODEMAP_H
#include <vector>
#include "NodeBend.hpp"
extern std::vector<std::vector<bool>> mapPosNode;
extern std::vector<NodeBend> vectorNodeBends;

// Map qui indique si une node ou un bend se trouve a ses coordonnées.
// mapPosNode[Y][X]
std::vector<std::vector<bool>> mapPosNode;
// Vector qui contient la liste de tout les noeuds et bends d'un graphe
std::vector<NodeBend> vectorNodeBends;
#endif