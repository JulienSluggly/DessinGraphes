#ifndef EDGEMAP_H
#define EDGEMAP_H
#include <map>
#include <set>
#include <ogdf/basic/EdgeArray.h>
using ogdf::edge;
extern std::map<edge, double> mapEdgeLength;
extern std::map<double, std::set<edge>> mapLengthEdgeSet;

// Map qui associe un edge a sa longueur
std::map<edge, double> mapEdgeLength;
// Map qui associe une longueur a un set de edge
std::map<double, std::set<edge>> mapLengthEdgeSet;
#endif