#ifndef OGDFFUNC_HPP
#define OGDFFUNC_HPP

#include <vector>
#include <string>

class GraphAttributes;
class GridLayout;
class Graph;
class Graphe;

void ogdfReadFromMM(Graphe& G, std::istream& inStream);

void createGrapheFromOGDFGraphe(Graphe& G, Graph& ogdfG);

int ogdfNumberOfCrossings(GraphAttributes& ogdfGA);

// Creer un graphe ogdf a partir d'un graphe placé ou non.
void createOGDFGraphFromGraphe(Graphe &G, GridLayout &ogdfGL, Graph &ogdfG);

// Creer un graphe ogdf a partir d'un graphe placé ou non.
void createOGDFGraphFromGraphe(Graphe &G, GraphAttributes &ogdfGA, Graph &ogdfG);

int crossingModule(const  Graph& ogdfG);

int planarizeMaxFace(GridLayout& ogdfGL, Graph& ogdfG);

void ogdfTranslateOgdfGraphToOrigin(Graph& ogdfG, GraphAttributes& ogdfGA);

// ogdfGA doit contenir le placement du graphe ogdf, on fait ensuite une translation puis scaling et on place au plus proche
void ogdfPlacementAuPlusProche(GraphAttributes& ogdfGA, Graph& ogdfG, Graphe& G);

// Planarize le graphe avec odgf et essaie de placer les noeuds au plus proche de ses emplacements possible par rapport au grpahe OGDF
int ogdfPlacementAuPlusProche(Graphe& G);

// Planarize le graphe avec odgf et essaie de placer les noeuds au plus proche de ses emplacements possible par rapport au graphe OGDF
void ogdfPlacementAuPlusProcheStress(Graphe& G);

// Remplace les emplacements du graphe par les emplacements trouvés par la planarizerMaxFace
int ogdfReverse(Graphe &G);

void ogdfRescaleOgdfG(Graph& ogdfG, GraphAttributes& ogdfGA, int scale);

void ogdfReverseAndPlace(Graphe &G, GraphAttributes& ogdfGA, Graph& ogdfG);

void ogdfReverseNonPlanar(Graphe &G);

void ogdfCrossingNumbers(std::vector<std::string> graphFiles);

void ogdfStressMinimization(Graphe& G);

void ogdfFastMultipoleEmbedder(Graphe& G);

void ogdfFastMultipoleMultilevelEmbedder(Graphe& G);

void ogdfFastMultipoleMultilevelEmbedderReel(Graphe& G);

void ogdfFastMultipoleMultilevelEmbedderReelMinute(Graphe& G);

void ogdfFMMMLayout(Graphe& G);

void ogdfGEMLayout(Graphe& G);

void ogdfPivotMDS(Graphe& G);

void ogdfOther(Graphe& G);

void ogdfOtherTest(Graphe& G);

void ogdfGutwenger(Graphe& G);

int ogdfTotalNumberOfBends(GraphAttributes& ogdfGa);

// Fait différents test sur le graphe: connexité, biconnecté, genus, crossing number(SubGraphPlanarize)
void ogdfRun(Graphe &G);

#endif