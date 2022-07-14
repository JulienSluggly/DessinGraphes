#include <ogdf/basic/GridLayout.h>

#include <ogdf/planarlayout/PlanarStraightLayout.h>
#include <ogdf/planarity/EmbedderMinDepth.h>
#include <ogdf/planarity/SimpleEmbedder.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <string>


#include "jsonIO.hpp"
#include "dispOpenGL.hpp"
#include "EdgeMap.hpp"
#include "NodeMap.hpp"
#include "graphFunctions.hpp"
#include "noDispRun.hpp"

using ogdf::Graph;
using ogdf::GridLayout;
using std::cout, std::endl;

int main() {

	Graph G;
	GridLayout GL{ G };

	int gridWidth, gridHeight, maxBends;

	// ----- LECTURE D'UN FICHIER JSON DANS UN Graph -----
	string file = "F:/The World/Cours/M1S2/Graphe/GitHub/ProjetGrapheM1-BinaryHeap/ProjetGrapheM1/manuel/man21-4.json";
	std::cout << "File: " << file << std::endl;
	readFromJson(file, G, GL, gridWidth, gridHeight, maxBends);
	writeToJson("output.json", G, GL, gridWidth, gridHeight, maxBends);

	int maxX = 0, maxY = 0, minX = 100000, minY = 100000;

	bool planarize = false;
	if (planarize) {
		std::cout << "Planarizing..." << std::endl;
		PlanarStraightLayout PL;
		PL.separation(-19);
		PL.callGrid(G, GL);
		node n = G.firstNode();
		while (n != nullptr) {
			if (GL.x(n) > maxX) maxX = GL.x(n);
			if (GL.x(n) < minX) minX = GL.x(n);
			if (GL.y(n) > maxY) maxY = GL.x(n);
			if (GL.y(n) < minY) minY = GL.x(n);
			n = n->succ();
		}
	}

	std::cout << "Embedding..." << std::endl;
	embedderCarte(G, GL);
	std::cout << "Embeded: " << G.representsCombEmbedding() << std::endl;

	//GraphIO::write(GL, "output-ERDiagram2.svg", GraphIO::drawSVG);
	std::cout << "Connexe: " << isConnected(G) << std::endl;
	std::cout << "Planaire: " << isPlanar(G) << std::endl;

	// Affichage des maps

	/*std::map<edge, double>::iterator it;
	for (it = mapEdgeLength.begin(); it != mapEdgeLength.end(); it++) {
		std::cout << "MapEdgeLength: " << it->second << std::endl;
	}

	std::map<double, std::set<edge>>::iterator it2;
	for (it2 = mapLengthEdgeSet.begin(); it2 != mapLengthEdgeSet.end(); it2++) {
		std::cout << "mapLengthEdgeSet: " << it2->first << std::endl;
	}*/

	// Ajout des node dans le vector
	node n = G.firstNode();
	while (n != nullptr) {
		NodeBend* tmpNodeBend = new NodeBend(n, GL);
		vectorNodeBends.push_back(tmpNodeBend);
		n = n->succ();
	}

	// Ajout des bend dans le vector
	edge e = G.firstEdge();
	while (e != nullptr) {
		IPolyline& bends = GL.bends(e);
		int k = 0;
		NodeBend* p1 = getNodeBendFromNode(e->source());
		NodeBend* p2 = getNodeBendFromNode(e->target());
		NodeBend* precedent = p1;
		for (ListIterator<IPoint> i = bends.begin(); i.valid(); k++) {
			p1 = getNodeBendFromNode(e->source());
			p2 = getNodeBendFromNode(e->target());
			NodeBend* tmpNodeBend = new NodeBend((*i), e, k);
			tmpNodeBend->parent1 = p1;
			tmpNodeBend->parent2 = p2;
			// Marche uniquement pour les bends supplémentaires qui s'initialisent sur le node parent
			if ((tmpNodeBend->getX() == p1->getX())&&(tmpNodeBend->getY() == p1->getY())) {
				tmpNodeBend->isStacked = true;
				p1->isStacked = true;
				if ((k == 0) || (k == bends.size()-1)) {
					p1->addAdjNodeBend(tmpNodeBend);
				}
			}
			else if ((tmpNodeBend->getX() == p2->getX()) && (tmpNodeBend->getY() == p2->getY())) {
				tmpNodeBend->isStacked = true;
				p2->isStacked = true;
				if ((k == 0) || (k == bends.size() - 1)) {
					p2->addAdjNodeBend(tmpNodeBend);
				}
			}
			tmpNodeBend->precedent = precedent;
			if (!precedent->isNode) {
				precedent->suivant = tmpNodeBend;
				if (precedent->suivant == nullptr)
					std::cout << "NULL" << std::endl;
			}
			i++;
			if (!i.valid()) {
				tmpNodeBend->suivant = p2;
			}
			precedent = tmpNodeBend;
			vectorNodeBends.push_back(tmpNodeBend);
		}
		e = e->succ();
	}

	// Assign du num global et du tableau de position
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		vectorNodeBends[i]->assignGlobalNum(i);
		posVectorNodeBend[*vectorNodeBends[i]->a_x][*vectorNodeBends[i]->a_y].insert(vectorNodeBends[i]);
	}

	// On initialise le tableau de segment de chaque nodebend
	ConstCombinatorialEmbedding CCE{G};
	int* srcX, *srcY, *trgX, *trgY;
	std::set<edge> edgeSet;
	//std::cout << "Remplissage du tableau: --------------------------------" << std::endl;
	NodeBend* source = nullptr;
	NodeBend* target = nullptr;
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		edgeSet.clear();
		edgeSet = getEdgesFromAdjFacesFromNodeBend(vectorNodeBends[i],CCE);
		for (auto it = edgeSet.begin();it != edgeSet.end();it++) {
			srcX = &GL.x((*it)->source());
			srcY = &GL.y((*it)->source());
			source = getNodeBendFromNode((*it)->source());
			IPolyline& p = GL.bends((*it));
			// Si l'edge contient des bends
			if (p.size() > 0) {
				auto it2 = p.begin();
				while (it2.valid()) {
					trgX = &(*it2).m_x;
					trgY = &(*it2).m_y;
					target = getNodeBendFromBend(&(*it2));
					Segment tmpSeg(srcX, srcY, trgX, trgY);
					tmpSeg.setSource(source);
					tmpSeg.setTarget(target);
					vectorNodeBends[i]->addAdjFaceSegment(tmpSeg);
					srcX = trgX;
					srcY = trgY;
					it2++;
					source = target;
				}
			}
			target = getNodeBendFromNode((*it)->target());
			trgX = &GL.x((*it)->target());
			trgY = &GL.y((*it)->target());
			Segment tmpSeg(srcX, srcY, trgX, trgY);
			tmpSeg.setSource(source);
			tmpSeg.setTarget(target);
			vectorNodeBends[i]->addAdjFaceSegment(tmpSeg);
		}
		//std::cout << "Numero: " << i << " Taille tableau: " << vectorNodeBends[i]->adjFaceSegment.size() << std::endl;
		if (vectorNodeBends[i]->isNode) {
			ListPure<adjEntry> nodeAdjEntries;
			vectorNodeBends[i]->getNode()->allAdjEntries(nodeAdjEntries);
			for (auto it = nodeAdjEntries.begin(); it.valid(); it++) {
				std::vector<Segment> adjSegmentVector = getSegmentFromAdjFacesFromAdjEntry((*it), CCE, GL);
				for (int j = 0; j < adjSegmentVector.size(); j++) {
					vectorNodeBends[i]->insertSegmentToAdjEntry((*it), adjSegmentVector[j]);
				}
			}
		}
	}
	//std::cout << "FIN Remplissage du tableau: --------------------------------" << std::endl;
	
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		std::cout << "Numero: " << i << " GlobalNum: " << vectorNodeBends[i]->globalNum << " isNode: " << vectorNodeBends[i]->isNode;
		if (!vectorNodeBends[i]->isNode) {
			std::cout << " precedent: " << vectorNodeBends[i]->precedent->globalNum << " suivant: " << vectorNodeBends[i]->suivant->globalNum;
		}
		if (!vectorNodeBends[i]->isNode) {
			std::cout << " k: " << vectorNodeBends[i]->numero;
		}
		std::cout << std::endl;
	}

	// On melange le vecteur de nodebend pour affecter de l'aléatoire sur certains algo
//	auto rd = std::random_device{};
//	auto rng = std::default_random_engine{ rd() };
//	std::shuffle(std::begin(vectorNodeBends), std::end(vectorNodeBends), rng);

	bool useOpenGL = false;

	// OpenGL
	srand(static_cast<unsigned int>(time(NULL)));
	if (useOpenGL) {
		dispOpenGL(G, GL, gridWidth, gridHeight, maxX, maxY, maxBends);
	}
	else {
		runAlgo(10, G, GL, gridWidth, gridHeight, maxX, maxY, maxBends);
	}
	return 0;
}