#include <ogdf/basic/GridLayout.h>

#include <ogdf/planarlayout/PlanarStraightLayout.h>
#include <ogdf/planarity/EmbedderMaxFace.h>
#include <ogdf/planarity/EmbedderMaxFaceLayers.h>
#include <ogdf/planarity/EmbedderMinDepth.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFace.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFaceLayers.h>
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
	//string file = "F:/The World/Cours/M1S2/Graphe/GitHub/ProjetGrapheM1-BinaryHeap/ProjetGrapheM1/bestResult.json";

	bool useOpenGL = true;

	std::cout << "File: " << file << std::endl;
	readFromJson(file, G, GL, gridWidth, gridHeight, maxBends);
	writeToJson("output.json", G, GL, gridWidth, gridHeight, maxBends);

	int maxX = gridWidth, maxY = gridHeight, minX = 0, minY = 0;

	bool planarize = true;
	if (planarize) {
		int specificEmbedding = 1;
		std::cout << "Planarizing..." << std::endl;
		PlanarStraightLayout PL;
		PL.separation(-19);

		if (specificEmbedding > 0) {
			EmbedderModule* embm = nullptr;
			switch (specificEmbedding) {
				case 1:
					std::cout << "Specific Embedder used: MinDepth" << std::endl;
					embm = new EmbedderMinDepth();
					break;
				case 2:
					std::cout << "Specific Embedder used: MaxFace" << std::endl;
					embm = new EmbedderMaxFace();
					break;
				case 3:
					std::cout << "Specific Embedder used: MaxFaceLayers" << std::endl;
					embm = new EmbedderMaxFaceLayers();
					break;
				case 4:
					std::cout << "Specific Embedder used: MinDepthMaxFace" << std::endl;
					embm = new EmbedderMinDepthMaxFace();
					break;
				case 5:
					std::cout << "Specific Embedder used: MinDepthMaxFaceLayers" << std::endl;
					embm = new EmbedderMinDepthMaxFaceLayers();
					break;
				default:
					break;
			}
			PL.setEmbedder(embm);
		}
		PL.callGrid(G, GL);
		node n = G.firstNode();
		while (n != nullptr) {
			if (GL.x(n) > maxX) maxX = GL.x(n);
			if (GL.x(n) < minX) minX = GL.x(n);
			if (GL.y(n) > maxY) maxY = GL.y(n);
			if (GL.y(n) < minY) minY = GL.y(n);
			n = n->succ();
		}
		std::cout << "minX: " << minX << " maxX: " << maxX << " minY: " << minY << " maxY: " << maxY << std::endl;
	}

	// Remplissage des tableaux globaux
	for (int i = 0; i <= maxX+10; i++) {
		std::vector<std::list<NodeBend*>> tmpVector;
		for (int j = 0; j <= maxY+10; j++) {
			std::list<NodeBend*>tmpVector2;
			tmpVector.push_back(tmpVector2);
		}
		posVectorNodeBend.push_back(tmpVector);
	}

	std::cout << "Embedding..." << std::endl;
	embedderCarte(G, GL);
	std::cout << "Embeded: " << G.representsCombEmbedding() << std::endl;

	ConstCombinatorialEmbedding CCE{ G };
	vectorFaceSegment.reserve(CCE.maxFaceIndex()+1);
	for (int i = 0; i < vectorFaceSegment.capacity(); i++) {
		std::vector<Segment*> tmpVecSegment;
		vectorFaceSegment.push_back(tmpVecSegment);
	}
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
		NodeBend* tmpNodeBend = new NodeBend(n, GL, CCE);
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
			NodeBend* tmpNodeBend = new NodeBend((*i), e, k, CCE);
			tmpNodeBend->parent1 = p1;
			tmpNodeBend->parent2 = p2;
			if (k == 0) {
				p1->addAdjNodeBend(tmpNodeBend, GL);
			}
			// Marche uniquement pour les bends supplémentaires qui s'initialisent sur le node parent
			if ((tmpNodeBend->getX() == p1->getX())&&(tmpNodeBend->getY() == p1->getY())) {
				tmpNodeBend->isStacked = true;
				p1->isStacked = true;
			}
			else if ((tmpNodeBend->getX() == p2->getX()) && (tmpNodeBend->getY() == p2->getY())) {
				tmpNodeBend->isStacked = true;
				p2->isStacked = true;
			}
			tmpNodeBend->precedent = precedent;
			if (!precedent->isNode) {
				precedent->suivant = tmpNodeBend;
			}
			i++;
			if (!i.valid()) {
				tmpNodeBend->suivant = p2;
				p2->addAdjNodeBend(tmpNodeBend, GL);
			}
			precedent = tmpNodeBend;
			vectorNodeBends.push_back(tmpNodeBend);
		}
		e = e->succ();
	}
	// Assign du num global et du tableau de position
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		vectorNodeBends[i]->assignGlobalNum(i);
		if (vectorNodeBends[i]->isNode) {
			posVectorNodeBend[*vectorNodeBends[i]->a_x][*vectorNodeBends[i]->a_y].push_front(vectorNodeBends[i]);
		}
		else {
			posVectorNodeBend[*vectorNodeBends[i]->a_x][*vectorNodeBends[i]->a_y].push_back(vectorNodeBends[i]);
		}
	}

	// On initialise le tableau global de segment et la map de face a segment
	face f = CCE.firstFace();
	std::vector<edge> vectorEdges;
	vectorEdges.reserve(f->size());
	int numSegment = 0;
	while (f != nullptr) {
		int numeroFace = f->index();
		// On recupere la liste des edges
		adjEntry firstAdj = f->firstAdj();
		adjEntry nextAdj = firstAdj;
		if (firstAdj != nullptr) {
			do {
				vectorEdges.push_back(nextAdj->theEdge());
				nextAdj = f->nextFaceEdge(nextAdj);
			} while ((nextAdj != firstAdj) && (nextAdj != nullptr));
		}
		// On parcour tout les edges et on les transforme en segments ou on regarde s'il existe deja.
		int* srcX, * srcY, * trgX, * trgY;
		NodeBend* source = nullptr;
		NodeBend* target = nullptr;
		Segment* s = nullptr;
		for (int i = 0;i<vectorEdges.size();i++) {
			srcX = &GL.x(vectorEdges[i]->source());
			srcY = &GL.y(vectorEdges[i]->source());
			source = getNodeBendFromNode(vectorEdges[i]->source());
			IPolyline& p = GL.bends(vectorEdges[i]);
			// Si l'edge contient des bends
			if (p.size() > 0) {
				auto it2 = p.begin();
				while (it2.valid()) {
					trgX = &(*it2).m_x;
					trgY = &(*it2).m_y;
					target = getNodeBendFromBend(&(*it2));
					s = getSegmentFromNodeBends(source, target);
					// On créé le segment s'il n'existe pas
					if (s == nullptr) {
						s = new Segment(srcX, srcY, trgX, trgY);
						s->setSource(source);
						s->setTarget(target);
						s->assignGlobalNum(numSegment);
						numSegment++;
						vectorSegments.push_back(s);
						
					}
					vectorFaceSegment[numeroFace].push_back(s);
					srcX = trgX;
					srcY = trgY;
					it2++;
					source = target;
				}
			}
			target = getNodeBendFromNode(vectorEdges[i]->target());
			trgX = &GL.x(vectorEdges[i]->target());
			trgY = &GL.y(vectorEdges[i]->target());
			s = getSegmentFromNodeBends(source, target);
			if (s == nullptr) {
				s = new Segment(srcX, srcY, trgX, trgY);
				s->setSource(source);
				s->setTarget(target);
				s->assignGlobalNum(numSegment);
				numSegment++;
				vectorSegments.push_back(s);
			}
			vectorFaceSegment[numeroFace].push_back(s);
		}
		f = f->succ();
	}

	// On verifie les données de stacking pour chaque nodebend
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		if (vectorNodeBends[i]->isNode) {
			vectorNodeBends[i]->initStackCheck();
		}
		else {
			vectorNodeBends[i]->recalculateIsStacked();
		}
	}
	
	// Affichage des numeros et suivants pour debug
	
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

	// OpenGL
	srand(static_cast<unsigned int>(time(NULL)));
	if (useOpenGL) {
		dispOpenGL(G, GL, gridWidth, gridHeight, maxX, maxY, maxBends);
	}
	else {
		runAlgo(10, G, GL, gridWidth, gridHeight, maxX, maxY, maxBends, file);
	}
	return 0;
}