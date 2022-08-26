#include <ogdf/basic/GridLayout.h>

#include <ogdf/planarlayout/PlanarStraightLayout.h>
#include <ogdf/planarity/EmbedderMaxFace.h>
#include <ogdf/planarity/EmbedderMaxFaceLayers.h>
#include <ogdf/planarity/EmbedderMinDepth.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFace.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFaceLayers.h>
#include <ogdf/planarity/SimpleEmbedder.h>
#include <ogdf/misclayout/BertaultLayout.h>

#include <ogdf/basic/simple_graph_alg.h>
#include <string>

#include "jsonIO.hpp"
#include "dispOpenGL.hpp"
#include "EdgeMap.hpp"
#include "NodeMap.hpp"
#include "graphFunctions.hpp"
#include "noDispRun.hpp"
#include "classUtils.hpp"

using ogdf::Graph;
using ogdf::GridLayout;
using std::cout, std::endl;

int main() {

	Graph G;
	GridLayout GL{ G };

	int gridWidth, gridHeight, maxBends;

	// ----- LECTURE D'UN FICHIER JSON DANS UN Graph -----
	string nom_fichier = "auto21-7_MaxFace2";
	string file = "D:/The World/Cours/M1S2/Graphe/Projet/DessinGraphe/embeddings/auto21-7/" + nom_fichier + ".json";
	//string file = "D:/The World/Cours/M1S2/Graphe/GitHub/ProjetGrapheM1-BinaryHeap/ProjetGrapheM1/bestResult.json";

	bool useOpenGL = true;

	std::cout << "File: " << file << std::endl;
	readFromJson(file, G, GL, gridWidth, gridHeight, maxBends);
	writeToJson("output.json", G, GL, gridWidth, gridHeight, maxBends);

	int maxX = gridWidth, maxY = gridHeight, minX = 0, minY = 0;

	bool planarize = false;
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
	}

	node n1 = G.firstNode();
	while (n1 != nullptr) {
		if (GL.x(n1) > maxX) maxX = GL.x(n1);
		if (GL.x(n1) < minX) minX = GL.x(n1);
		if (GL.y(n1) > maxY) maxY = GL.y(n1);
		if (GL.y(n1) < minY) minY = GL.y(n1);
		n1 = n1->succ();
	}
	std::cout << "minX: " << minX << " maxX: " << maxX << " minY: " << minY << " maxY: " << maxY << std::endl;

	posVectorNodeBend.resize(maxX + 11);
	// Remplissage des tableaux globaux
	for (int i = 0; i <= maxX + 10; i++) {
		posVectorNodeBend[i].resize(maxY + 11);
	}

	std::cout << "Embedding..." << std::endl;
	embedderCarte(G, GL);
	std::cout << "Embeded: " << G.representsCombEmbedding() << std::endl;
	std::cout << "Connexe: " << isConnected(G) << std::endl;
	std::cout << "Planaire: " << isPlanar(G) << std::endl;
	
	// Ajout des node dans le vector
	node n = G.firstNode();
	while (n != nullptr) {
		NodeBend* tmpNodeBend = new NodeBend(n, GL);
		if ((tmpNodeBend->getX() <= gridWidth) && (tmpNodeBend->getY() <= gridHeight)) {
			tmpNodeBend->isInGrille = true;
		}
		else {
			tmpNodeBend->isInGrille = false;
		}
		vectorNodeBends.push_back(tmpNodeBend);
		n = n->succ();
	}

	if (G.representsCombEmbedding() && isConnected(G) && isPlanar(G)) {
		ConstCombinatorialEmbedding CCE{ G };
		vectorFaceSegment.reserve(CCE.maxFaceIndex() + 1);
		for (int i = 0; i < vectorFaceSegment.capacity(); i++) {
			std::vector<Segment*> tmpVecSegment;
			vectorFaceSegment.push_back(tmpVecSegment);
		}

		// Ajout des bend dans le vector
		edge e = G.firstEdge();
		while (e != nullptr) {
			IPolyline& bends = GL.bends(e);
			int k = 0;
			NodeBend* p1 = getNodeBendFromNode(e->source());
			NodeBend* p2 = getNodeBendFromNode(e->target());
			NodeBend* precedent = p1;
			adjEntry adj1 = e->adjSource();
			adjEntry adj2 = e->adjTarget();
			std::pair<int, int> tmpPair(CCE.leftFace(adj1)->index(), CCE.rightFace(adj1)->index());
			mapAdjEntryFaces.insert(std::pair<adjEntry, std::pair<int, int>>(adj1, tmpPair));
			mapAdjEntryFaces.insert(std::pair<adjEntry, std::pair<int, int>>(adj2, tmpPair));
			if (bends.size() != 0) {
				for (ListIterator<IPoint> i = bends.begin(); i.valid(); k++) {
					p1 = getNodeBendFromNode(e->source());
					p2 = getNodeBendFromNode(e->target());
					NodeBend* tmpNodeBend = new NodeBend((*i), e, k);
					tmpNodeBend->parent1 = p1;
					tmpNodeBend->parent2 = p2;
					if (k == 0) {
						mapAdjEntryFirstNodeBend.insert(std::pair<adjEntry, NodeBend*>(adj1, tmpNodeBend));
					}
					// Marche uniquement pour les bends supplémentaires qui s'initialisent sur le node parent
					if ((tmpNodeBend->getX() == p1->getX()) && (tmpNodeBend->getY() == p1->getY())) {
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
						mapAdjEntryFirstNodeBend.insert(std::pair<adjEntry, NodeBend*>(adj2, tmpNodeBend));
					}
					precedent = tmpNodeBend;
					vectorNodeBends.push_back(tmpNodeBend);
				}
			}
			else {
				mapAdjEntryFirstNodeBend.insert(std::pair<adjEntry, NodeBend*>(adj1, p2));
				mapAdjEntryFirstNodeBend.insert(std::pair<adjEntry, NodeBend*>(adj2, p1));
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
			for (int i = 0; i < vectorEdges.size(); i++) {
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
	}
	// Bertault Layout
	/*
	GraphAttributes GA(G, GraphAttributes::nodeGraphics | GraphAttributes::edgeGraphics);
	BertaultLayout BL;
	node tmpN = G.firstNode();
	while (tmpN != nullptr) {
		GA.x(tmpN) = GL.x(tmpN);
		GA.y(tmpN) = GL.y(tmpN);
		tmpN = tmpN->succ();
	}
	BL.call(GA);
	tmpN = G.firstNode();
	while (tmpN != nullptr) {
		std::cout << "X: " << GA.x(tmpN) << " Y: " << GA.y(tmpN) << std::endl;
		GL.x(tmpN) = GA.x(tmpN);
		GL.y(tmpN) = GA.y(tmpN);
		tmpN = tmpN->succ();
	}
	*/

	gridWidth = maxX;
	gridHeight = maxY;

	// OpenGL
	srand(static_cast<unsigned int>(time(NULL)));
	if (useOpenGL) {
		dispOpenGL(G, GL, gridWidth, gridHeight, maxX, maxY, maxBends, nom_fichier);
	}
	else {
		runAlgo(11, G, GL, gridWidth, gridHeight, maxX, maxY, maxBends, file);
	}
	return 0;
}