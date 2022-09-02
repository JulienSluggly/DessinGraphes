#include <ogdf/basic/GridLayout.h>

#include <ogdf/planarlayout/PlanarStraightLayout.h>
#include <ogdf/planarity/EmbedderMaxFace.h>
#include <ogdf/planarity/EmbedderMaxFaceLayers.h>
#include <ogdf/planarity/EmbedderMinDepth.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFace.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFaceLayers.h>
#include <ogdf/planarity/SimpleEmbedder.h>
#include <ogdf/misclayout/BertaultLayout.h>
#include <ogdf/augmentation/AugmentationModule.h>
#include <ogdf/augmentation/PlanarAugmentation.h>
#include <ogdf/basic/Graph_d.h>

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
	string nom_fichier = "auto21-8_MaxFace2";
	string file = "D:/The World/Cours/M1S2/Graphe/Projet/DessinGraphe/embeddings/auto21-8/" + nom_fichier + ".json";
	//string file = "D:/The World/Cours/M1S2/Graphe/GitHub/ProjetGrapheM1-BinaryHeap/ProjetGrapheM1/bestResult.json";

	bool useOpenGL = true;
	bool useBertault = false;
	bool planarize = false;
	bool displayNodeBends = true;
	bool useNodeBends = true;
	bool useAugmenter = false;
	bool upscaleGrid = true;
	int margeGrid = 300;

	std::cout << "File: " << file << std::endl;
	//string tmpFile = "D:/The World/Cours/M1S2/Graphe/Projet/DessinGraphe/tmp/auto21-10_MaxFace2_CC0Ordered.json";
	string tmpFile = "D:/The World/Cours/M1S2/Graphe/Projet/DessinGraphe/bestResult2.json";
	//readFromJsonDoubleUpscale(tmpFile, G, GL, gridWidth, gridHeight, maxBends);
	readFromJson(tmpFile, G, GL, gridWidth, gridHeight, maxBends);
	//readFromJsonMove(tmpFile, margeGrid, margeGrid, G, GL, gridWidth, gridHeight, maxBends);
	//string combineFile = "D:/The World/Cours/M1S2/Graphe/Projet/DessinGraphe/tmp/auto21-10_MaxFace2_CC";
	//combinesFromJson(combineFile,61, G, GL, gridWidth, gridHeight, maxBends);
	//writeToJson("output.json", G, GL, gridWidth, gridHeight, maxBends);
	//saveInOrder("auto21-10_MaxFace2_CC0Ordered.json", G, GL, gridWidth, gridHeight, maxBends);
	//saveAllConnectedComponents(nom_fichier,G,GL,gridWidth,gridHeight,maxBends);
	//saveAllConnectedComponentsv2(nom_fichier,G,GL,gridWidth,gridHeight,maxBends);

	int maxX = gridWidth, maxY = gridHeight, minX = 0, minY = 0;

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

	if (useAugmenter) {
		std::cout << "Augmenting..." << std::endl;
		PlanarAugmentation AM;
		List<edge> addedEdges;
		AM.call(G, addedEdges);
		std::cout << "Augmented: " << addedEdges.size() << " edges added!" << std::endl;

		for (auto it = addedEdges.begin(); it.valid(); it++) {
			double length = calcEdgeLength((*it), GL);
			mapEdgeLength.insert(std::pair<edge, double>((*it), length));
			std::map<double, std::set<edge>>::iterator it2 = mapLengthEdgeSet.begin();
			it2 = mapLengthEdgeSet.find(length);
			// La valeur est déja présente, on ajoute dans le set
			if (it2 != mapLengthEdgeSet.end()) {
				it2->second.insert((*it));
			}
			// La valeur n'est pas présente, on créer un nouveau set.
			else {
				std::set<edge> tmpSet;
				tmpSet.insert((*it));
				mapLengthEdgeSet.insert(std::pair<double, std::set<edge>>(length, tmpSet));
			}
		}
	}

	std::cout << "Embedding..." << std::endl;
	embedderCarte(G, GL);
	std::cout << "Embeded: " << G.representsCombEmbedding() << std::endl;
	std::cout << "Connexe: " << isConnected(G) << std::endl;
	std::cout << "Planaire: " << isPlanar(G) << std::endl;

	if (useNodeBends) {
		std::cout << "Populating global vectors..." << std::endl;
		posVectorNodeBend.resize(maxX + margeGrid + 1);
		// Remplissage des tableaux globaux
		for (int i = 0; i <= maxX + margeGrid; i++) {
			posVectorNodeBend[i].resize(maxY + margeGrid + 1);
		}
	}

	if (useNodeBends || displayNodeBends) {
		std::cout << "Instanciating NodeBends from nodes..." << std::endl;
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
	}

	if (useNodeBends) {
		if (G.representsCombEmbedding() && isConnected(G) && isPlanar(G)) {
			ConstCombinatorialEmbedding CCE{ G };
			vectorFaceSegment.reserve(CCE.maxFaceIndex() + 1);
			for (int i = 0; i < vectorFaceSegment.capacity(); i++) {
				std::vector<Segment*> tmpVecSegment;
				vectorFaceSegment.push_back(tmpVecSegment);
			}
			std::cout << "Instanciating NodeBends from bends..." << std::endl;
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
			std::cout << "Assigning Global num and position in vectors..." << std::endl;
			// Assign du num global et du tableau de position
			for (int i = 0; i < vectorNodeBends.size(); i++) {
				std::vector<int> tmpVector;
				vectorNodeBends[i]->assignGlobalNum(i);
				if (vectorNodeBends[i]->isNode) {
					posVectorNodeBend[*vectorNodeBends[i]->a_x][*vectorNodeBends[i]->a_y].push_front(vectorNodeBends[i]);
				}
				else {
					posVectorNodeBend[*vectorNodeBends[i]->a_x][*vectorNodeBends[i]->a_y].push_back(vectorNodeBends[i]);
				}
				for (int k = 0; k < vectorNodeBends.size(); k++) {
					tmpVector.push_back(-1);
				}
				vectorSegmentBool.push_back(tmpVector);
			}


			std::cout << "Initializing Segment global vector and Face mapping..." << std::endl;
			// On initialise le tableau global de segment et la map de face a segment
			edge edg = G.firstEdge();
			int numSegment = 0;
			while (edg != nullptr) {
				// On parcour tout les edges et on les transforme en segments ou on regarde s'il existe deja.
				int* srcX, * srcY, * trgX, * trgY;
				NodeBend* source = getNodeBendFromNode(edg->source());
				NodeBend* target = nullptr;
				Segment* s = nullptr;
				srcX = &GL.x(edg->source());
				srcY = &GL.y(edg->source());
				IPolyline& p = GL.bends(edg);
				// Si l'edge contient des bends
				if (p.size() > 0) {
					auto it2 = p.begin();
					while (it2.valid()) {
						trgX = &(*it2).m_x;
						trgY = &(*it2).m_y;
						target = getNodeBendFromBend(&(*it2));
						s = new Segment(srcX, srcY, trgX, trgY);
						s->setSource(source);
						s->setTarget(target);
						s->assignGlobalNum(numSegment);
						vectorSegmentBool[source->globalNum][target->globalNum] = numSegment;
						vectorSegmentBool[target->globalNum][source->globalNum] = numSegment;
						numSegment++;
						vectorSegments.push_back(s);
						vectorFaceSegment[CCE.leftFace(edg->adjSource())->index()].push_back(s);
						vectorFaceSegment[CCE.rightFace(edg->adjSource())->index()].push_back(s);
						srcX = trgX;
						srcY = trgY;
						it2++;
						source = target;
					}
				}
				target = getNodeBendFromNode(edg->target());
				trgX = &GL.x(edg->target());
				trgY = &GL.y(edg->target());
				s = new Segment(srcX, srcY, trgX, trgY);
				s->setSource(source);
				s->setTarget(target);
				s->assignGlobalNum(numSegment);
				vectorSegmentBool[source->globalNum][target->globalNum] = numSegment;
				vectorSegmentBool[target->globalNum][source->globalNum] = numSegment;
				numSegment++;
				vectorSegments.push_back(s);
				vectorFaceSegment[CCE.leftFace(edg->adjSource())->index()].push_back(s);
				vectorFaceSegment[CCE.rightFace(edg->adjSource())->index()].push_back(s);
				edg = edg->succ();
			}

			std::cout << "Calculating stacking status of NodeBends..." << std::endl;
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
	}

	// Bertault Layout
	if (useBertault) {
		GraphAttributes GA(G, GraphAttributes::nodeGraphics | GraphAttributes::edgeGraphics);
		BertaultLayout BL;
		//BL.setImpred(true);
		BL.iterno(G.numberOfNodes());
		node tmpN = G.firstNode();
		while (tmpN != nullptr) {
			GA.x(tmpN) = GL.x(tmpN);
			GA.y(tmpN) = GL.y(tmpN);
			tmpN = tmpN->succ();
		}
		std::cout << "Debut Bertault Layout" << std::endl;
		BL.call(GA);
		std::cout << "Fin Bertault Layout" << std::endl;
		tmpN = G.firstNode();
		while (tmpN != nullptr) {
			std::cout << "X: " << GA.x(tmpN) << " Y: " << GA.y(tmpN) << std::endl;
			GL.x(tmpN) = GA.x(tmpN);
			GL.y(tmpN) = GA.y(tmpN);
			tmpN = tmpN->succ();
		}
		writeToJson("auto21-10BertInt.json", G, GL, gridWidth, gridHeight, maxBends);
		writeToJsonDouble("auto21-10BertDouble.json", G, GA, gridWidth, gridHeight, maxBends);

	}
	if (upscaleGrid) {
		gridWidth = maxX + margeGrid;
		gridHeight = maxY + margeGrid;
	}
	minX = 0;
	minY = 0;
	std::cout << "Setup complete!" << std::endl;

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