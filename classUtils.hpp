#ifndef CLASSUTILS_HPP
#define CLASSUTILS_HPP

#include <ogdf/basic/Graph_d.h>
#include <ogdf/planarlayout/PlanarDrawLayout.h>

// Renvoie une vecteur associant les NodeBend sur vecteur de nodebend passé en parametre a leur coordonnée actuelle
std::vector<std::pair<int,int>> copyGraph(std::vector<NodeBend*> &vectorN1) {
	std::vector<std::pair<int, int>> vectorNodeCoord;
	vectorNodeCoord.reserve(vectorN1.size());
	for (int i = 0; i < vectorN1.size(); i++) {
		std::pair<int, int> tmpPair(vectorN1[i]->getX(), vectorN1[i]->getY());
		vectorNodeCoord.push_back(tmpPair);
	}
	return vectorNodeCoord;
}

// Applique le vecteur de coordonnée passée en parametre
void applyGraph(std::vector<std::pair<int, int>> vectorNodeCoord) {
	for (int i = 0; i < vectorNodeBends.size(); i++) {
		NodeBend* nb = vectorNodeBends[i];
		int newX = vectorNodeCoord[i].first;
		int newY = vectorNodeCoord[i].second;
		removeNodeBendPos(nb, *nb->a_x, *nb->a_y);
		(*nb->a_x) = newX;
		(*nb->a_y) = newY;
		if (nb->isNode) {
			nb->recalculateAdjBendStack();
			posVectorNodeBend[newX][newY].push_front(nb);
		}
		else {
			nb->recalculateIsStacked();
			nb->precedent->recalculateIsStacked();
			nb->suivant->recalculateIsStacked();
			posVectorNodeBend[newX][newY].push_back(nb);
		}
	}
}

// Enregistre les différents embedding d'un graphe dans les fichiers json correspondants
void saveAllEmbeddings(string output, Graph& G, GridLayout& GL, int gridWidth, int gridHeight, int maxBends) {
	std::cout << "Saving embeddings" << std::endl;
	std::vector<std::pair<string, EmbedderModule*>> vectorEmbedding;
	vectorEmbedding.push_back(std::make_pair("MinDepth", new EmbedderMinDepth()));
	vectorEmbedding.push_back(std::make_pair("MaxFace", new EmbedderMaxFace()));
	vectorEmbedding.push_back(std::make_pair("MaxFaceLayers", new EmbedderMaxFaceLayers()));
	vectorEmbedding.push_back(std::make_pair("MinDepthMaxFace", new EmbedderMinDepthMaxFace()));
	vectorEmbedding.push_back(std::make_pair("MinDepthMaxFaceLayers", new EmbedderMinDepthMaxFaceLayers()));
	PlanarStraightLayout PL;
	PL.separation(-19);
	for (int i = 0; i < vectorEmbedding.size(); i++) {
		string new_output = output + "_" + vectorEmbedding[i].first + ".json";
		PL.setEmbedder(vectorEmbedding[i].second);
		PL.callGrid(G, GL);
		writeToJson(new_output, G, GL, gridWidth, gridHeight, maxBends);
	}
}

// Enregistre les différents embedding d'un graphe dans les fichiers json correspondants
void saveAllEmbeddings2(string output, Graph& G, GridLayout& GL, int gridWidth, int gridHeight, int maxBends) {
	std::cout << "Saving embeddings" << std::endl;
	std::vector<std::pair<string, EmbedderModule*>> vectorEmbedding;
	vectorEmbedding.push_back(std::make_pair("MinDepth", new EmbedderMinDepth()));
	vectorEmbedding.push_back(std::make_pair("MaxFace", new EmbedderMaxFace()));
	vectorEmbedding.push_back(std::make_pair("MaxFaceLayers", new EmbedderMaxFaceLayers()));
	vectorEmbedding.push_back(std::make_pair("MinDepthMaxFace", new EmbedderMinDepthMaxFace()));
	vectorEmbedding.push_back(std::make_pair("MinDepthMaxFaceLayers", new EmbedderMinDepthMaxFaceLayers()));
	PlanarDrawLayout PL2;
	PL2.separation(-19);
	for (int i = 0; i < vectorEmbedding.size(); i++) {
		string new_output = output + "_" + vectorEmbedding[i].first + "2.json";
		PL2.setEmbedder(vectorEmbedding[i].second);
		PL2.callGrid(G, GL);
		writeToJson(new_output, G, GL, gridWidth, gridHeight, maxBends);
	}
}
#endif