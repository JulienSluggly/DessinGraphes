#include "pivotMDS.hpp"
#include "graphe.hpp"
#include <iostream>

void PivotMDS::bfs_SPSS(int nodeId, std::vector<double>& shortestPathSingleSource, int edgeCosts) {
    std::vector<bool> mark(G->_noeuds.size(),false);
    std::vector<int> bfs;
    bfs.push_back(nodeId);
    // mark s and set distance to itself 0
    mark[nodeId] = true;
    shortestPathSingleSource[nodeId] = 0;
    int numero = 0;
    while (numero < bfs.size()) {
        int id = bfs[numero];
        double dist = shortestPathSingleSource[id] + edgeCosts;
        for (Noeud* voisin : G->_noeuds[id].voisins) {
            int voisinId = voisin->_id;
            if (!mark[voisinId]) {
                mark[voisinId] = true;
                bfs.push_back(voisinId);
                shortestPathSingleSource[voisinId] = dist;
            }
        }
        numero++;
    }
}

void PivotMDS::getPivotDistanceMatrix() {
	const int n = G->_noeuds.size();

	// lower the number of pivots if necessary
	int numberOfPivots = std::min(n, m_numberOfPivots);

	// number of pivots times n matrix used to store the graph distances
	for (int i = 0; i < numberOfPivots; i++) {
        std::vector<double> tmpPivArray(n,0.0);
		pivDistMatrix.push_back(tmpPivArray);
	}
	// edges costs array
    std::vector<double> edgeCosts;
	// used for min-max strategy
	std::vector<double> minDistances(n, std::numeric_limits<double>::infinity());
	std::vector<double> shortestPathSingleSource(n);
	// the current pivot node
	int pivNode = G->_noeuds[0]._id;
	for (int i = 0; i < numberOfPivots; i++) {
		// get the shortest path from the currently processed pivot node to
		// all other nodes in the graph
        for (int j=0;j<shortestPathSingleSource.size();j++) {
            shortestPathSingleSource[j] = std::numeric_limits<double>::infinity();
        }
		bfs_SPSS(pivNode, shortestPathSingleSource, m_edgeCosts);
        pivDistMatrix[i] = shortestPathSingleSource;
		// update the pivot and the minDistances array ... to ensure the
		// correctness set minDistance of the pivot node to zero
		minDistances[pivNode] = 0;
        for (int nodeId=0; nodeId<G->_noeuds.size();nodeId++) {
            if (minDistances[nodeId] > shortestPathSingleSource[nodeId]) { minDistances[nodeId] = shortestPathSingleSource[nodeId]; }
			if (minDistances[nodeId] > minDistances[pivNode]) { pivNode = nodeId; }
        }
	}
}

void PivotMDS::centerPivotmatrix() {
	int numberOfPivots = pivDistMatrix.size();
	// this is ensured since the graph size is at least 2!
	int nodeCount = pivDistMatrix[0].size();

	double normalizationFactor = 0;
	double rowColNormalizer;
    std::vector<double> colNormalization(numberOfPivots);

	for (int i = 0; i < numberOfPivots; i++) {
		rowColNormalizer = 0;
		for (int j = 0; j < nodeCount; j++) {
			rowColNormalizer += pivDistMatrix[i][j] * pivDistMatrix[i][j];
		}
		normalizationFactor += rowColNormalizer;
		colNormalization[i] = rowColNormalizer / nodeCount;
	}
	normalizationFactor = normalizationFactor / (nodeCount * numberOfPivots);
	for (int i = 0; i < nodeCount; i++) {
		rowColNormalizer = 0;
		for (int j = 0; j < numberOfPivots; j++) {
			double square = pivDistMatrix[j][i] * pivDistMatrix[j][i];
			pivDistMatrix[j][i] = square + normalizationFactor
					- colNormalization[j];
			rowColNormalizer += square;
		}
		rowColNormalizer /= numberOfPivots;
		for (int j = 0; j < numberOfPivots; j++) {
			pivDistMatrix[j][i] = m_factor * (pivDistMatrix[j][i] - rowColNormalizer);
		}
	}
}

void PivotMDS::selfProduct(const std::vector<std::vector<double> >& d, std::vector<std::vector<double> >& result) {
	double sum;
	for (int i = 0; i < d.size(); i++) {
		for (int j = 0; j <= i; j++) {
			sum = 0;
			for (int k = 0; k < d[0].size(); k++) {
				sum += d[i][k] * d[j][k];
			}
			result[i][j] = sum;
			result[j][i] = sum;
		}
	}
}

double PivotMDS::prod(const std::vector<double>& x, const std::vector<double>& y) {
	double result = 0;
	for (int i = 0; i < x.size(); i++) {
		result += x[i] * y[i];
	}
	return result;
}

double PivotMDS::normalize(std::vector<double>& x) {
	double norm = sqrt(prod(x, x));
	if (norm != 0) {
		for (auto &elem : x) {
			elem /= norm;
		}
	}
	return norm;
}

void PivotMDS::randomize(std::vector<std::vector<double> >& matrix) {
	srand(m_seed);
	for (auto &elem : matrix) {
		for (int j = 0; j < elem.size(); j++) {
			elem[j] = ((double) rand()) / RAND_MAX;
		}
	}
}

void PivotMDS::eigenValueDecomposition(std::vector<std::vector<double> >& K, std::vector<std::vector<double> >& eVecs, std::vector<double>& eValues) {
    randomize(eVecs);
	const int p = K.size();
	double r = 0;
	for (int i = 0; i < 2; i++) {
		eValues[i] = normalize(eVecs[i]);
	}
	while (r < m_epsilon) {
		// remember prev values
		std::vector<std::vector<double> > tmpOld;
		for (int i = 0; i < 2; i++) {
            std::vector<double> tmpOldI(p);
            tmpOld.push_back(tmpOldI);
			for (int j = 0; j < p; j++) {
				tmpOld[i][j] = eVecs[i][j];
				eVecs[i][j] = 0;
			}
		}
		// multiply matrices
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < p; j++) {
				for (int k = 0; k < p; k++) {
					eVecs[i][k] += K[j][k] * tmpOld[i][j];
				}
			}
		}
		// orthogonalize
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < i; j++) {
				double fac = prod(eVecs[j], eVecs[i])
						/ prod(eVecs[j], eVecs[j]);
				for (int k = 0; k < p; k++) {
					eVecs[i][k] -= fac * eVecs[j][k];
				}
			}
		}
		// normalize
		for (int i = 0; i < 2; i++) {
			eValues[i] = normalize(eVecs[i]);
		}
		r = 1;
		for (int i = 0; i < 2; i++) {
			// get absolute value (abs only defined for int)
			double tmp = prod(eVecs[i], tmpOld[i]);
			if (tmp < 0) {
				tmp *= -1;
			}
            if (r > tmp) { r = tmp; }
		}
	}
}


void PivotMDS::singularValueDecomposition(std::vector<std::vector<double> >& eVecs, std::vector<double>& eVals){
	const int size = pivDistMatrix.size();
	const int n = pivDistMatrix[0].size();
    std::vector<std::vector<double>> K;
	for (int i = 0; i < size; i++) {
        std::vector<double> tmpK(size);
        K.push_back(tmpK);
	}
	// calc C^TC
	selfProduct(pivDistMatrix, K);

    std::vector<std::vector<double>> tmp;
    std::vector<double> tmpX(size);
    std::vector<double> tmpY(size);
    tmp.push_back(tmpX); tmp.push_back(tmpY);

	eigenValueDecomposition(K, tmp, eVals);

	// C^Tx
	for (int i = 0; i < 2; i++) {
		eVals[i] = sqrt(eVals[i]);
		for (int j = 0; j < n; j++) { // node j
			eVecs[i][j] = 0;
			for (int k = 0; k < size; k++) { // pivot k
				eVecs[i][j] += pivDistMatrix[k][j] * tmp[i][k];
			}
		}
	}
	for (int i = 0; i < 2; i++) {
		normalize(eVecs[i]);
	}
}

void PivotMDS::runAlgo() {
	const int n = G->_noeuds.size();

    // compute the pivot matrix
    getPivotDistanceMatrix();
    // center the pivot matrix
    centerPivotmatrix();
    // init the coordinate matrix
    std::vector<std::vector<double>> coord;
    std::vector<double> coordX(n);
    std::vector<double> coordY(n);
    coord.push_back(coordX); coord.push_back(coordY);
    // init the eigen values array
    std::vector<double> eVals(2);
    singularValueDecomposition(coord, eVals);
    // compute the correct aspect ratio
    for (int i = 0; i < coord.size(); i++) {
        eVals[i] = sqrt(eVals[i]);
        for (int j = 0; j < n; j++) {
            coord[i][j] *= eVals[i];
        }
    }
    // set the new positions to the graph
    double maxX = coord[0][0], minX=maxX;
    double maxY = coord[1][0], minY=maxY;
    for (int i=0;i<G->_noeuds.size();i++) {
        double currX = coord[0][i];
        double currY = coord[1][i];
        if (currX < minX) { minX = currX; }
        else if (currX > maxX) { maxX = currX; }
        if (currY < minY) { minY = currY; }
        else if (currY > maxY) { maxY = currY; }
        G->_noeuds[i].pivotX = currX;
        G->_noeuds[i].pivotY = currY;
    }
    double distXGauche = minX, distXDroite = G->gridWidth - maxX;
    double distYBas = minY, distYHaut = G->gridHeight - maxY;
    double decalX = (distXDroite-distXGauche)/2;
    double decalY = (distYHaut-distYBas)/2;
    for (int i=0;i<G->_noeuds.size();i++) {
        G->_noeuds[i].pivotX += decalX;
        G->_noeuds[i].pivotY += decalY;
    }
    for (int i=0;i<G->_noeuds.size();i++) {
        Emplacement* currentEmp;
        if (m_useGrille) {
            currentEmp = G->getClosestEmplacementFromPointGrid(G->_noeuds[i].pivotX,G->_noeuds[i].pivotY,true);
        }
        else {
            currentEmp = G->getClosestEmplacementFromPoint(G->_noeuds[i].pivotX,G->_noeuds[i].pivotY,true);
        }
        G->_noeuds[i].setEmplacement(currentEmp);
    }
}