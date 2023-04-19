#include "stressMaj.hpp"
#include "graphe.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>

void StressMajorization::initMatrices() {
    shortestPathMatrix.clear();
    weightMatrix.clear();
    for (int i=0;i<G->_noeuds.size();i++) {
        std::vector<double> tmpArrayPath;
        std::vector<double> tmpArrayWeight;
        for (int j=0;j<G->_noeuds.size();j++) {
            if (i==j) {
                tmpArrayPath.push_back(0);
            }
            else {
                tmpArrayPath.push_back(std::numeric_limits<double>::infinity());
            }
            tmpArrayWeight.push_back(0);
        }
        shortestPathMatrix.push_back(tmpArrayPath);
        weightMatrix.push_back(tmpArrayWeight);
    }
    bfs_SPAP();
    if (!G->isGrapheConnected()) { replaceInfinityDistances(m_edgeCosts * sqrt((double)(G->_noeuds.size()))); }
    calcWeights();
}

void StressMajorization::bfs_SPSS(int nodeId, std::vector<double>& shortestPathArray) {
    std::vector<bool> mark(G->_noeuds.size(),false);
    std::vector<int> bfs;
    bfs.push_back(nodeId);
    // mark s and set distance to itself 0
    mark[nodeId] = true;
    shortestPathArray[nodeId] = 0;
    int numero = 0;
    while (numero < bfs.size()) {
        int id = bfs[numero];
        double dist = shortestPathArray[id] + m_edgeCosts;
        for (Noeud* voisin : G->_noeuds[id].voisins) {
            int voisinId = voisin->_id;
            if (!mark[voisinId]) {
                mark[voisinId] = true;
                bfs.push_back(voisinId);
                shortestPathArray[voisinId] = dist;
            }
        }
        numero++;
    }
}

void StressMajorization::bfs_SPAP() {
    for (int i=0;i<G->_noeuds.size();i++) {
        bfs_SPSS(i,shortestPathMatrix[i]);
    }
}

void StressMajorization::calcWeights() {
    for (int i=0;i<G->_noeuds.size();i++) {
        for (int j=0;j<G->_noeuds.size();j++) {
            if (i!=j) {
                // w_ij = d_ij^-2
                weightMatrix[i][j] = 1 / (shortestPathMatrix[i][j] * shortestPathMatrix[i][j]);
            }
        }
    }
}

bool StressMajorization::nextIteration(std::vector<double> customParam) {
    double moyenneDist = 0.0;
    double dist = 0.0;
    int nombreDeplacement = 0;
    std::vector<int> distMatrix(11,0);
    bool converged = true;
    for (int nodeId=0;nodeId<G->_noeuds.size();nodeId++) {
		double newXCoord = 0.0;
		double newYCoord = 0.0;
		double currXCoord = G->_noeuds[nodeId].getX();
		double currYCoord = G->_noeuds[nodeId].getY();
		double totalWeight = 0;
		for (int secondNodeId=0;secondNodeId<G->_noeuds.size();secondNodeId++) {
			if (nodeId == secondNodeId) {
				continue;
			}
			// calculate euclidean distance between both points
			double xDiff = currXCoord - G->_noeuds[secondNodeId].getX();
			double yDiff = currYCoord - G->_noeuds[secondNodeId].getY();
			double euclideanDist = sqrt(xDiff * xDiff + yDiff * yDiff);
			// get the weight
			double weight = weightMatrix[nodeId][secondNodeId];
			// get the desired distance
			double desDistance = shortestPathMatrix[nodeId][secondNodeId];
			// reset the voted x coordinate
            double voteX = G->_noeuds[secondNodeId].getX();
            if (euclideanDist != 0) {
                // calc the vote
                voteX += desDistance * (currXCoord - voteX) / euclideanDist;
            }
            // add the vote
            newXCoord += weight * voteX;
			// reset the voted y coordinate
            double voteY = G->_noeuds[secondNodeId].getY();
            if (euclideanDist != 0) {
                // calc the vote
                voteY += desDistance * (currYCoord - voteY) / euclideanDist;
            }
            newYCoord += weight * voteY;
			// sum up the weights
			totalWeight += weight;
		}
		// update the positions
		if (totalWeight != 0) {
			currXCoord = newXCoord / totalWeight;
			currYCoord = newYCoord / totalWeight;
		}
        dist = sqrt((currXCoord - G->_noeuds[nodeId].getX()) * (currXCoord - G->_noeuds[nodeId].getX())  + (currYCoord - G->_noeuds[nodeId].getY()) * (currYCoord - G->_noeuds[nodeId].getY()));
        if (dist < 10) {
            distMatrix[floor(dist)]++;
        }
        else {
            distMatrix[distMatrix.size()-1]++;
        }
        moyenneDist += dist;
        Emplacement* closestEmplacement;
        if (m_useGrille) {
            closestEmplacement = G->getClosestEmplacementFromPointGrid(currXCoord,currYCoord,m_useClosest);
        }
        else {
            closestEmplacement = G->getClosestEmplacementFromPoint(currXCoord,currYCoord,m_useClosest);
        }
        Emplacement* currentEmplacement = G->_noeuds[nodeId].getEmplacement();
        if (closestEmplacement->_id != currentEmplacement->_id) {
            converged = false;
            nombreDeplacement++;
            if (closestEmplacement->estDisponible()) {
                G->_noeuds[nodeId].setEmplacement(closestEmplacement);
            }
            else {
                int idSwappedNode = closestEmplacement->_noeud->_id;
                G->_noeuds[nodeId].swap(closestEmplacement);
                G->_noeuds[idSwappedNode].stressX = G->_noeuds[idSwappedNode].getEmplacement()->getX();
                G->_noeuds[idSwappedNode].stressY = G->_noeuds[idSwappedNode].getEmplacement()->getY();
            }
            G->_noeuds[nodeId].stressX = G->_noeuds[nodeId].getEmplacement()->getX();
            G->_noeuds[nodeId].stressY = G->_noeuds[nodeId].getEmplacement()->getY();
        }
	}
    bool log = false;
    if (log) {
        moyenneDist = moyenneDist / (double)G->_noeuds.size();
        std::string nomFichier = chemin + "/resultats/" + G->nomGraphe + "ST.csv";
        std::ofstream resultats(nomFichier, std::ios_base::app);
        resultats << std::fixed << moyenneDist << "," << nombreDeplacement << ",";
        for (int i=0;i<distMatrix.size();i++) {
            resultats << distMatrix[i] << ",";
        }
        resultats << "\n";
        resultats.close();
    }
    return converged;
}

bool StressMajorization::nextIterationReel() {
    int nombreDeplacement = 0;
    for (int nodeId=0;nodeId<G->_noeuds.size();nodeId++) {
		double newXCoord = 0.0;
		double newYCoord = 0.0;
		double currXCoord = G->_noeuds[nodeId]._xreel;
		double currYCoord = G->_noeuds[nodeId]._yreel;
		double totalWeight = 0;
		for (int secondNodeId=0;secondNodeId<G->_noeuds.size();secondNodeId++) {
			if (nodeId == secondNodeId) {
				continue;
			}
			// calculate euclidean distance between both points
			double xDiff = currXCoord - G->_noeuds[secondNodeId]._xreel;
			double yDiff = currYCoord - G->_noeuds[secondNodeId]._yreel;
			double euclideanDist = sqrt(xDiff * xDiff + yDiff * yDiff);
			// get the weight
			double weight = weightMatrix[nodeId][secondNodeId];
			// get the desired distance
			double desDistance = shortestPathMatrix[nodeId][secondNodeId];
			// reset the voted x coordinate
            double voteX = G->_noeuds[secondNodeId]._xreel;
            if (euclideanDist != 0) {
                // calc the vote
                voteX += desDistance * (currXCoord - voteX) / euclideanDist;
            }
            // add the vote
            newXCoord += weight * voteX;
			// reset the voted y coordinate
            double voteY = G->_noeuds[secondNodeId]._yreel;
            if (euclideanDist != 0) {
                // calc the vote
                voteY += desDistance * (currYCoord - voteY) / euclideanDist;
            }
            newYCoord += weight * voteY;
			// sum up the weights
			totalWeight += weight;
		}
		// update the positions
		if (totalWeight != 0) {
			currXCoord = newXCoord / totalWeight;
			currYCoord = newYCoord / totalWeight;
            if ((currXCoord != G->_noeuds[nodeId]._xreel)||(currYCoord != G->_noeuds[nodeId]._yreel)) {
                nombreDeplacement++;
                G->_noeuds[nodeId]._xreel = currXCoord;
                G->_noeuds[nodeId]._yreel = currYCoord;
            }
		}
	}
    return nombreDeplacement==0;
}

bool StressMajorization::nextIterationDelayTest(int iteration,std::vector<double> customParam) {
    double moyenneDist = 0.0;
    double dist = 0.0;
    int nombreDeplacement = 0;
    int delai=1, numero = 1;
    if (customParam.size() > 1) {
        if (customParam[0] == 4) { delai = customParam[1]; }
        else if (customParam[0] == 5) { numero = customParam[1]; }
    }
    std::vector<int> distMatrix(11,0);
    for (int nodeId=0;nodeId<G->_noeuds.size();nodeId++) {
        if ((nodeId+(iteration%numero))%numero == 0) {
		double newXCoord = 0.0;
		double newYCoord = 0.0;
		double currXCoord = G->_noeuds[nodeId].stressX;
		double currYCoord = G->_noeuds[nodeId].stressY;
		double totalWeight = 0;
		for (int secondNodeId=0;secondNodeId<G->_noeuds.size();secondNodeId++) {
			if (nodeId == secondNodeId) {
				continue;
			}
			// calculate euclidean distance between both points
			double xDiff = currXCoord - G->_noeuds[secondNodeId].stressX;
			double yDiff = currYCoord - G->_noeuds[secondNodeId].stressY;
			double euclideanDist = sqrt(xDiff * xDiff + yDiff * yDiff);
			// get the weight
			double weight = weightMatrix[nodeId][secondNodeId];
			// get the desired distance
			double desDistance = shortestPathMatrix[nodeId][secondNodeId];
			// reset the voted x coordinate
            double voteX = G->_noeuds[secondNodeId].stressX;
            if (euclideanDist != 0) {
                // calc the vote
                voteX += desDistance * (currXCoord - voteX) / euclideanDist;
            }
            // add the vote
            newXCoord += weight * voteX;
			// reset the voted y coordinate
            double voteY = G->_noeuds[secondNodeId].stressY;
            if (euclideanDist != 0) {
                // calc the vote
                voteY += desDistance * (currYCoord - voteY) / euclideanDist;
            }
            newYCoord += weight * voteY;
			// sum up the weights
			totalWeight += weight;
		}
		// update the positions
		if (totalWeight != 0) {
			currXCoord = newXCoord / totalWeight;
			currYCoord = newYCoord / totalWeight;
            G->_noeuds[nodeId].stressX = currXCoord;
            G->_noeuds[nodeId].stressY = currYCoord;
		}
        dist = sqrt((currXCoord - G->_noeuds[nodeId].getX()) * (currXCoord - G->_noeuds[nodeId].getX())  + (currYCoord - G->_noeuds[nodeId].getY()) * (currYCoord - G->_noeuds[nodeId].getY()));
        if (dist < 10) {
            distMatrix[floor(dist)]++;
        }
        else {
            distMatrix[distMatrix.size()-1]++;
        }
        moyenneDist += dist;
        }
	}
    if (iteration%delai == 0) {
        for (int i=0;i<G->_noeuds.size();i++) {
            if ((i+(iteration%numero))%numero == 0) {
            Emplacement* closestEmplacement;
            double currXCoord = G->_noeuds[i].stressX;
            double currYCoord = G->_noeuds[i].stressY;
            if (m_useGrille) {
                closestEmplacement = G->getClosestEmplacementFromPointGrid(currXCoord,currYCoord,m_useClosest);
            }
            else {
                closestEmplacement = G->getClosestEmplacementFromPoint(currXCoord,currYCoord,m_useClosest);
            }
            Emplacement* currentEmplacement = G->_noeuds[i].getEmplacement();
            if (closestEmplacement->_id != currentEmplacement->_id) {
                nombreDeplacement++;
                if (closestEmplacement->estDisponible()) {
                    G->_noeuds[i].setEmplacement(closestEmplacement);
                }
                else {
                    int idSwappedNode = closestEmplacement->_noeud->_id;
                    G->_noeuds[i].swap(closestEmplacement);
                    G->_noeuds[idSwappedNode].stressX = G->_noeuds[idSwappedNode].getEmplacement()->getX();
                    G->_noeuds[idSwappedNode].stressY = G->_noeuds[idSwappedNode].getEmplacement()->getY();
                }
                G->_noeuds[i].stressX = G->_noeuds[i].getEmplacement()->getX();
                G->_noeuds[i].stressY = G->_noeuds[i].getEmplacement()->getY();
            }
            }
        }
    }
    bool log = false;
    if (log) {
        moyenneDist = moyenneDist / (double)G->_noeuds.size();
        std::string nomFichier = chemin + "/resultats/" + G->nomGraphe + "ST.csv";
        std::ofstream resultats(nomFichier, std::ios_base::app);
        resultats << std::fixed << moyenneDist << "," << nombreDeplacement << ",";
        for (int i=0;i<distMatrix.size();i++) {
            resultats << distMatrix[i] << ",";
        }
        resultats << "\n";
        resultats.close();
    }
    return false;
}

bool StressMajorization::nextIterationDelay(int iteration) {
    int nombreDeplacement = 0;
    for (int nodeId=0;nodeId<G->_noeuds.size();nodeId++) {
		double newXCoord = 0.0;
		double newYCoord = 0.0;
		double currXCoord = G->_noeuds[nodeId].stressX;
		double currYCoord = G->_noeuds[nodeId].stressY;
		double totalWeight = 0;
		for (int secondNodeId=0;secondNodeId<G->_noeuds.size();secondNodeId++) {
			if (nodeId == secondNodeId) {
				continue;
			}
			// calculate euclidean distance between both points
			double xDiff = currXCoord - G->_noeuds[secondNodeId].stressX;
			double yDiff = currYCoord - G->_noeuds[secondNodeId].stressY;
			double euclideanDist = sqrt(xDiff * xDiff + yDiff * yDiff);
			// get the weight
			double weight = weightMatrix[nodeId][secondNodeId];
			// get the desired distance
			double desDistance = shortestPathMatrix[nodeId][secondNodeId];
			// reset the voted x coordinate
            double voteX = G->_noeuds[secondNodeId].stressX;
            if (euclideanDist != 0) {
                // calc the vote
                voteX += desDistance * (currXCoord - voteX) / euclideanDist;
            }
            // add the vote
            newXCoord += weight * voteX;
			// reset the voted y coordinate
            double voteY = G->_noeuds[secondNodeId].stressY;
            if (euclideanDist != 0) {
                // calc the vote
                voteY += desDistance * (currYCoord - voteY) / euclideanDist;
            }
            newYCoord += weight * voteY;
			// sum up the weights
			totalWeight += weight;
		}
		// update the positions
		if (totalWeight != 0) {
			currXCoord = newXCoord / totalWeight;
			currYCoord = newYCoord / totalWeight;
            G->_noeuds[nodeId].stressX = currXCoord;
            G->_noeuds[nodeId].stressY = currYCoord;
		}
	}
    
    for (int i=0;i<G->_noeuds.size();i++) {
        Emplacement* closestEmplacement;
        double currXCoord = G->_noeuds[i].stressX;
        double currYCoord = G->_noeuds[i].stressY;
        if (m_useClosest) {
            G->clearNodeEmplacement();
        }
        if (m_useGrille) {
            closestEmplacement = G->getClosestEmplacementFromPointGrid(currXCoord,currYCoord,m_useClosest);
        }
        else {
            closestEmplacement = G->getClosestEmplacementFromPoint(currXCoord,currYCoord,m_useClosest);
        }
        Emplacement* currentEmplacement = G->_noeuds[i].getEmplacement();
        if (closestEmplacement->_id != currentEmplacement->_id) {
            nombreDeplacement++;
            if (closestEmplacement->estDisponible()) {
                G->_noeuds[i].setEmplacement(closestEmplacement);
            }
            else {
                int idSwappedNode = closestEmplacement->_noeud->_id;
                G->_noeuds[i].swap(closestEmplacement);
                G->_noeuds[idSwappedNode].stressX = G->_noeuds[idSwappedNode].getEmplacement()->getX();
                G->_noeuds[idSwappedNode].stressY = G->_noeuds[idSwappedNode].getEmplacement()->getY();
            }
            G->_noeuds[i].stressX = G->_noeuds[i].getEmplacement()->getX();
            G->_noeuds[i].stressY = G->_noeuds[i].getEmplacement()->getY();
        }
    }
    return nombreDeplacement == 0;
}

bool StressMajorization::finished(int numberOfPerformedIterations) {
    return numberOfPerformedIterations >= m_iterations;
}

void StressMajorization::minimizeStress(std::vector<double> customParam) {
    bool converged;
	do {
		converged = nextIterationDelay(totalIterationDone);
        totalIterationDone++;
	} while (!converged && !finished(totalIterationDone));
}

void StressMajorization::minimizeStressReel() {
    bool converged;
    double curStress = calcStressReel();
    double prevStress;
	do {
        prevStress = curStress;
		converged = nextIterationReel();
        curStress = calcStressReel();
        totalIterationDone++;
	} while (!converged && !(curStress == 0 || prevStress - curStress < prevStress * m_epsilon));
}

void StressMajorization::updateStressCoord() {
    for (int i=0;i<G->_noeuds.size();i++) {
        G->_noeuds[i].stressX = G->_noeuds[i].getEmplacement()->getX();
        G->_noeuds[i].stressY = G->_noeuds[i].getEmplacement()->getY();
    }
}

double StressMajorization::minimizeStressDynStress(int nombreIter, std::vector<double> customParam) {
    updateStressCoord();
    double bestStress = calcStress();
    int iter = 0;
    int numberNoUpgrade = 0;
    bool converged;
    double stress;
    int nbIterCheckStress = 5;
	do {
		converged = nextIterationDelay(totalIterationDone);
        totalIterationDone++;
        iter++;
        if (iter%nbIterCheckStress == 0) {
            stress = calcStress();
            if (stress < bestStress) {
                bestStress = stress;
                numberNoUpgrade = 0;
            }
            else if (stress > bestStress) {
                numberNoUpgrade++;
            }
        }
	} while (!converged && iter < nombreIter && numberNoUpgrade < 10/nbIterCheckStress);
    return bestStress;
}

long StressMajorization::minimizeStressDynCross(int nombreIter, std::vector<double> customParam) {
    updateStressCoord();
    long bestCross = G->getNbCroisementDiff();
    int iter = 0;
    int numberNoUpgrade = 0;
    bool converged;
    long cross;
    int nbIterCheckCross = 5;
	do {
		converged = nextIterationDelay(totalIterationDone);
        totalIterationDone++;
        iter++;
        if (iter%nbIterCheckCross == 0) {
            cross = G->getNbCroisementDiff();
            if (cross < bestCross) {
                bestCross = cross;
                numberNoUpgrade = 0;
            }
            else if (cross > bestCross) {
                numberNoUpgrade++;
            }
        }
	} while (!converged && iter < nombreIter && numberNoUpgrade < 10/nbIterCheckCross);
    return bestCross;
}

void StressMajorization::replaceInfinityDistances(double newVal) {
    for (int i=0;i<G->_noeuds.size();i++) {
        for (int j=0;j<G->_noeuds.size();j++) {
            if (i != j) {
                if (isinf(shortestPathMatrix[i][j])) {
                    shortestPathMatrix[i][j] = newVal;
                }
            }
        }
    }
}

void StressMajorization::runAlgo(std::vector<double> customParam) {
    m_edgeCosts = 45;
    initMatrices();
    minimizeStress();
}

void StressMajorization::runAlgoReel() {
    m_edgeCosts = 45;
    initMatrices();
    minimizeStressReel();
}

void StressMajorization::runStepAlgo(std::vector<double> customParam) {
    if (!areVectorsSetup) {
        initMatrices();
        m_iterations = 1;
        areVectorsSetup = true;
        totalIterationDone = 0;
    }
    minimizeStress();
}

double StressMajorization::calcStress(){
	double stress = 0;
    for (int i=0; i<G->_noeuds.size()-1;i++) {
        for (int j=i+1;j<G->_noeuds.size();j++) {
            double xDiff = G->_noeuds[i].stressX - G->_noeuds[j].stressX;
            double yDiff = G->_noeuds[i].stressY - G->_noeuds[j].stressY;
			double dist = sqrt(xDiff * xDiff + yDiff * yDiff);
			if (dist != 0) {
				stress += weightMatrix[i][j] * (shortestPathMatrix[i][j] - dist) * (shortestPathMatrix[i][j] - dist);
			}
        }
    }
	return stress;
}

double StressMajorization::calcStressReel(){
	double stress = 0;
    for (int i=0; i<G->_noeuds.size()-1;i++) {
        for (int j=i+1;j<G->_noeuds.size();j++) {
            double xDiff = G->_noeuds[i]._xreel - G->_noeuds[j]._xreel;
            double yDiff = G->_noeuds[i]._yreel - G->_noeuds[j]._yreel;
			double dist = sqrt(xDiff * xDiff + yDiff * yDiff);
			if (dist != 0) {
				stress += weightMatrix[i][j] * (shortestPathMatrix[i][j] - dist) * (shortestPathMatrix[i][j] - dist);
			}
        }
    }
	return stress;
}

void StressMajorization::addToEdgeCost(double n) {
    m_edgeCosts += n;
    initMatrices();
}

void StressMajorization::setEdgeCost(double n) {
    m_edgeCosts = n;
    initMatrices();
}

void StressMajorization::runAlgoDynStress() {
    setEdgeCost(1);
    double bestEdgeCost = m_edgeCosts;
    double bestStress = calcStress();
    double stress;
    double diffStress;
    double pourcentDiff=100.0;
    int nombreNoUpgrade = 0;
    double seuilAmelioration = 1.0;
    int seuilNoUpgrade = 10;
    std::vector<int> GCopy;
    // Recherche de la bonne taille de m_edgeCosts en trouvant celle qui minimize le stress du graphe.
    do {
        stress = minimizeStressDynStress();
        nombreNoUpgrade++;
        if (stress <= bestStress) {
            diffStress = bestStress - stress;
            pourcentDiff = diffStress/bestStress*100;
            bestEdgeCost = m_edgeCosts;
            bestStress = stress;
            GCopy = G->saveCopy();
            if (pourcentDiff > seuilAmelioration) {
                nombreNoUpgrade = 0;
            }
        }
        if (DEBUG_STRESS) std::cout << std::fixed << m_edgeCosts << " " << std::setprecision(0) << stress << " %: " << std::setprecision(2) << pourcentDiff << " NoUp: " << nombreNoUpgrade << std::endl;
        addToEdgeCost(1);
    } while (nombreNoUpgrade < seuilNoUpgrade);
    // Rotation complete du graphe en cherchant celle qui minimize le stress du graphe.
    double bestAngle = 0.0;
    double increment = 10.0;
    setEdgeCost(bestEdgeCost);
    for (int i=1;i<=35;i++) { // 35 Rotations de 10° d'incrément
        G->loadCopy(GCopy);
        G->rotateGraph(increment*i);
        stress = minimizeStressDynStress();
        if (stress <= bestStress) {
            bestStress = stress;
            bestAngle = increment * i;
            GCopy = G->saveCopy();
        }
        if (DEBUG_STRESS) std::cout << "Rotation: " << increment*i << "°" << " " << std::setprecision(0) << stress << " BestAngle: " << std::setprecision(1) << bestAngle << std::endl;
    }
    // Meilleur rotation trouvée, on peut rechercher de nouveau la meilleur longeur de coté
    G->loadCopy(GCopy);
    G->rotateGraph(bestAngle);
    nombreNoUpgrade = 0;
    do {
        stress = minimizeStressDynStress();
        nombreNoUpgrade++;
        if (stress <= bestStress) {
            diffStress = bestStress - stress;
            pourcentDiff = diffStress/bestStress*100;
            bestEdgeCost = m_edgeCosts;
            bestStress = stress;
            GCopy = G->saveCopy();
            if (pourcentDiff > seuilAmelioration) {
                nombreNoUpgrade = 0;
            }
        }
        if (DEBUG_STRESS) std::cout << std::fixed << m_edgeCosts << " " << std::setprecision(0) << stress << " %: " << std::setprecision(2) << pourcentDiff << " NoUp: " << nombreNoUpgrade << std::endl;
        addToEdgeCost(1);
    } while (nombreNoUpgrade < seuilNoUpgrade);
    G->loadCopy(GCopy);
    // Dernier fine tuning avec la meilleur taille.
    setEdgeCost(bestEdgeCost);
    minimizeStressDynStress();
}

void StressMajorization::runAlgoDynCross() {
    setEdgeCost(1);
    double bestEdgeCost = m_edgeCosts;
    long bestCross = G->getNbCroisementDiff();
    long cross;
    long diffCross;
    double pourcentDiff=100.0;
    int nombreNoUpgrade = 0;
    double seuilAmelioration = 1.0;
    int seuilNoUpgrade = 10;
    std::vector<int> GCopy;
    // Recherche de la bonne taille de m_edgeCosts en trouvant celle qui minimize le stress du graphe.
    do {
        cross = minimizeStressDynCross();
        nombreNoUpgrade++;
        if (cross <= bestCross) {
            diffCross = bestCross - cross;
            pourcentDiff = (double)diffCross/(double)bestCross*100.0;
            bestEdgeCost = m_edgeCosts;
            bestCross = cross;
            GCopy = G->saveCopy();
            if (pourcentDiff > seuilAmelioration) {
                nombreNoUpgrade = 0;
            }
        }
        if (DEBUG_STRESS) std::cout << std::fixed << m_edgeCosts << " " << std::setprecision(0) << cross << " %: " << std::setprecision(2) << pourcentDiff << " NoUp: " << nombreNoUpgrade << std::endl;
        addToEdgeCost(1);
    } while (nombreNoUpgrade < seuilNoUpgrade);
    // Rotation complete du graphe en cherchant celle qui minimize le stress du graphe.
    double bestAngle = 0.0;
    double increment = 10.0;
    setEdgeCost(bestEdgeCost);
    for (int i=1;i<=35;i++) { // 35 Rotations de 10° d'incrément
        G->loadCopy(GCopy);
        G->rotateGraph(increment*i);
        cross = minimizeStressDynCross();
        if (cross <= bestCross) {
            bestCross = cross;
            bestAngle = increment * i;
            GCopy = G->saveCopy();
        }
        if (DEBUG_STRESS) std::cout << "Rotation: " << increment*i << "°" << " " << std::setprecision(0) << cross << " BestAngle: " << std::setprecision(1) << bestAngle << std::endl;
    }
    // Meilleur rotation trouvée, on peut rechercher de nouveau la meilleur longeur de coté
    G->loadCopy(GCopy);
    G->rotateGraph(bestAngle);
    nombreNoUpgrade = 0;
    do {
        cross = minimizeStressDynCross();
        nombreNoUpgrade++;
        if (cross <= bestCross) {
            diffCross = bestCross - cross;
            pourcentDiff = (double)diffCross/(double)bestCross*100.0;
            bestEdgeCost = m_edgeCosts;
            bestCross = cross;
            GCopy = G->saveCopy();
            if (pourcentDiff > seuilAmelioration) {
                nombreNoUpgrade = 0;
            }
        }
        if (DEBUG_STRESS) std::cout << std::fixed << m_edgeCosts << " " << std::setprecision(0) << cross << " %: " << std::setprecision(2) << pourcentDiff << " NoUp: " << nombreNoUpgrade << std::endl;
        addToEdgeCost(1);
    } while (nombreNoUpgrade < seuilNoUpgrade);
    G->loadCopy(GCopy);
    // Dernier fine tuning avec la meilleur taille.
    setEdgeCost(bestEdgeCost);
    minimizeStressDynCross();
}

void StressMajorization::runAlgoDynDichStress() {
    double borneInf = 1.0;
    double borneSup = min(G->gridHeight,G->gridWidth);
    double m1, m2;
    m1 = (borneSup-borneInf)/3.0 + borneInf;
    m2 = 2.0*((borneSup-borneInf)/3.0) + borneInf;
    double stressM1 = -1.0, stressM2;
    double seuilLimite = 0.01;
    double bestStress = -1.0;
    int tirageAleatoire = 0;
    int nbTirageNoChange = 0;
    bool finished = (ceil(m1) == ceil(m2));
    std::vector<int> GCopy;
    while (!finished) {
        std::cout << "BorneInf: " << borneInf << " m1: " << m1 << " m2: " << m2 << " BorneSup: " << borneSup << std::endl;
        setEdgeCost(m1);
        stressM1 = minimizeStressDynStress();
        setEdgeCost(m2);
        stressM2 = minimizeStressDynStress();
        std::cout << "StressM1: " << stressM1 << " StressM2: " << stressM2 << std::endl;
        if (abs(stressM1-stressM2) <= max(stressM1,stressM2)*seuilLimite) { // Egalite
            if (tirageAleatoire == 0) { m1 = generateDoubleRand(m2-borneInf) + borneInf; }
            else { m2 = generateDoubleRand(borneSup-m2)+m2; }
            tirageAleatoire = (tirageAleatoire+1)%2;
            nbTirageNoChange++;
        }
        else {
            if (stressM1 < stressM2) {
                borneSup = m2;
                bestStress = stressM1;
            }
            else {
                borneInf = m1;
                bestStress = stressM2;
            }
            nbTirageNoChange = 0;
            m1 = (borneSup-borneInf)/3.0 + borneInf;
            m2 = 2.0*((borneSup-borneInf)/3.0) + borneInf;
            finished = (ceil(m1) == ceil(m2));
        }
        if (nbTirageNoChange > 10) {
            finished = true;
        }
    }
    std::cout << "Dichotomie End\n";
    double bestEdgeCost = m1;
    // Rotation complete du graphe en cherchant celle qui minimize le stress du graphe.
    double bestAngle = 0.0;
    double increment = 10.0;
    double stress;
    double diffStress;
    double pourcentDiff=100.0;
    int nombreNoUpgrade = 0;
    double seuilAmelioration = 1.0;
    int seuilNoUpgrade = 10;
    setEdgeCost(bestEdgeCost);
    minimizeStressDynStress();
    GCopy = G->saveCopy();
    for (int i=1;i<=35;i++) { // 35 Rotations de 10° d'incrément
        G->loadCopy(GCopy);
        G->rotateGraph(increment*i);
        stress = minimizeStressDynStress();
        if (stress <= bestStress) {
            bestStress = stress;
            bestAngle = increment * i;
            GCopy = G->saveCopy();
        }
        if (DEBUG_STRESS) std::cout << "Rotation: " << increment*i << "°" << " " << std::setprecision(0) << stress << " BestAngle: " << std::setprecision(1) << bestAngle << std::endl;
    }
    // Meilleur rotation trouvée, on peut rechercher de nouveau la meilleur longeur de coté
    G->loadCopy(GCopy);
    G->rotateGraph(bestAngle);
    nombreNoUpgrade = 0;
    do {
        stress = minimizeStressDynStress();
        nombreNoUpgrade++;
        if (stress <= bestStress) {
            diffStress = bestStress - stress;
            pourcentDiff = diffStress/bestStress*100;
            bestEdgeCost = m_edgeCosts;
            bestStress = stress;
            GCopy = G->saveCopy();
            if (pourcentDiff > seuilAmelioration) {
                nombreNoUpgrade = 0;
            }
        }
        if (DEBUG_STRESS) std::cout << std::fixed << m_edgeCosts << " " << std::setprecision(0) << stress << " %: " << std::setprecision(2) << pourcentDiff << " NoUp: " << nombreNoUpgrade << std::endl;
        addToEdgeCost(1);
    } while (nombreNoUpgrade < seuilNoUpgrade);
    G->loadCopy(GCopy);
    // Dernier fine tuning avec la meilleur taille.
    setEdgeCost(bestEdgeCost);
    minimizeStressDynStress();
}