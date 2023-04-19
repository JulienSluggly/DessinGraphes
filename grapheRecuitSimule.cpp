#include "graphe.hpp"
#include <vector>
#include <chrono>
#include <iostream>
#include <climits>
#include <algorithm>
#include <omp.h>
#include <gperftools/profiler.h>

// Selectionne deux noeud et retourne celui avec le score le plus faible. (le moin d'intersection)
int Graphe::selectionNoeudTournoiBinaire(bool isScoreUpdated) {
    int randomId = generateRand(_noeuds.size() - 1);
    int randomId2;
    do {
        randomId2 = generateRand(_noeuds.size() - 1);
    } while (randomId2 == randomId);
    if (isScoreUpdated) {
        if (_noeuds[randomId].score > _noeuds[randomId2].score)
            return randomId;
    }
    else {
    if (getScoreCroisementNode(randomId) > getScoreCroisementNode(randomId2))
        return randomId;
    }
    return randomId2;
}

// Selectionne n noeuds et retourne celui avec le score le plus faible. (le moin d'intersection)
int Graphe::selectionNoeudTournoiMultiple(int n, bool isScoreUpdated) {
    int randomId = generateRand(_noeuds.size() - 1);
    long scoreMeilleur;
    if (isScoreUpdated)
        scoreMeilleur = _noeuds[randomId].score;
    else
        scoreMeilleur = getScoreCroisementNode(randomId);
    if (n > 1) {
        for (int i = 0; i < n; i++) {
            int randomId2;
            do {
                randomId2 = generateRand(_noeuds.size() - 1);
            } while (randomId2 == randomId);
            long scoreId2;
            if (isScoreUpdated)
                scoreId2 = _noeuds[randomId2].score;
            else
                scoreId2 = getScoreCroisementNode(randomId2);
            if (scoreId2 > scoreMeilleur) {
                randomId = randomId2;
                scoreMeilleur = scoreId2;
            }
        }
    }
    return randomId;
}

// Selectionne deux emplacements different entre eux et de celui du noeud, puis renvoie le plus proche du noeud.
// Le noeud doit etre place.
int Graphe::selectionEmplacementTournoiBinaire(int nodeId) {
    int randomEmpId = generateRand(_emplacements.size() - 1); // Selection aléatoire d'un emplacement disponible (pas tres équiprobable)
    // on retire si on pioche le meme emplacement
    while (_noeuds[nodeId].getEmplacement()->getId() == randomEmpId) {
        randomEmpId = generateRand(_emplacements.size() - 1);
    }
    int randomEmpId2 = generateRand(_emplacements.size() - 1); // Selection aléatoire d'un emplacement disponible (pas tres équiprobable)
    // on retire si on pioche le meme emplacement
    while ((_noeuds[nodeId].getEmplacement()->getId() == randomEmpId2)||(randomEmpId2 == randomEmpId)) {
        randomEmpId2 = generateRand(_emplacements.size() - 1);
    }
    double dist1 = ((_emplacements[randomEmpId].getX() - _noeuds[nodeId].getX()) * (_emplacements[randomEmpId].getX() - _noeuds[nodeId].getX())) + ((_emplacements[randomEmpId].getY() - _noeuds[nodeId].getY()) * (_emplacements[randomEmpId].getY() - _noeuds[nodeId].getY()));
    double dist2 = ((_emplacements[randomEmpId2].getX() - _noeuds[nodeId].getX()) * (_emplacements[randomEmpId2].getX() - _noeuds[nodeId].getX())) + ((_emplacements[randomEmpId2].getY() - _noeuds[nodeId].getY()) * (_emplacements[randomEmpId2].getY() - _noeuds[nodeId].getY()));
    if (dist2 < dist1) {
        randomEmpId = randomEmpId2;
    }
    return randomEmpId;
}

// Selectionne n emplacements different entre eux et de celui du noeud, puis renvoie le plus proche du noeud.
// Le noeud doit etre place.
int Graphe::selectionEmplacementTournoiMultiple(int n, int nodeId) {
    int randomEmpId = generateRand(_emplacements.size() - 1); // Selection aléatoire d'un emplacement disponible (pas tres équiprobable)
    // on retire si on pioche le meme emplacement
    while (_noeuds[nodeId].getEmplacement()->getId() == randomEmpId) {
        randomEmpId = generateRand(_emplacements.size() - 1);
    }
    double dist;
    if (n > 1) {
        dist = ((_emplacements[randomEmpId].getX() - _noeuds[nodeId].getX()) * (_emplacements[randomEmpId].getX() - _noeuds[nodeId].getX())) + ((_emplacements[randomEmpId].getY() - _noeuds[nodeId].getY()) * (_emplacements[randomEmpId].getY() - _noeuds[nodeId].getY()));
    }
    for (int i = 1; i < n; i++) {
        int randomEmpId2 = generateRand(_emplacements.size() - 1); // Selection aléatoire d'un emplacement disponible (pas tres équiprobable)
        // on retire si on pioche le meme emplacement
        while ((_noeuds[nodeId].getEmplacement()->getId() == randomEmpId2)||(randomEmpId2 == randomEmpId)) {
            randomEmpId2 = generateRand(_emplacements.size() - 1);
        }
        double dist2 = ((_emplacements[randomEmpId2].getX() - _noeuds[nodeId].getX()) * (_emplacements[randomEmpId2].getX() - _noeuds[nodeId].getX())) + ((_emplacements[randomEmpId2].getY() - _noeuds[nodeId].getY()) * (_emplacements[randomEmpId2].getY() - _noeuds[nodeId].getY()));
        if (dist2 < dist) {
            randomEmpId = randomEmpId2;
            dist = dist2;
        }
    }
    return randomEmpId;
}

int Graphe::selectionEmplacementTriangulation(int nodeId, int profondeur) {
    Emplacement* depart = _noeuds[nodeId].getEmplacement();
    Emplacement* arrive = depart;
    int randomId=-1;
    for (int i=0;i<profondeur;i++) {
        if (i == profondeur - 1) {
            if (arrive->voisinsDelaunay->size() == 1) {
                if (arrive->voisinsDelaunay->at(0) == depart->_id) {
                    return arrive->_id;
                }
            }
            else {
                randomId = generateRand(arrive->voisinsDelaunay->size() - 1);
                while (arrive->voisinsDelaunay->at(randomId) == depart->_id) {
                    randomId = generateRand(arrive->voisinsDelaunay->size() - 1);
                }
            }
        }
        else {
            randomId = generateRand(arrive->voisinsDelaunay->size() - 1);
            while (arrive->voisinsDelaunay->at(randomId) == depart->_id) {
                randomId = generateRand(arrive->voisinsDelaunay->size() - 1);
            }
        }
        arrive = &_emplacements[arrive->voisinsDelaunay->at(randomId)];
    }
    return arrive->_id;
}

// Effectue la selection du noeud en fonction de modeNoeud, 0=Aleatoire,1=TournoiBinaire,2=TournoiMultiple
int Graphe::selectionNoeud(int modeNoeud, int t, bool isScoreUpdated) {
    int nodeId=-1;
    if (modeNoeud == 0)
        nodeId = generateRand(_noeuds.size() - 1);
    else if (modeNoeud == 1) {
        nodeId = selectionNoeudTournoiBinaire(isScoreUpdated);
    }
    else if (modeNoeud == 2) {
        int nbTirageNoeud = ((100 - t) / 15) + 1;
        nodeId = selectionNoeudTournoiMultiple(nbTirageNoeud,isScoreUpdated);
    }
    return nodeId;
}

// Effectue la selection de l'emplacement en fonction de modeEmplacement, 0=Aleatoire,1=TournoiBinaire,2=TournoiMultiple,3=Triangulation
int Graphe::selectionEmplacement(int modeEmplacement, int nodeId, int t, std::vector<std::vector<double>>& customParam, int iter) {
    int slotId = -1;
    switch (modeEmplacement) {
    case 0: { // Random
        slotId = generateRand(_emplacements.size() - 1); // Selection aléatoire d'un emplacement disponible (pas tres équiprobable)
        // on retire si on pioche le meme emplacement
        while (_noeuds[nodeId].getEmplacement()->getId() == slotId) {
            slotId = generateRand(_emplacements.size() - 1);
        }
        break;
    }
    case 1: { // Tournoi Binaire
        slotId = selectionEmplacementTournoiBinaire(nodeId);
        break;
    }
    case 2: { // Tournoi Multiple
        int nbTirage = (iter / 100000) + 3;
        if (customParam.size() > 0) {
            for (std::vector<double>& param : customParam) {
                if (param.size() > 0) {
                    if (param[0] == 0) {
                        if (param[1] == 0) {
                            nbTirage = param[2];
                        }
                        else if (param[1] == 1) {
                            nbTirage = ((100 - t) / param[2]) + 1;
                        }
                        else if (param[1] == 2) {
                            nbTirage = (iter / 100000) + param[2];
                        }
                        else if (param[1] == 3) {
                            nbTirage = (iter / param[2]) + 1;
                        }
                    }
                }
            }
        }
        slotId = selectionEmplacementTournoiMultiple(nbTirage, nodeId);
        break;
    }
    case 3: { // Voisin Triangulation
        int profondeur = (iter / 100000) + 10;
        if (customParam.size() > 0) {
            for (std::vector<double>& param : customParam) {
                if (param.size() > 0) {
                    if (param[0] == 7) {
                        profondeur = param[1];
                    }
                    else if (param[0] == 8) {
                        profondeur = (iter / 100000) + param[1];
                    }
                    else if (param[0] == 9) {
                        profondeur = ((1381545-iter) / 100000) + param[1];
                    }
                }
            }
        }
        slotId = selectionEmplacementTriangulation(nodeId,profondeur);
        break;
    }
    }
    return slotId;
}

std::pair<double,double> Graphe::selectionEmplacementReel(int modeEmplacement, int nodeId, int t, std::vector<std::vector<double>>& customParam, int iter) {
    std::pair<double,double> randCoord;
    switch (modeEmplacement) {
    case 0: {
        tirageCoordReel(randCoord);
        return randCoord;
    }
    case 1: {
        std::pair<double,double> nodeCoord({_noeuds[nodeId]._xreel,_noeuds[nodeId]._yreel});
        tirageCoordReel(randCoord);
        double closestDistance = distanceReel(randCoord,nodeCoord);
        std::pair<double,double> randCoordSecond;
        tirageCoordReel(randCoordSecond);
        if (closestDistance >= distanceReel(randCoordSecond,nodeCoord)) { return randCoordSecond; }
        return randCoord;
    }
    case 2: {
        int nbTirage = (iter / 100000) + 3;
        if (customParam.size() > 0) {
            for (std::vector<double>& param : customParam) {
                if (param.size() > 0) {
                    if (param[0] == 0) {
                        if (param[1] == 0) {
                            nbTirage = param[2];
                        }
                        else if (param[1] == 1) {
                            nbTirage = ((100 - t) / param[2]) + 1;
                        }
                        else if (param[1] == 2) {
                            nbTirage = (iter / 100000) + param[2];
                        }
                        else if (param[1] == 3) {
                            nbTirage = (iter / param[2]) + 1;
                        }
                    }
                }
            }
        }
        std::pair<double,double> nodeCoord({_noeuds[nodeId]._xreel,_noeuds[nodeId]._yreel});
        tirageCoordReel(randCoord);
        double closestDistance = distanceReel(randCoord,nodeCoord);
        std::pair<double,double> randCoordSecond;
        for (int tir=0;tir<nbTirage-1;tir++) {
            tirageCoordReel(randCoordSecond);
            double distance = distanceReel(randCoordSecond,nodeCoord);
            if (closestDistance >= distance) {
                randCoord = randCoordSecond;
                closestDistance = distance;
            }
        }
        return randCoord;
        break;
    }
    case 4: { // Boite autour du noeud
        double sizeX = boiteXSizeDepart - (diffXBoiteIter * iter);
        double sizeY = boiteYSizeDepart - (diffYBoiteIter * iter);
        randCoord.first =  std::min(std::max(_noeuds[nodeId]._xreel + generateDoubleRand(sizeX*2)-sizeX,0.0),(double)gridWidth);
        randCoord.second =  std::min(std::max(_noeuds[nodeId]._yreel + generateDoubleRand(sizeY*2)-sizeY,0.0),(double)gridHeight);
    }
    }
    return randCoord;
}

// Calcule le delay a appliquer lors du recuit si besoin.
void Graphe::calculDelaiRefroidissement(int& delay, std::vector<std::vector<double>>& customParam, int iter) {
    if (customParam.size() > 0) {
        for (std::vector<double>& param : customParam) {
            if (param.size() > 0) {
                if (param[0] == 3) {
                    if (param[1] == 0) {
                        delay = param[2];
                    }
                    else if (param[1] == 1) {
                        delay = ceil((double)_noeuds.size() / param[2]) + 1;
                    }
                    else if (param[1] == 2) {
                        delay = (iter / 100000) + param[2];
                    }
                    else if (param[1] == 3) {
                        delay = ((1381545-iter) / 100000) + param[2];
                    }
                }
            }
        }
    }
    else if (delay == -1) {
        delay = std::min((int)ceil((double)_noeuds.size() / 20.0) + 1,5);
    }
}

void Graphe::setupSelectionEmplacement(int modeEmplacement, double t, double cool, double seuil, std::vector<std::vector<double>>& customParam) {
    if (modeEmplacement == 4) {
        std::pair<double,double> sizeGraphe = sizeOfGraphe();
        int nbIterationRecuit = nombreIterationRecuit(t, cool, seuil) ;
        double coeffDepart = 0.75, coeffArrivee = 0.25;
        if (customParam.size() > 0) {
            for (std::vector<double>& param : customParam) {
                if (param.size() > 0) {
                    if (param[0] == 6) {
                        coeffDepart = param[1];
                    }
                    else if (param[0] == 7) {
                        coeffArrivee = param[1];
                    }
                }
            }
        }
        boiteXSizeDepart = sizeGraphe.first*coeffDepart;
        boiteYSizeDepart = sizeGraphe.second*coeffDepart;
        diffXBoiteIter = (boiteXSizeDepart - coeffArrivee*(boiteXSizeDepart/10.0))/(double)nbIterationRecuit;
        diffYBoiteIter = (boiteYSizeDepart - coeffArrivee*(boiteYSizeDepart/10.0))/(double)nbIterationRecuit;
    }
}

long Graphe::calculScoreNodeMethode(int nodeId, int idSwappedNode, bool swapped, bool useGrille, bool useScore, bool useReel) {
    if (!useReel) {
        if (useGrille) {
            if (swapped) { return getScoreCroisementNodeGrid(nodeId, idSwappedNode); }
            else { return getScoreCroisementNodeGrid(nodeId);}
        }
        else if (useScore) {
            if (swapped) { return getNodeLinkedScore(nodeId, idSwappedNode); }
            else { return _noeuds[nodeId].score; }
        }
        else { 
            if (swapped) { return getScoreCroisementNode(nodeId,idSwappedNode); }
            else { return getScoreCroisementNode(nodeId); }
        }
    }
    else {
        if (useGrille) { return getScoreCroisementNodeGridReel(nodeId); }
        else if (useScore) { return _noeuds[nodeId].score; }
        else { return getScoreCroisementNodeReel(nodeId); }
    }
}

long Graphe::calculScoreNodeMethodeThread(int nodeId, int idSwappedNode, bool swapped, bool useGrille, bool useScore, bool useReel, bool isFirstThread) {
    if (!useReel) {
        if (useGrille) {
            //if (swapped) { return getScoreCroisementNodeGrid(nodeId, idSwappedNode); }
            //else { return getScoreCroisementNodeGrid(nodeId);}
        }
        else if (useScore) {
            //if (swapped) { return getNodeLinkedScore(nodeId, idSwappedNode); }
            //else { return _noeuds[nodeId].score; }
        }
        else { 
            //if (swapped) { return getScoreCroisementNode(nodeId,idSwappedNode); }
            //else { return getScoreCroisementNode(nodeId); }
        }
    }
    else {
        if (useGrille) { return getScoreCroisementNodeGridReelThread(nodeId,isFirstThread); }
        else if (useScore) { return _noeuds[nodeId].score; }
        //else { return getScoreCroisementNodeReel(nodeId); }
    }
    return -1; // A supprimer
}

int Graphe::calculImprove(int nodeId,int slotId,bool& swapped,int& idSwappedNode,bool useGrille,bool useScore) {
    long scoreNode;
    if (!_emplacements[slotId].estDisponible()) {
        swapped = true;
        idSwappedNode = _emplacements[slotId]._noeud->getId();
        scoreNode = calculScoreNodeMethode(nodeId,idSwappedNode,true,useGrille,useScore);
        scoreNode += calculScoreNodeMethode(idSwappedNode,-1,false,useGrille,useScore);
        _noeuds[nodeId].swap(&_emplacements[slotId]);
        if (useScore) {
            changeUpdateValue(nodeId);
            changeUpdateValue(idSwappedNode);
            updateNodeScore(idSwappedNode);
        }
    }
    else {
        scoreNode = calculScoreNodeMethode(nodeId,-1,false,useGrille,useScore);
        _noeuds[nodeId].setEmplacement(&_emplacements[slotId]);
        if (useScore) { changeUpdateValue(nodeId); }
    }
    if (useScore) { updateNodeScore(nodeId); }
    if (useGrille) { recalcNodeCellule(nodeId); }
    long newScoreNode;
    if (swapped) {
        if (useGrille) { recalcNodeCellule(idSwappedNode); }
        newScoreNode = calculScoreNodeMethode(nodeId,idSwappedNode,true,useGrille,useScore);
        newScoreNode += calculScoreNodeMethode(idSwappedNode,-1,false,useGrille,useScore);
    }
    else {
        newScoreNode = calculScoreNodeMethode(nodeId,-1,false,useGrille,useScore);
    }
    return newScoreNode - scoreNode;
}

int Graphe::calculImproveReel(int nodeId,std::pair<double,double>& randCoord, bool useGrille,bool useScore) {
    long scoreNode = calculScoreNodeMethode(nodeId,-1,false,useGrille,useScore,true);
    _noeuds[nodeId].setCoordReel(randCoord);
    if (useScore) { changeUpdateValue(nodeId); }
    if (useScore) { updateNodeScore(nodeId); }
    if (useGrille) { recalcNodeCelluleReel(nodeId); }
    long newScoreNode = calculScoreNodeMethode(nodeId,-1,false,useGrille,useScore,true);
    return newScoreNode - scoreNode;
}

int Graphe::calculImproveReelThread(int nodeId,std::pair<double,double>& randCoord, bool useGrille,bool useScore) {
    int newTempNodeId = creationNoeudTemporaire(nodeId, randCoord);
    long scoreNode, newScoreNode;
#pragma omp parallel num_threads(2) 
    {
        #pragma omp single nowait
        scoreNode = calculScoreNodeMethodeThread(nodeId,-1,false,useGrille,useScore,true,true);
        #pragma omp single nowait
        newScoreNode = calculScoreNodeMethodeThread(newTempNodeId,-1,false,useGrille,useScore,true,false);
    }
    return newScoreNode - scoreNode;
}

int Graphe::calculImproveReelThreadPool(int nodeId,std::pair<double,double>& randCoord, bool useGrille,bool useScore) {
    thread_tempNodeId = creationNoeudTemporaire(nodeId, randCoord);
    long scoreNode;
    #pragma omp barrier
    scoreNode = calculScoreNodeMethodeThread(nodeId,-1,false,useGrille,useScore,true,true);
    #pragma omp barrier
    return thread_tempNewScore - scoreNode;
}

void Graphe::applyRerecuitCustomParam(double& t,double& cool,double& coolt,double& seuil,std::vector<std::vector<double>>& customParam) {
    if (customParam.size() > 0) {
        for (std::vector<double>& param : customParam) {
            if (param.size() > 0) {
                if (param[0] == 8) { t = param[1]; }
                else if (param[0] == 9) { cool = param[1]; }
                else if (param[0] == 13) { coolt = param[1]; }
                else if (param[0] == 14) { seuil = param[1]; }
            }
        }
    }
}

void Graphe::applyRecuitCustomParam(double& coeffImprove,std::vector<std::vector<double>>& customParam) {
    if (customParam.size() > 0) {
        for (std::vector<double>& param : customParam) {
            if (param.size() > 0) {
                if (param[0] == 12) { coeffImprove = param[1]; }
            }
        }
    }
}

void Graphe::saveBestResultRecuit(std::vector<int>& bestResultVector, Graphe& bestResultGraphe, bool useScore, bool useGrille) {
    if (useScore) { bestResultGraphe.copyFromGraphe(*this); }
    else if (useGrille) { bestResultVector = saveCopy(); }
    else { bestResultVector = saveCopy(); }
}

void Graphe::saveBestResultRecuitReel(std::vector<std::pair<double,double>>& bestResultVector) {
    bestResultVector.clear();
    for (int i=0;i<_noeuds.size();i++) {
        bestResultVector.push_back({_noeuds[i]._xreel,_noeuds[i]._yreel});
    }
}

void Graphe::loadBestResultRecuit(std::vector<int>& bestResultVector, Graphe& bestResultGraphe, long bestNbCroisement, bool useScore, bool useGrille) {
    nombreCroisement = bestNbCroisement;
    if (useScore) { copyFromGraphe(bestResultGraphe); }
    else if (useGrille) { loadCopy(bestResultVector); }
    else { loadCopy(bestResultVector); }
}

void Graphe::loadBestResultRecuitReel(std::vector<std::pair<double,double>>& bestResultVector, long bestNbCroisement) {
    nombreCroisement = bestNbCroisement;
    for (int i=0;i<_noeuds.size();i++) {
        _noeuds[i].setCoordReel(bestResultVector[i]);
    }
}

void Graphe::updateGraphDataRecuit(bool useScore, bool useGrille) {
    isNombreCroisementUpdated = true;
    if (useScore) { isNodeScoreUpdated = true; }
    else { isNodeScoreUpdated = false; }
    if (useScore) { isIntersectionVectorUpdated = true; }
    else { isIntersectionVectorUpdated = false; }
}

void Graphe::stepRecuitSimule(double& t, int& nbCrois, double cool, int modeNoeud, int modeEmplacement, std::vector<std::vector<double>> customParam) {
    std::cout << "Not Yet Updated.\n";
}

// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection
// Met à jour la variable nombreCroisement du graphe.
// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple, 3=Triangulation(Emplacement uniquement)
// Par defaut utilise la grille et le Tournoi Multiple sur les Emplacements.
void Graphe::recuitSimule(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam, double cool, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    auto bestEnd = start; auto end = start; // Timer pour le meilleur resultat trouvé et total
    std::vector<int> bestResultVector; Graphe bestResultGraphe; // Sauvegarde du meilleur graphe.
    saveBestResultRecuit(bestResultVector,bestResultGraphe,useScore,useGrille);
    long nbCroisement;
    if (isNombreCroisementUpdated) { nbCroisement = nombreCroisement; }
    else { nbCroisement = getNbCroisement(); }
    long bestCroisement = nbCroisement;
    calculDelaiRefroidissement(delay,customParam,0);
    #if defined(DEBUG_GRAPHE)
        std::cout << "Nb Croisement avant recuit: " << nbCroisement << std::endl;
    #endif
    int nodeId, slotId, idSwappedNode, improve;
    bool swapped;
    Emplacement* oldEmplacement;
    std::chrono::duration<double> secondsTotal = end - start;
    for (int iter = 0; t > seuil && nbCroisement > 0 && ((secondsTotal.count() < 3600)||(noLimit)); iter++) {
        calculDelaiRefroidissement(delay,customParam,iter); // Utile uniquement si customParam[0]==3 et customParam[1]==2 ou 3
        for (int del = 0; del < delay; del++) {
            nodeId = selectionNoeud(modeNoeud, t);
            slotId = selectionEmplacement(modeEmplacement, nodeId, t,customParam,iter);
            oldEmplacement = _noeuds[nodeId].getEmplacement();
            swapped = false;
            idSwappedNode = -1;
            improve = calculImprove(nodeId,slotId,swapped,idSwappedNode,useGrille,useScore);
            if (improve < 0) {
                nbCroisement += improve;
                if (nbCroisement < bestCroisement) {
                    bestCroisement = nbCroisement;
                    saveBestResultRecuit(bestResultVector,bestResultGraphe,useScore,useGrille);
                    bestEnd = std::chrono::system_clock::now();
                    #if defined(DEBUG_GRAPHE_PROGRESS)
                        std::cout << "Meilleur Recuit: " << bestCroisement << " Iteration: " << iter << " t: " << t << std::endl;
                    #endif
                }
            }
            else {
                double randDouble = generateDoubleRand(1.0);
                if (randDouble >= exp(-improve / t)) {
                    if (useScore) { changeUpdateValue(nodeId); }
                    if (swapped) {
                        if (useScore) { changeUpdateValue(idSwappedNode); }
                        _noeuds[nodeId].swap(oldEmplacement);
                        if (useScore) { updateNodeScore(idSwappedNode); }
                        if (useGrille) { recalcNodeCellule(idSwappedNode); }
                    }
                    else {
                        _noeuds[nodeId].setEmplacement(oldEmplacement);
                    }
                    if (useScore) { updateNodeScore(nodeId); }
                    if (useGrille) { recalcNodeCellule(nodeId); }
                }
                else {
                    nbCroisement += improve;
                }
            }
        }
        t *= cool;
        end = std::chrono::system_clock::now();
        secondsTotal = end - start;
    }
    loadBestResultRecuit(bestResultVector,bestResultGraphe,bestCroisement,useScore,useGrille);
    updateGraphDataRecuit(useScore,useGrille);
    std::chrono::duration<double> secondsBest = bestEnd - start;
    timeBest = secondsBest.count();
    #if defined(DEBUG_GRAPHE)
        std::cout << "Meilleur resultat du recuit: " << bestCroisement << std::endl;
    #endif
}

// Applique le recuit simulé plusieurs fois
// Met a jour le nombre de croisement du graphe.
void Graphe::rerecuitSimule(double &timeBest,int &nombreRecuit,std::chrono::time_point<std::chrono::system_clock> start,std::vector<std::vector<double>> customParam, int iter, double cool, double coolt, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    auto end = start;
    applyRerecuitCustomParam(t,cool,coolt,seuil,customParam);
    nombreRecuit= 0;
    #if defined(DEBUG_GRAPHE)
        std::cout << "Starting Rerecuit " << iter << " iterations." << std::endl;
    #endif
    int numberOfNoUpgrade = 0, maxIter = 10;
    if (iter != -1) { maxIter = iter; }
    long lastCroisement;
    if (isNombreCroisementUpdated) { lastCroisement = nombreCroisement; }
    else { lastCroisement = getNbCroisementConst(); }
    int i=1;
    double recuitTimeBest = -1.0;
    auto totalEnd = std::chrono::system_clock::now();
	std::chrono::duration<double> secondsTotalExec = totalEnd - start;
    while ((numberOfNoUpgrade < maxIter)&&(noLimit||secondsTotalExec.count() < 3600)) {
        if (useGrille) { if (i>1) { reinitGrille(); } }
        #if defined(DEBUG_GRAPHE)
            std::cout << "Starting Recuit Number: " << i << " t: " << t << " cool " << cool << " NumNoUp: " << numberOfNoUpgrade << std::endl;
        #endif
        nombreRecuit++;
        recuitSimule(recuitTimeBest,start,customParam, cool, t, seuil, delay, modeNoeud, modeEmplacement, useGrille, useScore, noLimit);
        t *= coolt;
        if (iter != -1) { numberOfNoUpgrade++; }
        else {
            long newCroisement;
            if (isNombreCroisementUpdated) { newCroisement = nombreCroisement; }
            else { newCroisement = getNbCroisementConst(); }
            if (newCroisement == lastCroisement) {
                numberOfNoUpgrade++;
            }
            else {
                lastCroisement = newCroisement;
                numberOfNoUpgrade = 0;
            }
        }
        i++;
        totalEnd = std::chrono::system_clock::now();
        secondsTotalExec = totalEnd - start;
    }
    timeBest = recuitTimeBest;
}

// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection
// Met à jour la variable nombreCroisement du graphe.
// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple, 3=Triangulation(Emplacement uniquement)
// Par defaut utilise la grille et le Tournoi Multiple sur les Emplacements.
void Graphe::recuitSimuleReel(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam, double cool, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    auto bestEnd = start; auto end = start; // Timer pour le meilleur resultat trouvé et total
    std::vector<std::pair<double,double>> bestResultVector;
    saveBestResultRecuitReel(bestResultVector);
    long nbCroisement;
    if (isNombreCroisementUpdated) { nbCroisement = nombreCroisement; }
    else { nbCroisement = getNbCroisementReel(); }
    long bestCroisement = nbCroisement;
    double coeffImprove = 1.0;
    applyRecuitCustomParam(coeffImprove,customParam);
    calculDelaiRefroidissement(delay,customParam,0);
    setupSelectionEmplacement(modeEmplacement,t,cool,seuil,customParam);
    #if defined(DEBUG_GRAPHE)
        std::cout << "Nb Croisement avant recuit: " << nbCroisement << std::endl;
    #endif
    int nodeId, idSwappedNode, improve;
    std::pair<double,double> randCoord;
    std::chrono::duration<double> secondsTotal = end - start;
    for (int iter = 0; t > seuil && nbCroisement > 0 && ((secondsTotal.count() < 3600)||(noLimit)); iter++) {
        calculDelaiRefroidissement(delay,customParam,iter); // Utile uniquement si customParam[0]==3 et customParam[1]==2 ou 3
        for (int del = 0; del < delay; del++) {
            nodeId = selectionNoeud(modeNoeud, t);
            std::pair<double,double> oldCoord({_noeuds[nodeId]._xreel,_noeuds[nodeId]._yreel});
            randCoord = selectionEmplacementReel(modeEmplacement, nodeId, t,customParam,iter);
            //double TMPDISTANCE = distanceReelSqrt(randCoord,oldCoord);
            //recuitDistanceAll.push_back(TMPDISTANCE);
            improve = calculImproveReel(nodeId,randCoord,useGrille,useScore);
            if (improve <= 0) {
                nbCroisement += improve;
                //recuitDistanceUpgrade.push_back(make_pair(iter,TMPDISTANCE));
                if (nbCroisement < bestCroisement) {
                    //recuitDistanceUpgradeGlobal.push_back(make_pair(iter,TMPDISTANCE));
                    bestCroisement = nbCroisement;
                    saveBestResultRecuitReel(bestResultVector);
                    bestEnd = std::chrono::system_clock::now();
                    #if defined(DEBUG_GRAPHE_PROGRESS)
                        std::cout << "Meilleur Recuit: " << bestCroisement << " Iteration: " << iter << " t: " << t << std::endl;
                    #endif
                }
            }
            else {
                double randDouble = generateDoubleRand(1.0);
                //double TMPDISTANCE = exp(-improve / t);
                //recuitDistanceUpgrade.push_back(make_pair(iter,TMPDISTANCE));
                double valImprove = exp(-improve / t) * coeffImprove;
                if (randDouble >= valImprove) {
                    if (useScore) { changeUpdateValue(nodeId); }
                    else { _noeuds[nodeId].setCoordReel(oldCoord); }
                    if (useScore) { updateNodeScore(nodeId); }
                    if (useGrille) { recalcNodeCelluleReel(nodeId); }
                }
                else {
                    nbCroisement += improve;
                }
            }
        }
        t *= cool;
        end = std::chrono::system_clock::now();
        secondsTotal = end - start;
    }
    loadBestResultRecuitReel(bestResultVector,bestCroisement);
    updateGraphDataRecuit(useScore,useGrille);
    std::chrono::duration<double> secondsBest = bestEnd - start;
    timeBest = secondsBest.count();
    #if defined(DEBUG_GRAPHE)
        std::cout << "Meilleur resultat du recuit: " << bestCroisement << std::endl;
    #endif
}

// Applique le recuit simulé plusieurs fois
// Met a jour le nombre de croisement du graphe.
void Graphe::rerecuitSimuleReel(double &timeBest,int &nombreRecuit,std::chrono::time_point<std::chrono::system_clock> start,std::vector<std::vector<double>> customParam, int iter, double cool, double coolt, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    auto end = start;
    applyRerecuitCustomParam(t,cool,coolt,seuil,customParam);
    nombreRecuit= 0;
    #if defined(DEBUG_GRAPHE)
        std::cout << "Starting Rerecuit " << iter << " iterations." << std::endl;
    #endif
    int numberOfNoUpgrade = 0, maxIter = 10;
    if (iter != -1) { maxIter = iter; }
    long lastCroisement;
    if (isNombreCroisementUpdated) { lastCroisement = nombreCroisement; }
    else { lastCroisement = getNbCroisementReelConst(); }
    int i=1;
    double recuitTimeBest = -1.0;
    auto totalEnd = std::chrono::system_clock::now();
	std::chrono::duration<double> secondsTotalExec = totalEnd - start;
    while ((numberOfNoUpgrade < maxIter)&&(noLimit||secondsTotalExec.count() < 3600)) {
        if (useGrille) { if (i>1) { reinitGrilleReel(); } }
        #if defined(DEBUG_GRAPHE)
            std::cout << "Starting Recuit Number: " << i << " t: " << t << " cool " << cool << " NumNoUp: " << numberOfNoUpgrade << std::endl;
        #endif
        nombreRecuit++;
        recuitSimuleReel(recuitTimeBest,start,customParam, cool, t, seuil, delay, modeNoeud, modeEmplacement, useGrille, useScore,noLimit);
        t *= coolt;
        if (iter != -1) { numberOfNoUpgrade++; }
        else {
            long newCroisement;
            if (isNombreCroisementUpdated) { newCroisement = nombreCroisement; }
            else { newCroisement = getNbCroisementReelConst(); }
            if (newCroisement == lastCroisement) {
                numberOfNoUpgrade++;
            }
            else {
                lastCroisement = newCroisement;
                numberOfNoUpgrade = 0;
                timeBest = recuitTimeBest;
            }
        }
        i++;
        totalEnd = std::chrono::system_clock::now();
        secondsTotalExec = totalEnd - start;
    }
}

// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection
// Met à jour la variable nombreCroisement du graphe.
// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple, 3=Triangulation(Emplacement uniquement)
// Par defaut utilise la grille et le Tournoi Multiple sur les Emplacements.
void Graphe::recuitSimuleReelThread(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam, double cool, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    auto bestEnd = start; auto end = start; // Timer pour le meilleur resultat trouvé et total
    std::vector<std::pair<double,double>> bestResultVector;
    saveBestResultRecuitReel(bestResultVector);
    long nbCroisement;
    if (isNombreCroisementUpdated) { nbCroisement = nombreCroisement; }
    else { nbCroisement = getNbCroisementReel(); }
    long bestCroisement = nbCroisement;
    double coeffImprove = 1.0;
    applyRecuitCustomParam(coeffImprove,customParam);
    calculDelaiRefroidissement(delay,customParam,0);
    setupSelectionEmplacement(modeEmplacement,t,cool,seuil,customParam);
    #if defined(DEBUG_GRAPHE)
        std::cout << "Nb Croisement avant recuit: " << nbCroisement << std::endl;
    #endif
    int nodeId, idSwappedNode, improve;
    std::pair<double,double> randCoord;
    std::chrono::duration<double> secondsTotal = end - start;
    for (int iter = 0; t > seuil && nbCroisement > 0 && ((secondsTotal.count() < 3600)||(noLimit)); iter++) {
        calculDelaiRefroidissement(delay,customParam,iter); // Utile uniquement si customParam[0]==3 et customParam[1]==2 ou 3
        for (int del = 0; del < delay; del++) {
            nodeId = selectionNoeud(modeNoeud, t);
            randCoord = selectionEmplacementReel(modeEmplacement, nodeId, t,customParam,iter);
            improve = calculImproveReelThread(nodeId,randCoord,useGrille,useScore);
            if (improve <= 0) {
                nbCroisement += improve;
                replaceNoeudTemporaire(nodeId);
                if (nbCroisement < bestCroisement) {
                    bestCroisement = nbCroisement;
                    saveBestResultRecuitReel(bestResultVector);
                    bestEnd = std::chrono::system_clock::now();
                    #if defined(DEBUG_GRAPHE_PROGRESS)
                        std::cout << "Meilleur Recuit: " << bestCroisement << " Iteration: " << iter << " t: " << t << std::endl;
                    #endif
                }
            }
            else {
                double randDouble = generateDoubleRand(1.0);
                double valImprove = exp(-improve / t) * coeffImprove;
                if (randDouble >= valImprove) {
                    supprimerNoeudTemporaire(nodeId);
                }
                else {
                    nbCroisement += improve;
                    replaceNoeudTemporaire(nodeId);
                }
            }
        }
        t *= cool;
        end = std::chrono::system_clock::now();
        secondsTotal = end - start;
    }
    loadBestResultRecuitReel(bestResultVector,bestCroisement);
    updateGraphDataRecuit(useScore,useGrille);
    std::chrono::duration<double> secondsBest = bestEnd - start;
    timeBest = secondsBest.count();
    #if defined(DEBUG_GRAPHE)
        std::cout << "Meilleur resultat du recuit: " << bestCroisement << std::endl;
    #endif
}

void Graphe::recuitSimuleReelThreadPool(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam, double cool, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    int tid;
    thread_IsRecuitFinished = false;
    #pragma omp parallel num_threads(2) private(tid)
    {
        tid = omp_get_thread_num();
        if (tid == 0) {
            auto bestEnd = start; auto end = start; // Timer pour le meilleur resultat trouvé et total
            std::vector<std::pair<double,double>> bestResultVector;
            saveBestResultRecuitReel(bestResultVector);
            long nbCroisement;
            if (isNombreCroisementUpdated) { nbCroisement = nombreCroisement; }
            else { nbCroisement = getNbCroisementReel(); }
            long bestCroisement = nbCroisement;
            double coeffImprove = 1.0;
            applyRecuitCustomParam(coeffImprove,customParam);
            calculDelaiRefroidissement(delay,customParam,0);
            setupSelectionEmplacement(modeEmplacement,t,cool,seuil,customParam);
            #if defined(DEBUG_GRAPHE)
                std::cout << "Nb Croisement avant recuit: " << nbCroisement << std::endl;
            #endif
            int nodeId, idSwappedNode, improve;
            std::pair<double,double> randCoord;
            std::chrono::duration<double> secondsTotal = end - start;
            for (int iter = 0; t > seuil && nbCroisement > 0 && ((secondsTotal.count() < 3600)||(noLimit)); iter++) {
                calculDelaiRefroidissement(delay,customParam,iter); // Utile uniquement si customParam[0]==3 et customParam[1]==2 ou 3
                for (int del = 0; del < delay; del++) {
                    nodeId = selectionNoeud(modeNoeud, t);
                    randCoord = selectionEmplacementReel(modeEmplacement, nodeId, t,customParam,iter);
                    improve = calculImproveReelThreadPool(nodeId,randCoord,useGrille,useScore);
                    if (improve <= 0) {
                        nbCroisement += improve;
                        replaceNoeudTemporaire(nodeId);
                        if (nbCroisement < bestCroisement) {
                            bestCroisement = nbCroisement;
                            saveBestResultRecuitReel(bestResultVector);
                            bestEnd = std::chrono::system_clock::now();
                            #if defined(DEBUG_GRAPHE_PROGRESS)
                                std::cout << "Meilleur Recuit: " << bestCroisement << " Iteration: " << iter << " t: " << t << std::endl;
                            #endif
                        }
                    }
                    else {
                        double randDouble = generateDoubleRand(1.0);
                        double valImprove = exp(-improve / t) * coeffImprove;
                        if (randDouble >= valImprove) {
                            supprimerNoeudTemporaire(nodeId);
                        }
                        else {
                            nbCroisement += improve;
                            replaceNoeudTemporaire(nodeId);
                        }
                    }
                }
                t *= cool;
                end = std::chrono::system_clock::now();
                secondsTotal = end - start;
            }
            loadBestResultRecuitReel(bestResultVector,bestCroisement);
            updateGraphDataRecuit(useScore,useGrille);
            std::chrono::duration<double> secondsBest = bestEnd - start;
            timeBest = secondsBest.count();
            #if defined(DEBUG_GRAPHE)
                std::cout << "Meilleur resultat du recuit: " << bestCroisement << std::endl;
            #endif
            std::cout << "Temps passe: " << tempsPasseTmp << std::endl;
            thread_IsRecuitFinished = true;
            #pragma omp barrier
        }
        else {
            while (true) {
                #pragma omp barrier
                if (thread_IsRecuitFinished) { break; }
                thread_tempNewScore = calculScoreNodeMethodeThread(thread_tempNodeId,-1,false,useGrille,useScore,true,false);
                #pragma omp barrier
            }
        }
    }
}

// Lance l'algorithme de recuit simulé sur le graphe pour minimiser le nombre d'intersection
// Met à jour la variable nombreCroisement du graphe.
// modeNoeud et modeEMplacement sont le mode de sélection de noeud et d'emplacement, 0=Aléatoire, 1=TournoiBinaire, 2=TournoiMultiple, 3=Triangulation(Emplacement uniquement)
// Par defaut utilise la grille et le Tournoi Multiple sur les Emplacements.
void Graphe::recuitSimuleReelThreadSelection(double &timeBest, std::chrono::time_point<std::chrono::system_clock> start, std::vector<std::vector<double>> customParam, double cool, double t, double seuil, int delay, int modeNoeud, int modeEmplacement, bool useGrille, bool useScore, bool noLimit) {
    int tid;
    thread_IsRecuitFinished = false;
    std::pair<double,double> bestCoord;
    long bestScore;
    int nodeId = -1, iter;
    int maxThread = omp_get_max_threads()-2;
    #pragma omp parallel num_threads(maxThread) private(tid)
    {
        tid = omp_get_thread_num();
        if (tid == 0) {
            auto bestEnd = start; auto end = start; // Timer pour le meilleur resultat trouvé et total
            std::vector<std::pair<double,double>> bestResultVector;
            saveBestResultRecuitReel(bestResultVector);
            long nbCroisement;
            if (isNombreCroisementUpdated) { nbCroisement = nombreCroisement; }
            else { nbCroisement = getNbCroisementReel(); }
            long bestCroisement = nbCroisement;
            double coeffImprove = 1.0;
            int nombreNoeud = (int)_noeuds.size()-1;
            applyRecuitCustomParam(coeffImprove,customParam);
            calculDelaiRefroidissement(delay,customParam,0);
            setupSelectionEmplacement(modeEmplacement,t,cool,seuil,customParam);
            #if defined(DEBUG_GRAPHE)
                std::cout << "Nb Croisement avant recuit: " << nbCroisement << std::endl;
            #endif
            int idSwappedNode, improve;
            std::chrono::duration<double> secondsTotal = end - start;
            for (iter = 0; t > seuil && nbCroisement > 0 && ((secondsTotal.count() < 3600)||(noLimit)); iter++) {
                calculDelaiRefroidissement(delay,customParam,iter); // Utile uniquement si customParam[0]==3 et customParam[1]==2 ou 3
                for (int del = 0; del < delay; del++) {
                    nodeId = selectionNoeud(modeNoeud, t);
                    #pragma omp barrier
                    bestScore = INT_MAX;
                    #pragma omp barrier
                    long scoreBefore = getScoreCroisementNodeGridReelNThread(nodeId,-1);
                    #pragma omp barrier
                    #pragma omp barrier
                    resizeVectorTemporaire(nodeId,maxThread-1);
                    improve = bestScore - scoreBefore;
                    if (improve <= 0) {
                        nbCroisement += improve;
                        if (useScore) { changeUpdateValue(nodeId); }
                        _noeuds[nodeId].setCoordReel(bestCoord);
                        if (useScore) { updateNodeScore(nodeId); }
                        if (useGrille) { recalcNodeCelluleReel(nodeId); }
                        if (nbCroisement < bestCroisement) {
                            bestCroisement = nbCroisement;
                            saveBestResultRecuitReel(bestResultVector);
                            bestEnd = std::chrono::system_clock::now();
                            #if defined(DEBUG_GRAPHE_PROGRESS)
                                std::cout << "Meilleur Recuit: " << bestCroisement << " Iteration: " << iter << " t: " << t << std::endl;
                            #endif
                        }
                    }
                    else {
                        double randDouble = generateDoubleRand(1.0);
                        double valImprove = exp(-improve / t) * coeffImprove;
                        if (randDouble < valImprove) {
                            nbCroisement += improve;
                            if (useScore) { changeUpdateValue(nodeId); }
                            _noeuds[nodeId].setCoordReel(bestCoord);
                            if (useScore) { updateNodeScore(nodeId); }
                            if (useGrille) { recalcNodeCelluleReel(nodeId); }
                        }
                    }
                }
                t *= cool;
                end = std::chrono::system_clock::now();
                secondsTotal = end - start;
            }
            loadBestResultRecuitReel(bestResultVector,bestCroisement);
            updateGraphDataRecuit(useScore,useGrille);
            std::chrono::duration<double> secondsBest = bestEnd - start;
            timeBest = secondsBest.count();
            #if defined(DEBUG_GRAPHE)
                std::cout << "Meilleur resultat du recuit: " << bestCroisement << std::endl;
            #endif
            thread_IsRecuitFinished = true;
            #pragma omp barrier
        }
        else {
            while (true) {
                #pragma omp barrier
                if (thread_IsRecuitFinished) { break; }
                std::pair<double,double> tmpRandCoord = selectionEmplacementReel(modeEmplacement, nodeId, t,customParam,iter);
                int tmpNodeId = creationNoeudTemporaireThread(nodeId, tmpRandCoord, tid);
                #pragma omp barrier
                long tmpScore = getScoreCroisementNodeGridReelNThread(tmpNodeId,tid);
                #pragma omp critical
                {
                    if (tmpScore < bestScore) {
                        bestScore = tmpScore;
                        bestCoord = make_pair(_noeuds[tmpNodeId]._xreel,_noeuds[tmpNodeId]._yreel);
                    }
                }
                #pragma omp barrier
                supprimerNoeudTemporaireThread(nodeId,tid);
                #pragma omp barrier
            }
        }
    }
}

// Applique l'algorithme meilleur deplacement sur le graphe.
// On parcoure tout les noeuds et on teste chaque deplacement possible et on effectue le meilleur s'il ameliore le score. (O(n²*e))
// Met a jour le nombre de croisement du graphe.
void Graphe::bestDeplacement() {
    long nbIntersection;
    if (isNombreCroisementUpdated) {
        nbIntersection = nombreCroisement;
    }
    else {
        nbIntersection = getNbCroisement();
    }
    int bestImprove = 0;
    int indexNoeud = -1, indexEmplacement = -1;
    do {
        bestImprove = 0;
        for (int i = 0; i < _noeuds.size(); i++) {
            for (int j = 0; j < _emplacements.size(); j++) {
                if (_noeuds[i].getEmplacement()->getId() != j) {
                    long score, newScore;
                    int swappedId;
                    bool swapped = false;
                    Emplacement* oldEmplacement = _noeuds[i].getEmplacement();
                    if (_emplacements[j].estDisponible()) {
                        score = getScoreCroisementNode(i);
                        _noeuds[i].setEmplacement(&_emplacements[j]);
                    }
                    else {
                        swappedId = _emplacements[j]._noeud->getId();
                        score = getScoreCroisementNode(i);
                        score += getScoreCroisementNode(swappedId, i);
                        _noeuds[i].swap(&_emplacements[j]);
                        swapped = true;
                    }
                    if (!swapped) {
                        newScore = getScoreCroisementNode(i);
                        _noeuds[i].setEmplacement(oldEmplacement);
                    }
                    else {
                        newScore = getScoreCroisementNode(i);
                        newScore += getScoreCroisementNode(swappedId, i);
                        _noeuds[i].swap(oldEmplacement);
                    }
                    int improve = newScore - score;
                    if (improve < bestImprove) {
                        bestImprove = improve;
                        indexNoeud = i;
                        indexEmplacement = j;
                    }
                }
            }
        }
        if (bestImprove < 0) {
            if (_emplacements[indexEmplacement].estDisponible()) {
                _noeuds[indexNoeud].setEmplacement(&_emplacements[indexEmplacement]);
            }
            else {
                _noeuds[indexNoeud].swap(&_emplacements[indexEmplacement]);
            }
            nbIntersection += bestImprove;
        }
    } while (bestImprove < 0);
    nombreCroisement = nbIntersection;
    isNombreCroisementUpdated = true;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
    #if defined(DEBUG_GRAPHE)
        std::cout << "Meilleur resultat de l'algo meilleur deplacement: " << nbIntersection << std::endl;
    #endif
}