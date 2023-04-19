#include "graphe.hpp"
#include <chrono>
#include <iostream>
#include <algorithm>

// Applique un algorithme genetique sur une population, la moitié de la population est remplacée à chaque génération en croisant 2 parents appartenant aux 50% des meilleurs.
// useRecuit indique si on applique un court recuit simulé à chaque nouvel enfant créé
// useRand indique si l'on doit utiliser l'algorithme aléatoire plutôt que le glouton pour le replacage des noeuds lors du croisement
// modeCroisement indique quel algorithme de croisement utiliser 0=Voisinage, 1=HalfParent, 2=Aléatoire, 3=VoisinageV2
void Graphe::grapheGenetique(double &timeBest, int &bestIteration, int &lastIteration, int population, int maxIteration, const std::string& nomGraphe, const std::string& nomSlot, bool useRecuit, bool useRand, int modeCroisement) {
    auto start = std::chrono::system_clock::now(); auto end = start;
    std::vector<int> methodeNonScore = { 0, 1, 2, 5 }; std::vector<int> methodeGrille = { 6 };
    bool modeNonScore = isInVector(methodeNonScore, modeCroisement);
    bool useGrille = isInVector(methodeGrille,modeCroisement);
    bool DEBUG_GENETIQUE = false, PRINT_RESULT = true;;
    std::vector<Graphe> graphes;
    graphes.resize(population);
    graphes[0].setupGraphe(nomGraphe,nomSlot);
    for (int i = 0; i < population; ++i) {
        graphes[i].nomGraphe = "Graphe" + std::to_string(i);
        if (i > 0) { graphes[i].copyFromGrapheGenetique(graphes[0]); }
        graphes[i].placementAleatoire();
        if (modeNonScore) {
            graphes[i].getNbCroisement();
        }
        else {
            graphes[i].initGraphAndNodeScoresAndCrossings();
        }
        if (useGrille) { graphes[i].initGrille(); graphes[i].registerSlotsAndEdgesInGrid(); }
    }
    std::vector<int> sortedEmpId; // Vecteur pour le mode half parent
    if (modeCroisement == 1) { getSortedEmpVecFromGraphe(sortedEmpId,graphes[0]); }
    sort(graphes.begin(), graphes.end());
    int currentIteration = 0; bestIteration = 0;
    long bestCrossingResult = graphes[0].nombreCroisement;
    if (PRINT_RESULT) {
        std::cout << bestCrossingResult << " Meilleur debut genetique\n[";
        for (int i = 0; i<10;i++) { std::cout << graphes[i].nombreCroisement << " "; }
        std::cout << "]" << std::endl;
    }
    if (DEBUG_GENETIQUE) {
        for (int i = 0; i<graphes.size();i++) { graphes[i].debugEverything(); }
    }
    if (PRINT_RESULT) { std::cout << "Debut Croisement Genetique." << std::endl; }
    bool noChange = false;
    while (currentIteration < maxIteration && bestCrossingResult>0 && !noChange) {
        int numberOfNoChange = 0;
        for (int i = population/2; i < population; ++i) {
            graphes[i].clearNodeEmplacement();
            int grapheID1, grapheID2;
            grapheID1 = generateRand(population/2 - 1);
            grapheID2 = generateRand(population/2 - 1);
            while (grapheID2 == grapheID1) { grapheID2 = generateRand(population/2 - 1); }
            bool result;
            if (modeCroisement == 0) result = graphes[i].croisementVoisinageFrom(graphes[grapheID1], graphes[grapheID2], useRand);
            else if (modeCroisement == 1) result = graphes[i].croisementHalfParent(graphes[grapheID1], graphes[grapheID2], sortedEmpId, useRand);
            else if (modeCroisement == 2) result = graphes[i].croisementAleatoire(graphes[grapheID1], graphes[grapheID2], useRand);
            else if (modeCroisement == 3) result = graphes[i].croisementVoisinageScore(graphes[grapheID1], graphes[grapheID2], useRand);
            else if (modeCroisement == 4) result = graphes[i].croisementBestOfBoth(graphes[grapheID1], graphes[grapheID2], useRand);
            else if (modeCroisement == 5) result = graphes[i].croisementEnfantScore(graphes[grapheID1], graphes[grapheID2], useRand);
            else if (modeCroisement == 6) result = graphes[i].croisementEnfantScoreGrille(graphes[grapheID1], graphes[grapheID2], useRand);
            if (!result) { numberOfNoChange++; }
            if (useRecuit) { // Le recuit met le nombre de croisement à jour.
                double tb;
                if (useGrille) graphes[i].recuitSimule(tb,start,{},0.99);
                else graphes[i].recuitSimule(tb,start,{},0.99);
            }
            else if (!graphes[i].isNombreCroisementUpdated){ // Si le nombre de croisement n'est pas à jour, on le recalcule.
                graphes[i].getNbCroisement();
            }
            if (graphes[i].nombreCroisement < bestCrossingResult) {
                end = std::chrono::system_clock::now();
                bestCrossingResult = graphes[i].nombreCroisement;
                bestIteration = currentIteration + 1;
            }
        }
        if (numberOfNoChange >= (population / 2)-1) {
            noChange = true;
        }
        ++currentIteration;
        sort(graphes.begin(), graphes.end());
        if (PRINT_RESULT) {
            std::cout << "Iteration: " << currentIteration << " Meilleur graphe : " << bestCrossingResult << " Number of no Change: " << numberOfNoChange <<"\n[";
            for (int i = 0; i<10;i++) {
                std::cout << graphes[i].nombreCroisement << " ";
            }
            std::cout << "]" << std::endl;
        }
        if (DEBUG_GENETIQUE) {
            for (int i = 0; i<graphes.size();i++) { graphes[i].debugEverything(); }
        }
    }
    std::chrono::duration<double> secondsTotal = end - start;
    timeBest = secondsTotal.count();
    lastIteration = currentIteration;
    copyFromGraphe(graphes[0]);
}

// Effectue le croisement entre deux parents,
// Renvoie vrai si les deux parents ne sont pas identique
// Met a jour le nombre de croisement du graphe
bool Graphe::croisementVoisinageScore(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand) {
    int nbNoeudATraiter = originalGraphe1._noeuds.size() - originalGraphe1.nbNoeudEnCommun(originalGraphe2);
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(originalGraphe1);
        return false;
    }
    Graphe graphe1, graphe2;
    graphe1.copyFromGraphe(originalGraphe1);
    graphe2.copyFromGraphe(originalGraphe2);
    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    std::vector<int> nodeToUpdate;
    std::vector<int> nodeToRelocate;
    nodeToUpdate.resize(originalGraphe1.maxVoisin+1);
    nodeToRelocate.resize(originalGraphe1.maxVoisin+1);
    while (nbNoeudATraiter > 0) {
        int bestNodeId = -1;
        int meilleurScore;
        int nbRencontre = 0;
        int numberOfNodeToUpdate=0;
        int numberOfNodeToRelocate = 0;

        //Trouve le meilleur noeud du graphe en cours d'analyse
        for (int i = 0; i < _noeuds.size(); ++i) {
            if (!_noeuds[i].estPlace()) {
                int nodeScore = currentGraphe->_noeuds[i].score;
                if (bestNodeId == -1 || nodeScore < meilleurScore) {
                    bestNodeId = i;
                    meilleurScore = nodeScore;
                    nbRencontre = 1;
                }
                else if (meilleurScore == nodeScore) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        bestNodeId = i;
                    }
                }
            }
        }

        int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
        _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
        if (!graphe1._noeuds[bestNodeId].compare(&graphe2._noeuds[bestNodeId])) {
            --nbNoeudATraiter;
            int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
            nodeToUpdate[numberOfNodeToUpdate] = bestNodeId;
            numberOfNodeToUpdate++;
            otherGraphe->changeUpdateValue(bestNodeId);
            if (oldNodeId != -1) {
                if (!areVoisin(bestNodeId,oldNodeId)) {
                    nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                    numberOfNodeToRelocate++;
                    otherGraphe->changeUpdateValue(oldNodeId);
                }
            }
        }
        //Place tout les voisins du point choisis
        for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
            int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
            if (!_noeuds[idNodeVoisin].estPlace()) {
                if (currentGraphe->_noeuds[idNodeVoisin].getEmplacement() == nullptr) {
                    bestEmplacementId = getMeilleurEmplacement(_noeuds[idNodeVoisin]);
                }
                else {
                    bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
                }
                _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
                if ((!otherGraphe->_noeuds[idNodeVoisin].estPlace())||(!graphe1._noeuds[idNodeVoisin].compare(&graphe2._noeuds[idNodeVoisin]))) {
                    --nbNoeudATraiter;
                    int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                    nodeToUpdate[numberOfNodeToUpdate] = idNodeVoisin;
                    numberOfNodeToUpdate++;
                    otherGraphe->changeUpdateValue(idNodeVoisin);
                    if (oldNodeId != -1) {
                        if (!areVoisin(bestNodeId,oldNodeId)) {
                            nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                            numberOfNodeToRelocate++;
                            otherGraphe->changeUpdateValue(oldNodeId);
                        }
                    }
                }
            }
        }
        if (useRand) {
            otherGraphe->completePlacementAleatoireScore(nodeToRelocate, numberOfNodeToRelocate);
        }
        else {
            otherGraphe->completeBasicGloutonScore(nodeToRelocate, numberOfNodeToRelocate);
        }
        for (int i=0;i<numberOfNodeToUpdate;i++) {
            otherGraphe->updateNodeScore(nodeToUpdate[i]);
        }
        //Si les lieux coincident les noeuds ne sont plus à traiter
        for (int i = 0; i < numberOfNodeToRelocate; ++i) {
            otherGraphe->updateNodeScore(nodeToRelocate[i]);
            if (graphe1._noeuds[nodeToRelocate[i]].compare(&graphe2._noeuds[nodeToRelocate[i]])) {
                --nbNoeudATraiter;
            }
        }

        //Change le parent choisis 
        if (currentGrapheNumber == 0) {
            currentGraphe = &graphe2;
            otherGraphe = &graphe1;
            currentGrapheNumber = 1;
        }
        else {
            currentGraphe = &graphe1;
            otherGraphe = &graphe2;
            currentGrapheNumber = 0;
        }
    }
    nombreCroisement = otherGraphe->nombreCroisement;
    _noeuds.swap(graphe1._noeuds);
    _aretes.swap(graphe1._aretes);
    _emplacements.swap(graphe1._emplacements);
    isNombreCroisementUpdated = true;
    isNodeScoreUpdated = true;
    isIntersectionVectorUpdated = true;
    return true;
}

// Effectue une seule etape dans l'algorithme de croisement, utile pour l'affichage opengl
void Graphe::stepCroisementVoisinageScore(Graphe& graphe1, Graphe& graphe2, bool useRand, int &nbNoeudATraiter, int &currentGrapheNumber) {
    if (nbNoeudATraiter == 0) {
        return;
    }
    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    std::vector<int> nodeToUpdate;
    std::vector<int> nodeToRelocate;
    nodeToUpdate.resize(graphe1.maxVoisin+1);
    nodeToRelocate.resize(graphe1.maxVoisin+1);

    int bestNodeId = -1;
    int meilleurScore;
    int nbRencontre = 0;
    int numberOfNodeToUpdate=0;
    int numberOfNodeToRelocate = 0;

    //Trouve le meilleur noeud du graphe en cours d'analyse
    for (int i = 0; i < _noeuds.size(); ++i) {
        if (!_noeuds[i].estPlace()) {
            int nodeScore = currentGraphe->_noeuds[i].score;
            if (bestNodeId == -1 || nodeScore < meilleurScore) {
                bestNodeId = i;
                meilleurScore = nodeScore;
                nbRencontre = 1;
            }
            else if (meilleurScore == nodeScore) {
                ++nbRencontre;
                if (generateRand(nbRencontre) == 1) {
                    bestNodeId = i;
                }
            }
        }
    }

    int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
    _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
    if (!graphe1._noeuds[bestNodeId].compare(&graphe2._noeuds[bestNodeId])) {
        --nbNoeudATraiter;
        int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
        nodeToUpdate[numberOfNodeToUpdate] = bestNodeId;
        numberOfNodeToUpdate++;
        otherGraphe->changeUpdateValue(bestNodeId);
        if (oldNodeId != -1) {
            if (!areVoisin(bestNodeId,oldNodeId)) {
                nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                numberOfNodeToRelocate++;
                otherGraphe->changeUpdateValue(oldNodeId);
            }
        }
    }
    //Place tout les voisins du point choisis
    for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
        int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
        if (!_noeuds[idNodeVoisin].estPlace()) {
            if (currentGraphe->_noeuds[idNodeVoisin].getEmplacement() == nullptr) {
                bestEmplacementId = getMeilleurEmplacement(_noeuds[idNodeVoisin]);
            }
            else {
                bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
            }
            _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
            if ((!otherGraphe->_noeuds[idNodeVoisin].estPlace())||(!graphe1._noeuds[idNodeVoisin].compare(&graphe2._noeuds[idNodeVoisin]))) {
                --nbNoeudATraiter;
                int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                nodeToUpdate[numberOfNodeToUpdate] = idNodeVoisin;
                numberOfNodeToUpdate++;
                otherGraphe->changeUpdateValue(idNodeVoisin);
                if (oldNodeId != -1) {
                    if (!areVoisin(bestNodeId,oldNodeId)) {
                        nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                        numberOfNodeToRelocate++;
                        otherGraphe->changeUpdateValue(oldNodeId);
                    }
                }
            }
        }
    }
    if (useRand) {
        otherGraphe->completePlacementAleatoireScore(nodeToRelocate, numberOfNodeToRelocate);
    }
    else {
        otherGraphe->completeBasicGloutonScore(nodeToRelocate, numberOfNodeToRelocate);
    }
    for (int i=0;i<nodeToUpdate.size();i++) {
        otherGraphe->updateNodeScore(nodeToUpdate[i]);
    }
    //Si les lieux coincident les noeuds ne sont plus à traiter
    for (int i = 0; i < numberOfNodeToRelocate; ++i) {
        otherGraphe->updateNodeScore(nodeToRelocate[i]);
        if (graphe1._noeuds[nodeToRelocate[i]].compare(&graphe2._noeuds[nodeToRelocate[i]])) {
            --nbNoeudATraiter;
        }
    }

    currentGrapheNumber = (currentGrapheNumber+1)%2;
    isNombreCroisementUpdated = true;
    if (nbNoeudATraiter == 0) {
        loadCopy(graphe1.saveCopy());
        nombreCroisement = otherGraphe->nombreCroisement;
    }
    std::cout << "NNT: " << nbNoeudATraiter << std::endl;
}

// Effectue le croisement entre deux parents,
// Renvoie vrai si les deux parents ne sont pas identique
// Met a jour le nombre de croisement du graphe
bool Graphe::croisementBestOfBoth(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand) {
    int nbNoeudATraiter = originalGraphe1._noeuds.size() - originalGraphe1.nbNoeudEnCommun(originalGraphe2);
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(originalGraphe1);
        return false;
    }
    Graphe graphe1, graphe2;
    graphe1.copyFromGraphe(originalGraphe1);
    graphe2.copyFromGraphe(originalGraphe2);
    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    std::vector<int> nodeToUpdate;
    std::vector<int> nodeToRelocate;
    nodeToUpdate.resize(originalGraphe1.maxVoisin+1);
    nodeToRelocate.resize(originalGraphe1.maxVoisin+1);
    while (nbNoeudATraiter > 0) {
        int bestNodeId = -1;
        int meilleurScore;
        int nbRencontre = 0;
        int numberOfNodeToUpdate=0;
        int numberOfNodeToRelocate = 0;

        //Trouve le meilleur noeud du graphe en cours d'analyse
        for (int i = 0; i < _noeuds.size(); ++i) {
            if (!_noeuds[i].estPlace()) {
                int nodeScore = graphe1._noeuds[i].score;
                if (bestNodeId == -1 || nodeScore < meilleurScore) {
                    bestNodeId = i;
                    meilleurScore = nodeScore;
                    nbRencontre = 1;
                    if (currentGraphe != &graphe1) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
                }
                else if (meilleurScore == nodeScore) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        bestNodeId = i;
                    }
                }
            }
        }
        for (int i = 0; i < _noeuds.size(); ++i) {
            if (!_noeuds[i].estPlace()) {
                int nodeScore = graphe2._noeuds[i].score;
                if (bestNodeId == -1 || nodeScore < meilleurScore) {
                    bestNodeId = i;
                    meilleurScore = nodeScore;
                    nbRencontre = 1;
                    if (currentGraphe != &graphe2) { currentGraphe = &graphe2; otherGraphe = &graphe1; }
                }
                else if (meilleurScore == nodeScore) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        bestNodeId = i;
                    }
                }
            }
        }

        int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
        _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
        if (!graphe1._noeuds[bestNodeId].compare(&graphe2._noeuds[bestNodeId])) {
            --nbNoeudATraiter;
            int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
            nodeToUpdate[numberOfNodeToUpdate] = bestNodeId;
            numberOfNodeToUpdate++;
            otherGraphe->changeUpdateValue(bestNodeId);
            if (oldNodeId != -1) {
                if (!areVoisin(bestNodeId,oldNodeId)) {
                    nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                    numberOfNodeToRelocate++;
                    otherGraphe->changeUpdateValue(oldNodeId);
                }
            }
        }
        //Place tout les voisins du point choisis
        for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
            int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
            if (!_noeuds[idNodeVoisin].estPlace()) {
                if (currentGraphe->_noeuds[idNodeVoisin].getEmplacement() == nullptr) {
                    bestEmplacementId = getMeilleurEmplacement(_noeuds[idNodeVoisin]);
                }
                else {
                    bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
                }
                _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
                if ((!otherGraphe->_noeuds[idNodeVoisin].estPlace())||(!graphe1._noeuds[idNodeVoisin].compare(&graphe2._noeuds[idNodeVoisin]))) {
                    --nbNoeudATraiter;
                    int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                    nodeToUpdate[numberOfNodeToUpdate] = idNodeVoisin;
                    numberOfNodeToUpdate++;
                    otherGraphe->changeUpdateValue(idNodeVoisin);
                    if (oldNodeId != -1) {
                        if (!areVoisin(bestNodeId,oldNodeId)) {
                            nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                            numberOfNodeToRelocate++;
                            otherGraphe->changeUpdateValue(oldNodeId);
                        }
                    }
                }
            }
        }
        if (useRand) {
            otherGraphe->completePlacementAleatoireScore(nodeToRelocate, numberOfNodeToRelocate);
        }
        else {
            otherGraphe->completeBasicGloutonScore(nodeToRelocate, numberOfNodeToRelocate);
        }
        for (int i=0;i<numberOfNodeToUpdate;i++) {
            otherGraphe->updateNodeScore(nodeToUpdate[i]);
        }
        //Si les lieux coincident les noeuds ne sont plus à traiter
        for (int i = 0; i < numberOfNodeToRelocate; ++i) {
            otherGraphe->updateNodeScore(nodeToRelocate[i]);
            if (graphe1._noeuds[nodeToRelocate[i]].compare(&graphe2._noeuds[nodeToRelocate[i]])) {
                --nbNoeudATraiter;
            }
        }
    }
    nombreCroisement = otherGraphe->nombreCroisement;
    _noeuds.swap(graphe1._noeuds);
    _aretes.swap(graphe1._aretes);
    _emplacements.swap(graphe1._emplacements);
    isNombreCroisementUpdated = true;
    isNodeScoreUpdated = true;
    isIntersectionVectorUpdated = true;
    return true;
}

// Effectue un croisement entre deux parents en séparant le graphe en deux parties
// Les emplacements sont triés par leur coord X
// Ne met pas à jour la variable nombreCroisement du graphe
bool Graphe::croisementHalfParent(Graphe& graphe1, Graphe& graphe2, std::vector<int>& sortedEmpId, bool useRand) {
    int nbNoeudATraiter = graphe1._noeuds.size() - graphe1.nbNoeudEnCommun(graphe2);
    //std::cout << "Nb noeud a traiter au debut: " << nbNoeudATraiter << "\n";
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(graphe1);
        return false;
    }
    std::vector<int> saveGraphe1 = graphe1.saveCopy();
    std::vector<int> saveGraphe2 = graphe2.saveCopy();
    int startVec2 = sortedEmpId.size() / 2;

    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    int startVec, endVec;
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; startVec = 0; endVec = startVec2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; startVec = startVec2; endVec = sortedEmpId.size(); }
    while (nbNoeudATraiter > 0) {
        Noeud* meilleurNoeud = nullptr;
        int meilleurScore;
        int nbRencontre = 0;
        for (int i = startVec; i < endVec; ++i) { // Recherche du meilleur noeud du coté du graphe choisi
            if (currentGraphe->_emplacements[sortedEmpId[i]]._noeud != nullptr) {
                int idNode = currentGraphe->_emplacements[sortedEmpId[i]]._noeud->getId();
                if (!_noeuds[idNode].estPlace()) {
                    int nodeScore = currentGraphe->getScoreCroisementNode(idNode);
                    if (meilleurNoeud == nullptr || meilleurScore < nodeScore) {
                        meilleurNoeud = &currentGraphe->_noeuds[idNode];
                        meilleurScore = nodeScore;
                        nbRencontre = 1;
                    }
                    else if (meilleurScore == nodeScore) {
                        ++nbRencontre;
                        if (generateRand(nbRencontre) == 1) {
                            meilleurNoeud = &currentGraphe->_noeuds[idNode];
                        }
                    }
                }
            }
        }

        if (meilleurNoeud != nullptr) {
            int meilleurEmplacement = meilleurNoeud->getEmplacement()->getId();
            _noeuds[meilleurNoeud->getId()].setEmplacement(&_emplacements[meilleurEmplacement]);
            if (!graphe1._noeuds[meilleurNoeud->getId()].compare(&graphe2._noeuds[meilleurNoeud->getId()])) {
                --nbNoeudATraiter;
                otherGraphe->_noeuds[meilleurNoeud->getId()].ecraseNoeud(otherGraphe->_emplacements[meilleurEmplacement]);
            }
            //Place tout les voisins du point choisis
            for (Noeud* noeudVoisin : meilleurNoeud->getVoisins()) {
                if (!_noeuds[noeudVoisin->getId()].estPlace()) {
                    if (noeudVoisin->getEmplacement() == nullptr) {
                        meilleurEmplacement = getMeilleurEmplacement(_noeuds[noeudVoisin->getId()]);
                    }
                    else {
                        meilleurEmplacement = noeudVoisin->getEmplacement()->getId();
                    }
                    _noeuds[noeudVoisin->getId()].setEmplacement(&_emplacements[meilleurEmplacement]);
                    Noeud* noeudGraphe1 = &graphe1._noeuds[noeudVoisin->getId()];
                    Noeud* noeudGraphe2 = &graphe2._noeuds[noeudVoisin->getId()];
                    if (noeudGraphe1->estPlace() && noeudGraphe2->estPlace()) {
                        if (!noeudGraphe1->compare(noeudGraphe2)) {
                            --nbNoeudATraiter;
                            otherGraphe->_noeuds[noeudVoisin->getId()].ecraseNoeud(graphe1._emplacements[meilleurEmplacement]);
                        }
                    }
                    else {
                        --nbNoeudATraiter;
                    }
                }
            }
            
            //Liste des noeuds à placer
            std::vector<int> noeudsAVerifier;
            for (int i = 0; i < _noeuds.size(); ++i) {
                if (!graphe1._noeuds[i].estPlace() || !graphe2._noeuds[i].estPlace()) {
                    noeudsAVerifier.push_back(i);
                }
            }
            if (useRand) {
                otherGraphe->completePlacementAleatoire();
            }
            else {
                otherGraphe->completeBasicGlouton();
            }
            //Si les lieux coincident les noeuds ne sont plus à traiter
            for (int i = 0; i < noeudsAVerifier.size(); ++i) {
                if (graphe1._noeuds[noeudsAVerifier[i]].compare(&graphe2._noeuds[noeudsAVerifier[i]])) {
                    --nbNoeudATraiter;
                }
            }
        }

        //Change le parent choisis 
        if (currentGrapheNumber == 0) {
            currentGraphe = &graphe2;
            otherGraphe = &graphe1;
            currentGrapheNumber = 1;
        }
        else {
            currentGraphe = &graphe1;
            otherGraphe = &graphe2;
            currentGrapheNumber = 0;
        }
    }

    _noeuds.swap(graphe1._noeuds);
    _aretes.swap(graphe1._aretes);
    _emplacements.swap(graphe1._emplacements);
    graphe1.loadCopy(saveGraphe1);
    graphe2.loadCopy(saveGraphe2);
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
    return true;
}

// Effectue un croisement entre deux parents en selectionnant un noeud aleatoirement de chaque parent en alternant
// Ne met pas à jour la variable nombreCroisement du graphe
bool Graphe::croisementAleatoire(Graphe& graphe1, Graphe& graphe2, bool useRand) {
    int nbNoeudATraiter = graphe1._noeuds.size() - graphe1.nbNoeudEnCommun(graphe2);
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(graphe1);
        return false;
    }
    std::vector<int> saveGraphe1 = graphe1.saveCopy();
    std::vector<int> saveGraphe2 = graphe2.saveCopy();

    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    while (nbNoeudATraiter > 0) {
        int bestNodeId = generateRand(_noeuds.size() - 1);
        while (_noeuds[bestNodeId].estPlace()) {
            bestNodeId = generateRand(_noeuds.size() - 1);
        }

        int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
        _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
        if (!graphe1._noeuds[bestNodeId].compare(&graphe2._noeuds[bestNodeId])) {
            --nbNoeudATraiter;
            int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
            if (oldNodeId != -1) {
                if (useRand) {
                    otherGraphe->completePlacementAleatoire();
                }
                else {
                    otherGraphe->completeBasicGlouton();
                }		//std::cout << "Nb noeud a traiter au debut: " << nbNoeudATraiter << "\n";
                if (graphe1._noeuds[oldNodeId].compare(&graphe2._noeuds[oldNodeId])) {
                    --nbNoeudATraiter;
                }
            }
        }

        //Change le parent choisis 
        if (currentGrapheNumber == 0) {
            currentGraphe = &graphe2;
            otherGraphe = &graphe1;
            currentGrapheNumber = 1;
        }
        else {
            currentGraphe = &graphe1;
            otherGraphe = &graphe2;
            currentGrapheNumber = 0;
        }
    }

    _noeuds.swap(graphe1._noeuds);
    _aretes.swap(graphe1._aretes);
    _emplacements.swap(graphe1._emplacements);
    graphe1.loadCopy(saveGraphe1);
    graphe2.loadCopy(saveGraphe2);
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
    return true;
}

// Effectue le croisement entre deux parents,
// On selectionne un noeud en alternant de parent, celui qui creer le moin d'intersection si le place chez l'enfant
// Renvoie vrai si les deux parents ne sont pas identique
// Ne met pas a jour le nombre de croisement du graphe et des noeuds
bool Graphe::croisementEnfantScore(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand) {
    std::vector<int> commonNodeVec, otherNodeVec, indexNodeInOtherVec;
    isNombreCroisementUpdated = false;
    originalGraphe1.separateNodesInCommon(originalGraphe2, commonNodeVec, otherNodeVec, indexNodeInOtherVec);
    int nbNoeudATraiter = otherNodeVec.size();
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(originalGraphe1);
        return false;
    }
    std::vector<int> activeEdges; // Contient les id des aretes actives chez l'enfant
    for (int i=0;i<commonNodeVec.size();i++) { // On place les noeuds communs et on active les aretes placées
        _noeuds[commonNodeVec[i]].setEmplacement(&_emplacements[originalGraphe1._noeuds[commonNodeVec[i]].getEmplacement()->_id]);
        for (const int& areteId : _noeuds[commonNodeVec[i]]._aretes) {
            if (_aretes[areteId].estPlace()) {
                activeEdges.push_back(areteId);
            }
        }
    }
    Graphe graphe1, graphe2;
    graphe1.copyFromGraphe(originalGraphe1);
    graphe2.copyFromGraphe(originalGraphe2);
    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    std::vector<int> nodeToRelocate;
    while (nbNoeudATraiter > 0) {
        int bestNodeId = -1;
        long bestScore;
        int bestNbVoisinsPlace = -1;
        int bestNbVoisin = -1;
        int bestNodeIndexInVec = -1;
        for (int i=0;i<nbNoeudATraiter;i++) {
            if (currentGraphe->_noeuds[otherNodeVec[i]].estPlace()) {
                if (_emplacements[currentGraphe->_noeuds[otherNodeVec[i]].getEmplacement()->_id].estDisponible()) {
                    int nodeNbVoisinsPlace = 0;
                    for (const int& areteId : _noeuds[otherNodeVec[i]]._aretes) {
                        if (_aretes[areteId].unPlace()) { nodeNbVoisinsPlace++; }
                    }
                    int nodeNbVoisins = _noeuds[otherNodeVec[i]]._aretes.size();
                    if (nodeNbVoisinsPlace > bestNbVoisinsPlace) {
                        bestNodeId = otherNodeVec[i];
                        bestNbVoisinsPlace = nodeNbVoisinsPlace;
                        bestNbVoisin = nodeNbVoisins;
                        bestScore = -1;
                        bestNodeIndexInVec = i;
                    }
                    else if (nodeNbVoisinsPlace == bestNbVoisinsPlace) {
                        if (bestScore == -1) {
                            bestScore = getNodeScoreEnfant(*currentGraphe, activeEdges,bestNodeId);
                        }
                        long nodeScore = getNodeScoreEnfant(*currentGraphe, activeEdges,otherNodeVec[i]);
                        if (nodeScore < bestScore) {
                            bestNodeId = otherNodeVec[i];
                            bestNbVoisinsPlace = nodeNbVoisinsPlace;
                            bestNbVoisin = nodeNbVoisins;
                            bestScore = nodeScore;
                            bestNodeIndexInVec = i;
                        }
                        else if (nodeScore == bestScore) {
                            if (nodeNbVoisins > bestNbVoisin) {
                                bestNodeId = otherNodeVec[i];
                                bestNbVoisinsPlace = nodeNbVoisinsPlace;
                                bestNbVoisin = nodeNbVoisins;
                                bestScore = nodeScore;
                                bestNodeIndexInVec = i;
                            }
                        }
                    }
                }
            }
        }
        if (bestNodeId != -1) {
            int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
            _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
            for (const int& areteId : _noeuds[bestNodeId]._aretes) {
                if (_aretes[areteId].estPlace())
                    activeEdges.push_back(areteId);
            }
            --nbNoeudATraiter;
            otherNodeVec[bestNodeIndexInVec] = otherNodeVec[nbNoeudATraiter];
            indexNodeInOtherVec[otherNodeVec[bestNodeIndexInVec]] = bestNodeIndexInVec;
            int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
            if (oldNodeId != -1) {
                if (!areVoisin(bestNodeId,oldNodeId)) {
                    if (!currentGraphe->_noeuds[oldNodeId].estPlace()) {
                        nbNoeudATraiter--;
                        nodeToRelocate.push_back(oldNodeId);
                        otherNodeVec[indexNodeInOtherVec[oldNodeId]] = otherNodeVec[nbNoeudATraiter];
                        indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[oldNodeId];
                    }
                }
            }
            //Place tout les voisins du point choisis
            for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
                int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
                if (!_noeuds[idNodeVoisin].estPlace()) {
                    if (currentGraphe->_noeuds[idNodeVoisin].estPlace()) {
                        bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
                        _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
                        for (const int& areteId : _noeuds[idNodeVoisin]._aretes) {
                            if (_aretes[areteId].estPlace())
                                activeEdges.push_back(areteId);
                        }
                        --nbNoeudATraiter;
                        otherNodeVec[indexNodeInOtherVec[idNodeVoisin]] = otherNodeVec[nbNoeudATraiter];
                        indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[idNodeVoisin];
                        int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                        if (oldNodeId != -1) {
                            if (!currentGraphe->_noeuds[oldNodeId].estPlace()) {
                                nodeToRelocate.push_back(oldNodeId);
                                nbNoeudATraiter--;
                                otherNodeVec[indexNodeInOtherVec[oldNodeId]] = otherNodeVec[nbNoeudATraiter];
                                indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[oldNodeId];
                            }
                        }
                    }
                    else {
                        if (!otherGraphe->_noeuds[idNodeVoisin].estPlace()) {
                            if (std::find(nodeToRelocate.begin(),nodeToRelocate.end(),idNodeVoisin) == nodeToRelocate.end()) {
                                nodeToRelocate.push_back(idNodeVoisin);
                                nbNoeudATraiter--;
                                otherNodeVec[indexNodeInOtherVec[idNodeVoisin]] = otherNodeVec[nbNoeudATraiter];
                                indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[idNodeVoisin];
                            }
                        }
                    }
                }
            }
        }
        //Change le parent choisis 
        if (currentGrapheNumber == 0) {
            currentGraphe = &graphe2;
            otherGraphe = &graphe1;
            currentGrapheNumber = 1;
        }
        else {
            currentGraphe = &graphe1;
            otherGraphe = &graphe2;
            currentGrapheNumber = 0;
        }
    }
    int tailleMax = nodeToRelocate.size();
    completeBasicGloutonScore(nodeToRelocate,tailleMax);
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
    return true;
}

// Effectue le croisement entre deux parents,
// Renvoie vrai si les deux parents ne sont pas identique
// Ne met pas à jour le nombre de croisement d'un graphe
bool Graphe::croisementVoisinageFrom(Graphe& graphe1, Graphe& graphe2, bool useRand) {
    int nbNoeudATraiter = graphe1._noeuds.size() - graphe1.nbNoeudEnCommun(graphe2);
    //std::cout << "Nb noeud a traiter au debut: " << nbNoeudATraiter << "\n";
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(graphe1);
        return false;
    }
    std::vector<int> saveGraphe1 = graphe1.saveCopy();
    std::vector<int> saveGraphe2 = graphe2.saveCopy();
    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    std::vector<int> nodeToRelocate;
    nodeToRelocate.resize(graphe1._noeuds.size());
    while (nbNoeudATraiter > 0) {
        int bestNodeId = -1;
        int meilleurScore;
        int nbRencontre = 0;
        int numberOfNodeToRelocate = 0;
        //Trouve le meilleur noeud du graphe en cours d'analyse
        for (int i = 0; i < _noeuds.size(); ++i) {
            if (!_noeuds[i].estPlace()) {
                int nodeScore = currentGraphe->getScoreCroisementNode(i);
                if (bestNodeId == -1 || nodeScore < meilleurScore) {
                    bestNodeId = i;
                    meilleurScore = nodeScore;
                    nbRencontre = 1;
                }
                else if (meilleurScore == nodeScore) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        bestNodeId = i;
                    }
                }
            }
        }
        int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
        _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
        if (!graphe1._noeuds[bestNodeId].compare(&graphe2._noeuds[bestNodeId])) {
            --nbNoeudATraiter;
            int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
            if (oldNodeId != -1) {
                if (!areVoisin(bestNodeId,oldNodeId)) {
                    nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                    numberOfNodeToRelocate++;
                }
            }
        }
        //Place tout les voisins du point choisis
        for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
            int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
            if (!_noeuds[idNodeVoisin].estPlace()) {
                if (currentGraphe->_noeuds[idNodeVoisin].getEmplacement() == nullptr) {
                    bestEmplacementId = getMeilleurEmplacement(_noeuds[idNodeVoisin]);
                }
                else {
                    bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
                }
                _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
                if ((!otherGraphe->_noeuds[idNodeVoisin].estPlace())||(!graphe1._noeuds[idNodeVoisin].compare(&graphe2._noeuds[idNodeVoisin]))) {
                    --nbNoeudATraiter;
                    int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                    if (oldNodeId != -1) {
                        if (!areVoisin(bestNodeId,oldNodeId)) {
                            nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                            numberOfNodeToRelocate++;
                        }
                    }
                }
            }
        }
        if (useRand) {
            otherGraphe->completePlacementAleatoire();
        }
        else {
            otherGraphe->completeBasicGlouton();
        }
        //Si les lieux coincident les noeuds ne sont plus à traitercurrentGraphe->_noeuds[idNodeVoisin].ecraseNoeud(currentGraphe->_emplacements[bestEmplacementId]);
        for (int i = 0; i < numberOfNodeToRelocate; ++i) {
            if (graphe1._noeuds[nodeToRelocate[i]].compare(&graphe2._noeuds[nodeToRelocate[i]])) {
                --nbNoeudATraiter;
            }
        }


        //Change le parent choisis 
        if (currentGrapheNumber == 0) {
            currentGraphe = &graphe2;
            otherGraphe = &graphe1;
            currentGrapheNumber = 1;
        }
        else {
            currentGraphe = &graphe1;
            otherGraphe = &graphe2;
            currentGrapheNumber = 0;
        }
    }
    _noeuds.swap(graphe1._noeuds);
    _aretes.swap(graphe1._aretes);
    _emplacements.swap(graphe1._emplacements);
    graphe1.loadCopy(saveGraphe1);
    graphe2.loadCopy(saveGraphe2);
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
    return true;
}

// Effectue une seule etape dans l'algorithme de croisement, utile pour l'affichage opengl
void Graphe::stepCroisementVoisinageFrom(Graphe& graphe1, Graphe& graphe2, bool useRand, int &nbNoeudATraiter, int &currentGrapheNumber) {
    if (nbNoeudATraiter == 0) {
        copyFromGraphe(graphe1);
        return;
    }

    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }

    int bestNodeId = -1;
    int bestEmplacementId = -1;
    int meilleurScore;
    int nbRencontre = 0;
    int numberOfNodeToRelocate = 0;

    std::vector<int> nodeToRelocate;
    nodeToRelocate.resize(graphe1.maxVoisin+1);

    for (int i = 0; i < _noeuds.size(); ++i) {
        if (!_noeuds[i].estPlace()) {
            int nodeScore = currentGraphe->getScoreCroisementNode(i);
            if (bestNodeId == -1 || nodeScore < meilleurScore) {
                bestNodeId = i;
                meilleurScore = nodeScore;
                nbRencontre = 1;
            }
            else if (meilleurScore == nodeScore) {
                ++nbRencontre;
                if (generateRand(nbRencontre) == 1) {
                    bestNodeId = i;
                }
            }
        }
    }

    bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
    _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
    if (!graphe1._noeuds[bestNodeId].compare(&graphe2._noeuds[bestNodeId])) {
        --nbNoeudATraiter;
        int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
        if (oldNodeId != -1) {
            if (!areVoisin(bestNodeId,oldNodeId)) {
                nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                numberOfNodeToRelocate++;
            }
        }
    }
    //Place tout les voisins du point choisis
    for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
        int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
        if (!_noeuds[idNodeVoisin].estPlace()) {
            if (currentGraphe->_noeuds[idNodeVoisin].getEmplacement() == nullptr) {
                bestEmplacementId = getMeilleurEmplacement(_noeuds[idNodeVoisin]);
            }
            else {
                bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
            }
            _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
            if ((!otherGraphe->_noeuds[idNodeVoisin].estPlace())||(!graphe1._noeuds[idNodeVoisin].compare(&graphe2._noeuds[idNodeVoisin]))) {
                --nbNoeudATraiter;
                int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                if (oldNodeId != -1) {
                    if (!areVoisin(bestNodeId,oldNodeId)) {
                        nodeToRelocate[numberOfNodeToRelocate] = oldNodeId;
                        numberOfNodeToRelocate++;
                    }
                }
            }
        }
    }

    if (useRand) {
        otherGraphe->completePlacementAleatoire();
    }
    else {
        otherGraphe->completeBasicGlouton();
    }
    //Si les lieux coincident les noeuds ne sont plus à traiter
    for (int i = 0; i < numberOfNodeToRelocate; ++i) {
        if (graphe1._noeuds[nodeToRelocate[i]].compare(&graphe2._noeuds[nodeToRelocate[i]])) {
            --nbNoeudATraiter;
        }
    }
    currentGrapheNumber = (currentGrapheNumber+1)%2;
    graphe1.isNombreCroisementUpdated = false;
    graphe2.isNombreCroisementUpdated = false;
    isNombreCroisementUpdated = false;
    if (nbNoeudATraiter == 0) {
        loadCopy(graphe1.saveCopy());
    }
}

// Effectue le croisement entre deux parents,
// On selectionne un noeud en alternant de parent, celui qui creer le moin d'intersection si le place chez l'enfant
// Renvoie vrai si les deux parents ne sont pas identique
// Ne met pas a jour le nombre de croisement du graphe et des noeuds
bool Graphe::croisementEnfantScoreGrille(Graphe& originalGraphe1, Graphe& originalGraphe2, bool useRand) {
    std::vector<int> commonNodeVec, otherNodeVec, indexNodeInOtherVec;
    isNombreCroisementUpdated = false;
    originalGraphe1.separateNodesInCommon(originalGraphe2, commonNodeVec, otherNodeVec, indexNodeInOtherVec);
    int nbNoeudATraiter = otherNodeVec.size();
    if (nbNoeudATraiter == 0) {
        copyNodesFromGraphe(originalGraphe1);
        copyGrilleFromGraphe(originalGraphe1);
        return false;
    }
    clearGrille();
    for (int i=0;i<commonNodeVec.size();i++) { // On place les noeuds communs et on active les aretes placées
        _noeuds[commonNodeVec[i]].setEmplacement(&_emplacements[originalGraphe1._noeuds[commonNodeVec[i]].getEmplacement()->_id]);
        for (const int& areteId : _noeuds[commonNodeVec[i]]._aretes) {
            if (_aretes[areteId].estPlace()) {
                initAreteCellule(areteId);
            }
        }
    }
    Graphe graphe1, graphe2;
    graphe1.copyFromGraphe(originalGraphe1);
    graphe2.copyFromGraphe(originalGraphe2);
    Graphe* currentGraphe = nullptr, * otherGraphe = nullptr;
    int currentGrapheNumber = generateRand(1);
    if (currentGrapheNumber == 0) { currentGraphe = &graphe1; otherGraphe = &graphe2; }
    else { currentGraphe = &graphe2; otherGraphe = &graphe1; }
    std::vector<int> nodeToRelocate;
    while (nbNoeudATraiter > 0) {
        int bestNodeId = -1;
        long bestScore;
        int bestNbVoisinsPlace = -1;
        int bestNbVoisin = -1;
        int bestNodeIndexInVec = -1;
        for (int i=0;i<nbNoeudATraiter;i++) {
            if (currentGraphe->_noeuds[otherNodeVec[i]].estPlace()) {
                if (_emplacements[currentGraphe->_noeuds[otherNodeVec[i]].getEmplacement()->_id].estDisponible()) {
                    int nodeNbVoisinsPlace = 0;
                    for (const int& areteId : _noeuds[otherNodeVec[i]]._aretes) {
                        if (_aretes[areteId].unPlace()) { nodeNbVoisinsPlace++; }
                    }
                    int nodeNbVoisins = _noeuds[otherNodeVec[i]]._aretes.size();
                    if (nodeNbVoisinsPlace > bestNbVoisinsPlace) {
                        bestNodeId = otherNodeVec[i];
                        bestNbVoisinsPlace = nodeNbVoisinsPlace;
                        bestNbVoisin = nodeNbVoisins;
                        bestScore = -1;
                        bestNodeIndexInVec = i;
                    }
                    else if (nodeNbVoisinsPlace == bestNbVoisinsPlace) {
                        if (bestScore == -1) {
                            bestScore = getNodeScoreEnfantGrille(*currentGraphe,bestNodeId);
                        }
                        long nodeScore = getNodeScoreEnfantGrille(*currentGraphe,otherNodeVec[i]);
                        if (nodeScore < bestScore) {
                            bestNodeId = otherNodeVec[i];
                            bestNbVoisinsPlace = nodeNbVoisinsPlace;
                            bestNbVoisin = nodeNbVoisins;
                            bestScore = nodeScore;
                            bestNodeIndexInVec = i;
                        }
                        else if (nodeScore == bestScore) {
                            if (nodeNbVoisins > bestNbVoisin) {
                                bestNodeId = otherNodeVec[i];
                                bestNbVoisinsPlace = nodeNbVoisinsPlace;
                                bestNbVoisin = nodeNbVoisins;
                                bestScore = nodeScore;
                                bestNodeIndexInVec = i;
                            }
                        }
                    }
                }
            }
        }
        if (bestNodeId != -1) {
            int bestEmplacementId = currentGraphe->_noeuds[bestNodeId].getEmplacement()->getId();
            _noeuds[bestNodeId].setEmplacement(&_emplacements[bestEmplacementId]);
            for (const int& areteId : _noeuds[bestNodeId]._aretes) {
                if (_aretes[areteId].estPlace()) {
                    initAreteCellule(areteId);
                }
            }
            --nbNoeudATraiter;
            otherNodeVec[bestNodeIndexInVec] = otherNodeVec[nbNoeudATraiter];
            indexNodeInOtherVec[otherNodeVec[bestNodeIndexInVec]] = bestNodeIndexInVec;
            int oldNodeId = otherGraphe->_noeuds[bestNodeId].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
            if (oldNodeId != -1) {
                if (!areVoisin(bestNodeId,oldNodeId)) {
                    if (!currentGraphe->_noeuds[oldNodeId].estPlace()) {
                        nbNoeudATraiter--;
                        nodeToRelocate.push_back(oldNodeId);
                        otherNodeVec[indexNodeInOtherVec[oldNodeId]] = otherNodeVec[nbNoeudATraiter];
                        indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[oldNodeId];
                    }
                }
            }
            //Place tout les voisins du point choisis
            for (int i=0;i<currentGraphe->_noeuds[bestNodeId].voisins.size();i++) {
                int idNodeVoisin = currentGraphe->_noeuds[bestNodeId].voisins[i]->getId();
                if (!_noeuds[idNodeVoisin].estPlace()) {
                    if (currentGraphe->_noeuds[idNodeVoisin].estPlace()) {
                        bestEmplacementId = currentGraphe->_noeuds[idNodeVoisin].getEmplacement()->getId();
                        _noeuds[idNodeVoisin].setEmplacement(&_emplacements[bestEmplacementId]);
                        for (const int& areteId : _noeuds[idNodeVoisin]._aretes) {
                            if (_aretes[areteId].estPlace()) {
                                initAreteCellule(areteId);
                            }
                        }
                        --nbNoeudATraiter;
                        otherNodeVec[indexNodeInOtherVec[idNodeVoisin]] = otherNodeVec[nbNoeudATraiter];
                        indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[idNodeVoisin];
                        int oldNodeId = otherGraphe->_noeuds[idNodeVoisin].ecraseNoeud(otherGraphe->_emplacements[bestEmplacementId]);
                        if (oldNodeId != -1) {
                            if (!currentGraphe->_noeuds[oldNodeId].estPlace()) {
                                nodeToRelocate.push_back(oldNodeId);
                                nbNoeudATraiter--;
                                otherNodeVec[indexNodeInOtherVec[oldNodeId]] = otherNodeVec[nbNoeudATraiter];
                                indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[oldNodeId];
                            }
                        }
                    }
                    else {
                        if (!otherGraphe->_noeuds[idNodeVoisin].estPlace()) {
                            if (std::find(nodeToRelocate.begin(),nodeToRelocate.end(),idNodeVoisin) == nodeToRelocate.end()) {
                                nodeToRelocate.push_back(idNodeVoisin);
                                nbNoeudATraiter--;
                                otherNodeVec[indexNodeInOtherVec[idNodeVoisin]] = otherNodeVec[nbNoeudATraiter];
                                indexNodeInOtherVec[otherNodeVec[nbNoeudATraiter]] = indexNodeInOtherVec[idNodeVoisin];
                            }
                        }
                    }
                }
            }
        }
        //Change le parent choisis 
        if (currentGrapheNumber == 0) {
            currentGraphe = &graphe2;
            otherGraphe = &graphe1;
            currentGrapheNumber = 1;
        }
        else {
            currentGraphe = &graphe1;
            otherGraphe = &graphe2;
            currentGrapheNumber = 0;
        }
    }
    int tailleMax = nodeToRelocate.size();
    completeBasicGloutonScoreGrille(nodeToRelocate,tailleMax);
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
    return true;
}
