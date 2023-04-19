#include "graphe.hpp"
#include "stressMaj.hpp"
#include "pivotMDS.hpp"
#include "intersection.hpp"
#include <iostream>
#include <climits>

void Graphe::tirageCoordReel(std::pair<double,double>& coord) {
    coord.first = generateDoubleRand(gridWidth);
    coord.second = generateDoubleRand(gridHeight);
}

// Place les noeuds aleatoirement sur les emplacements disponibles.
// Ne tient pas a jour le score des noeuds ou du graphe.
void Graphe::placementAleatoire() {
    #if defined(DEBUG_GRAPHE)
        std::cout << "Placement aleatoire" << std::endl;
    #endif
    for (int i = 0; i < _noeuds.size(); ++i) {
        int emplacementId = generateRand(_emplacements.size() - 1);
        while (!_emplacements[emplacementId].estDisponible()) {
            emplacementId = (emplacementId + 1) % _emplacements.size();
        }
        _noeuds[i].setEmplacement(&_emplacements[emplacementId]);
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::placementAleatoireReel() {
    #if defined(DEBUG_GRAPHE)
        std::cout << "Placement aleatoire coord reel" << std::endl;
    #endif
    if ((gridWidth == 10)&&(gridHeight == 10)) {
        gridWidth = std::min((int)_noeuds.size()*2,6000);
        gridHeight = gridWidth;
    }
    for (int i = 0; i < _noeuds.size(); ++i) {
        int randLargeur = generateRand(gridWidth-1);
        int randHauteur = generateRand(gridHeight-1);
        double incrementX = generateDoubleRand(1);
        double incrementY = generateDoubleRand(1);
        _noeuds[i]._xreel = (double)randLargeur + incrementX;
        _noeuds[i]._yreel = (double)randHauteur + incrementY;
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

// Place les noeuds sur l'emplacement de meme ID
// Ne tient pas a jour le score des noeuds ou du graphe.
void Graphe::placementFixe() {
    #if defined(DEBUG_GRAPHE)
        std::cout << "Placement fixe" << std::endl;
    #endif
    for (int i=0;i<_noeuds.size();i++) {
        _noeuds[i].setEmplacement(&_emplacements[i]);
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

// Place un noeud aleatoirement sur un emplacement disponible.
// Ne tient pas a jour le score du noeud ou du graphe.
void Graphe::placementNoeudAleatoire(int idNoeud) {
    int emplacementId = generateRand(_emplacements.size() - 1);
    while (!_emplacements[emplacementId].estDisponible()) {
        emplacementId = (emplacementId + 1) % _emplacements.size();
    }
    _noeuds[idNoeud].setEmplacement(&_emplacements[emplacementId]);
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

// Selectionne les emplacements disponibles a egale distance d'un point et en renvoie un aleatoirement.
Emplacement* Graphe::getEmplacementPlusProche(std::pair<int,int>& origin) {
    Emplacement* meilleurEmplacement = &_emplacements[generateRand(_emplacements.size() - 1)];
    while (!meilleurEmplacement->estDisponible()) {
        meilleurEmplacement = &_emplacements[(meilleurEmplacement->getId() + 1) % _emplacements.size()];
    }
    int meilleurDistance = meilleurEmplacement->distance(origin);
    int nbRencontre = 0;
    for (int i = 0; i < _emplacements.size(); ++i) {
        if (_emplacements[i].estDisponible() && i != meilleurEmplacement->getId()) {
            int distanceActuel = _emplacements[i].distance(origin);
            if (meilleurDistance > distanceActuel) {
                meilleurEmplacement = &_emplacements[i];
                meilleurDistance = distanceActuel;
                nbRencontre = 0;
            }
            else if (meilleurDistance == distanceActuel) {
                ++nbRencontre;
                if (generateRand(nbRencontre) == 1) {
                    meilleurEmplacement = &_emplacements[i];
                    meilleurDistance = distanceActuel;
                }
            }
        }

    }
    return meilleurEmplacement;
}

// Selectionne les emplacements different de l'emplacement en parametre a egale distance de celui ci et en renvoie un aleatoirement.
Emplacement* Graphe::getEmplacementPlusProche(Emplacement* origin) {
    Emplacement* meilleurEmplacement = &_emplacements[0];
    int meilleurDistance = meilleurEmplacement->distance(origin);
    int nbRencontre = 0;
    for (int i = 1; i < _emplacements.size(); ++i) {
        if (origin->getId() != _emplacements[i].getId()) {
            int distanceActuel = _emplacements[i].distance(origin);
            if (distanceActuel < meilleurDistance) {
                meilleurEmplacement = &_emplacements[i];
                meilleurDistance = distanceActuel;
                nbRencontre = 0;
            }
            else if (meilleurDistance == distanceActuel) {
                ++nbRencontre;
                if (generateRand(nbRencontre) == 1) {
                    meilleurEmplacement = &_emplacements[i];
                }
            }
        }
    }
    return meilleurEmplacement;
}

// Retourne le centre de gravite des emplacements.
std::pair<int,int> Graphe::getCentreGravite() {
    double totalX = 0.0, totalY = 0.0;
    for (int i=0;i<_emplacements.size();i++) {
        totalX += _emplacements[i].getX();
        totalY += _emplacements[i].getY();
    }
    return std::pair<int,int>(totalX / _emplacements.size(),totalY / _emplacements.size());
}

// Retourne le centre de gravite des noeuds place
std::pair<int,int> Graphe::getCentreGraviteNoeudPlaces() {
    double totalX = 0.0, totalY = 0.0;
    int nbIter = 0;
    for (int i=0;i<_noeuds.size();i++) {
        if (_noeuds[i].estPlace()){
            totalX += _noeuds[i].getX();
            totalY += _noeuds[i].getY();
            nbIter++;
        }
    }
    return std::pair<int,int>(totalX / nbIter,totalY / nbIter);
}

std::pair<double,double> Graphe::getCentreGraviteDoubleNoeuds() {
    double totalX = 0.0, totalY = 0.0;
    double nbIter = 0.0;
    if (useCoordReel) {
        for (int i=0;i<_noeuds.size();i++) {
            totalX += _noeuds[i]._xreel;
            totalY += _noeuds[i]._yreel;
            nbIter++;
        }
    }
    else {
        for (int i=0;i<_noeuds.size();i++) {
            if (_noeuds[i].estPlace()){
                totalX += _noeuds[i].getEmplacement()->getX();
                totalY += _noeuds[i].getEmplacement()->getY();
                nbIter++;
            }
        }
    }
    return std::pair<double,double>(totalX / nbIter,totalY / nbIter);
}

// Retourne le centre de gravite des voisins place d'un noeud.
std::pair<int,int> Graphe::getCentreGraviteVoisin(Noeud* noeud) {
    double totalX = 0.0, totalY = 0.0;
    int nbIter = 0;
    for (int i=0;i<noeud->voisins.size();i++) {
        if (noeud->voisins[i]->estPlace()) {
            totalX += noeud->voisins[i]->getX();
            totalY += noeud->voisins[i]->getY();
            nbIter++;
        }
    }
    return std::pair<int,int>(totalX / nbIter,totalY / nbIter);
}

// Renvoie l'id du meilleur emplacement disponible
// Le noeud passé en argument ne doit pas être placé
int Graphe::getMeilleurEmplacement(Noeud& meilleurNoeud) {
    int nbRencontre = 0;
    long bestScore = INT_MAX;
    int bestId = -1;
    for (int j = 0; j < _emplacements.size(); j++) {
        if (_emplacements[j].estDisponible()) {
            meilleurNoeud.setEmplacement(&_emplacements[j]);
            long newScore = getNbCroisementGlouton();
            if (newScore < bestScore) {
                bestScore = newScore;
                bestId = j;
                nbRencontre = 1;
            }
            else if (newScore == bestScore) {
                ++nbRencontre;
                if (generateRand(nbRencontre) == 1) {
                    bestId = j;
                }
            }
        }
    }
    meilleurNoeud.clearEmplacement();
    return _emplacements[bestId].getId();
}

// Renvoie l'id du meilleur emplacement disponible
// Le noeud passé en argument ne doit pas être placé
int Graphe::getMeilleurEmplacementScore(Noeud& meilleurNoeud) {
    int nbRencontre = 0;
    long bestScore = -1;
    int bestId = -1;
    for (int j = 0; j < _emplacements.size(); j++) {
        if (_emplacements[j].estDisponible()) {
            if (bestId == -1) {
                bestId = j;
            }
            else {
                if (bestScore == -1) {
                    meilleurNoeud.setEmplacement(&_emplacements[bestId]);
                    bestScore = getNbCroisementGloutonScore(meilleurNoeud._id);
                }
                meilleurNoeud.setEmplacement(&_emplacements[j]);
                long newScore = getNbCroisementGloutonScore(meilleurNoeud._id);
                if (newScore < bestScore) {
                    bestScore = newScore;
                    bestId = j;
                    nbRencontre = 1;
                }
                else if (newScore == bestScore) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        bestId = j;
                    }
                }
            }
        }
    }
    meilleurNoeud.clearEmplacement();
    return bestId;
}

// Renvoie le meilleur emplacement disponible
int Graphe::getMeilleurEmplacementGravite(Noeud* meilleurNoeud, std::pair<int,int>& gravite) {
    Emplacement* oldEmplacement = meilleurNoeud->getEmplacement();
    int nbRencontre = 0;
    int bestId = -1;
    long bestScore = INT_MAX;
    long bestDistance = INT_MAX;
    for (int i = 0; i < _emplacements.size(); i++) {
        if (_emplacements[i].estDisponible()) {
            meilleurNoeud->setEmplacement(&_emplacements[i]);
            long newScore = getNbCroisementGlouton();
            if (newScore < bestScore) {
                bestScore = newScore;
                bestId = i;
                bestDistance = _emplacements[bestId].distance(gravite);
            }
            else if (newScore == bestScore) {
                long newDistance = _emplacements[i].distance(gravite);
                if (newDistance < bestDistance) {
                    bestScore = newScore;
                    bestId = i;
                    bestDistance = newDistance;
                }
            }
        }
    }
    if (oldEmplacement == nullptr) {
        meilleurNoeud->clearEmplacement();
    }
    else {
        meilleurNoeud->setEmplacement(oldEmplacement);
    }
    return bestId;
}

// Ancien algorithme glouton non optimisé mais fonctionnel.
void Graphe::glouton() {
    for (int i = 0; i < _noeuds.size(); i++) {
        // Selection aléatoire du noeud
        int randomId = generateRand(_noeuds.size() - 1);
        while (_noeuds[randomId].getEmplacement() != nullptr) {
            randomId = (randomId + 1) % _noeuds.size();
        }
        long bestScore = INT_MAX;
        int bestEmpId = -1;
        for (int j = 0; j < _emplacements.size(); j++) {
            if (_emplacements[j].estDisponible()) {
                _noeuds[randomId].setEmplacement(&_emplacements[j]);
                long newScore = getNbCroisementGlouton();
                if (newScore < bestScore) {
                    bestScore = newScore;
                    bestEmpId = j;
                }
            }
        }
        _noeuds[randomId].setEmplacement(&_emplacements[bestEmpId]);
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

// Algorithme glouton
void Graphe::gloutonRevisite() {
    int idNoeud = 0;
    int nbRencontre = 0;
    for (int i = 1; i < _noeuds.size(); ++i) {
        if (!(_noeuds[i].getVoisins().size() > _noeuds[idNoeud].getVoisins().size())) {
            idNoeud = i;
            nbRencontre = 1;
        }
        else if (_noeuds[i].getVoisins().size() == _noeuds[idNoeud].getVoisins().size()){
            ++nbRencontre;
            if (generateRand(nbRencontre) == 1){
                idNoeud = i;
            }
        }
    }

    for (int nbNoeud=0;nbNoeud<_noeuds.size();nbNoeud++) {
        Noeud* meilleurNoeud = nullptr;
        nbRencontre = 0;
        for (int i = 0; i < _noeuds.size(); ++i) {
            if (!_noeuds[i].estPlace()) {
                Noeud* currentNoeud = &_noeuds[i];
                if (meilleurNoeud == nullptr) {
                    meilleurNoeud = currentNoeud;
                }
                else if (meilleurNoeud->getVoisins().size() < currentNoeud->getVoisins().size()) {
                    meilleurNoeud = currentNoeud;
                    nbRencontre = 0;
                }
                else if (meilleurNoeud->getVoisins().size() == currentNoeud->getVoisins().size()) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        meilleurNoeud = currentNoeud;
                    }
                }
            }
        }
        if (meilleurNoeud != nullptr) {
            int bestId = -1;
            long bestScore = -1;
            nbRencontre = 0;
            for (int i=0;i<_emplacements.size();i++) {
                if (_emplacements[i].estDisponible()) {
                    if (bestId == -1) {
                        bestId = i;
                    }
                    else {
                        if (bestScore == -1) {
                            meilleurNoeud->setEmplacement(&_emplacements[bestId]);
                            bestScore = getScoreCroisementNodeGlouton(meilleurNoeud->_id);
                        }
                        meilleurNoeud->setEmplacement(&_emplacements[i]);
                        long newScore = getScoreCroisementNodeGlouton(meilleurNoeud->_id);
                        if (newScore < bestScore) {
                            bestScore = newScore;
                            bestId = i;
                            nbRencontre = 0;
                        }
                        else if (newScore == bestScore) {
                            nbRencontre++;
                            if (generateRand(nbRencontre) == 1) {
                                bestScore = newScore;
                                bestId = i;
                            }
                        }
                    }
                }
            }
            meilleurNoeud->setEmplacement(&_emplacements[bestId]);
        }
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

// Algorithme glouton avec grille
void Graphe::gloutonRevisiteGrid() {
    initGrille();
    registerSlotsInGrid();
    int idNoeud = 0;
    int nbRencontre = 0;
    for (int i = 1; i < _noeuds.size(); ++i) {
        if (!(_noeuds[i].getVoisins().size() > _noeuds[idNoeud].getVoisins().size())) {
            idNoeud = i;
            nbRencontre = 1;
        }
        else if (_noeuds[i].getVoisins().size() == _noeuds[idNoeud].getVoisins().size()){
            ++nbRencontre;
            if (generateRand(nbRencontre) == 1){
                idNoeud = i;
            }
        }
    }

    for (int nbNoeud=0;nbNoeud<_noeuds.size();nbNoeud++) {
        Noeud* meilleurNoeud = nullptr;
        nbRencontre = 0;
        for (int i = 0; i < _noeuds.size(); ++i) {
            if (!_noeuds[i].estPlace()) {
                Noeud* currentNoeud = &_noeuds[i];
                if (meilleurNoeud == nullptr) {
                    meilleurNoeud = currentNoeud;
                }
                else if (meilleurNoeud->getVoisins().size() < currentNoeud->getVoisins().size()) {
                    meilleurNoeud = currentNoeud;
                    nbRencontre = 0;
                }
                else if (meilleurNoeud->getVoisins().size() == currentNoeud->getVoisins().size()) {
                    ++nbRencontre;
                    if (generateRand(nbRencontre) == 1) {
                        meilleurNoeud = currentNoeud;
                    }
                }
            }
        }
        if (meilleurNoeud != nullptr) {
            int bestId = -1;
            long bestScore = -1;
            nbRencontre = 0;
            for (int i=0;i<_emplacements.size();i++) {
                if (_emplacements[i].estDisponible()) {
                    if (bestId == -1) {
                        bestId = i;
                    }
                    else {
                        if (bestScore == -1) {
                            meilleurNoeud->setEmplacement(&_emplacements[bestId]);
                            initNodeCellule(meilleurNoeud->_id);
                            bestScore = getScoreCroisementNodeGloutonGrid(meilleurNoeud->_id);
                        }
                        meilleurNoeud->setEmplacement(&_emplacements[i]);
                        recalcNodeCellule(meilleurNoeud->_id);
                        long newScore = getScoreCroisementNodeGloutonGrid(meilleurNoeud->_id);
                        if (newScore < bestScore) {
                            bestScore = newScore;
                            bestId = i;
                            nbRencontre = 0;
                        }
                        else if (newScore == bestScore) {
                            nbRencontre++;
                            if (generateRand(nbRencontre) == 1) {
                                bestScore = newScore;
                                bestId = i;
                            }
                        }
                    }
                }
            }
            meilleurNoeud->setEmplacement(&_emplacements[bestId]);
            if (bestScore == -1) { initNodeCellule(meilleurNoeud->_id); }
            else { recalcNodeCellule(meilleurNoeud->_id); }
        }
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::gloutonRevisiteGravite()
{
    int idNoeud = 0;
    int nbRencontre = 0;
    for (int i = 1; i < _noeuds.size(); ++i)
    {
        if (!_noeuds[i].estPlace() &&
            _noeuds[i].getVoisins().size() > _noeuds[idNoeud].getVoisins().size())
        {
            idNoeud = i;
            nbRencontre = 1;
        }
        else if (!_noeuds[i].estPlace() &&
            _noeuds[i].getVoisins().size() == _noeuds[idNoeud].getVoisins().size())
        {
            ++nbRencontre;
            int aleatoire = generateRand(nbRencontre);
            if (aleatoire == 1)
            {
                idNoeud = i;
            }
        }
    }

    std::pair<int,int> centreGravite = getCentreGravite();
    _noeuds[idNoeud].setEmplacement(getEmplacementPlusProche(centreGravite));

    while (!estPlace())
    {
        Noeud* meilleurNoeud = nullptr;
        nbRencontre = 0;
        for (int i = 0; i < _noeuds.size(); ++i)
        {
            if (!_noeuds[i].estPlace())
            {
                Noeud* currentVoisin = &_noeuds[i];
                if (meilleurNoeud == nullptr)
                {
                    meilleurNoeud = currentVoisin;
                }
                else if (meilleurNoeud->getVoisins().size() < currentVoisin->getVoisins().size())
                {
                    meilleurNoeud = currentVoisin;
                    nbRencontre = 0;
                }
                else if (meilleurNoeud->getVoisins().size() == currentVoisin->getVoisins().size())
                {
                    ++nbRencontre;
                    int aleatoire = generateRand(nbRencontre);
                    if (aleatoire == 1)
                    {
                        meilleurNoeud = currentVoisin;
                    }
                }
            }

        }

        if (meilleurNoeud != nullptr)
        {
            centreGravite = getCentreGraviteNoeudPlaces();
            Emplacement* emplacement = getEmplacementPlusProche(centreGravite);
            meilleurNoeud->setEmplacement(emplacement);
        }

    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::gloutonRevisiteVoisin()
{
    int idNoeud = 0;
    int nbRencontre = 0;
    for (int i = 1; i < _noeuds.size(); ++i)
    {
        if (!_noeuds[i].estPlace() &&
            _noeuds[i].getVoisins().size() > _noeuds[idNoeud].getVoisins().size())
        {
            idNoeud = i;
            nbRencontre = 1;
        }
        else if (!_noeuds[i].estPlace() &&
            _noeuds[i].getVoisins().size() == _noeuds[idNoeud].getVoisins().size())
        {
            ++nbRencontre;
            int aleatoire = generateRand(nbRencontre);
            if (aleatoire == 1)
            {
                idNoeud = i;
            }
        }
    }

    std::pair<int,int> centreGravite = getCentreGravite();
    _noeuds[idNoeud].setEmplacement(getEmplacementPlusProche(centreGravite));

    while (!estPlace())
    {
        Noeud* meilleurNoeud = nullptr;
        nbRencontre = 0;
        for (int i = 0; i < _noeuds.size(); ++i)
        {
            if (!_noeuds[i].estPlace())
            {
                Noeud* currentVoisin = &_noeuds[i];
                if (meilleurNoeud == nullptr)
                {
                    meilleurNoeud = currentVoisin;
                }
                else if (meilleurNoeud->getVoisins().size() < currentVoisin->getVoisins().size())
                {
                    meilleurNoeud = currentVoisin;
                    nbRencontre = 0;
                }
                else if (meilleurNoeud->getVoisins().size() == currentVoisin->getVoisins().size())
                {
                    ++nbRencontre;
                    int aleatoire = generateRand(nbRencontre);
                    if (aleatoire == 1)
                    {
                        meilleurNoeud = currentVoisin;
                    }
                }
            }

        }

        if (meilleurNoeud != nullptr)
        {
            centreGravite = getCentreGraviteVoisin(meilleurNoeud);
            Emplacement* emplacement = getEmplacementPlusProche(centreGravite);
            meilleurNoeud->setEmplacement(emplacement);
        }

    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::completeBasicGlouton() {
    int nbNode = nbAPlace();
    while (nbNode > 0) {
        Noeud* meilleurNoeud = nullptr;
        int nbRencontre = 0;
        for (int i = 0; i < _noeuds.size(); ++i) {

            if (!_noeuds[i].estPlace()) {
                if (meilleurNoeud == nullptr) {
                    meilleurNoeud = &_noeuds[i];
                }
                else if (_noeuds[i].getVoisinsPlaces() > meilleurNoeud->getVoisinsPlaces()) {
                    meilleurNoeud = &_noeuds[i];
                    nbRencontre = 1;
                }
                else if (_noeuds[i].getVoisinsPlaces() == meilleurNoeud->getVoisinsPlaces()) {
                    if (_noeuds[i].getVoisins().size() > meilleurNoeud->getVoisins().size()) {
                        meilleurNoeud = &_noeuds[i];
                        nbRencontre = 1;
                    }
                    else if (_noeuds[i].getVoisins().size() == meilleurNoeud->getVoisins().size()) {
                        ++nbRencontre;
                        if (generateRand(nbRencontre) == 1) {
                            meilleurNoeud = &_noeuds[i];
                        }
                    }
                }
            }
        }
        if (meilleurNoeud != nullptr) {
            meilleurNoeud->setEmplacement(&_emplacements[getMeilleurEmplacement(*meilleurNoeud)]);
            nbNode--;
        }
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::completeBasicGloutonScore(std::vector<int>& vecNode, int tailleMax) {
    std::vector<bool> marque;
    marque.resize(tailleMax);
    for (int i=0;i<tailleMax;i++) {
        int nbRencontre = 0;
        int bestNbVoisinsPlaces = -1;
        int bestNbVoisins = -1;
        int bestNodeId = -1;
        for (int j=0;j<tailleMax;j++) {
            if (!marque[j]) {
                int nodeNbVoisinsPlaces = _noeuds[vecNode[j]].getVoisinsPlaces();
                if (nodeNbVoisinsPlaces > bestNbVoisinsPlaces) {
                    bestNbVoisinsPlaces = nodeNbVoisinsPlaces;
                    bestNbVoisins = _noeuds[vecNode[j]].voisins.size();
                    bestNodeId = j;
                }
                else if (nodeNbVoisinsPlaces == bestNbVoisinsPlaces) {
                    int nodeNbVoisins = _noeuds[vecNode[j]].voisins.size();
                    if (nodeNbVoisins > bestNbVoisins) {
                        bestNbVoisinsPlaces = nodeNbVoisinsPlaces;
                        bestNbVoisins = _noeuds[vecNode[j]].voisins.size();
                        bestNodeId = j;
                    }
                    else if (nodeNbVoisins == bestNbVoisins) {
                        nbRencontre++;
                        if (generateRand(nbRencontre) == 1) {
                            bestNbVoisinsPlaces = nodeNbVoisinsPlaces;
                            bestNbVoisins = _noeuds[vecNode[j]].voisins.size();
                            bestNodeId = j;
                        }
                    }
                }
            }
        }
        marque[bestNodeId] = true;
        int meilleurEmplacement = getMeilleurEmplacementScore(_noeuds[vecNode[bestNodeId]]);
        _noeuds[vecNode[bestNodeId]].setEmplacement(&_emplacements[meilleurEmplacement]);
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::completeBasicGloutonScoreGrille(std::vector<int>& vecNode, int tailleMax) {
    std::vector<bool> marque;
    marque.resize(tailleMax);
    for (int i=0;i<tailleMax;i++) {
        int nbRencontre = 0;
        int bestNbVoisinsPlaces = -1;
        int bestNbVoisins = -1;
        int bestNodeId = -1;
        for (int j=0;j<tailleMax;j++) {
            if (!marque[j]) {
                int nodeNbVoisinsPlaces = _noeuds[vecNode[j]].getVoisinsPlaces();
                if (nodeNbVoisinsPlaces > bestNbVoisinsPlaces) {
                    bestNbVoisinsPlaces = nodeNbVoisinsPlaces;
                    bestNbVoisins = _noeuds[vecNode[j]].voisins.size();
                    bestNodeId = j;
                }
                else if (nodeNbVoisinsPlaces == bestNbVoisinsPlaces) {
                    int nodeNbVoisins = _noeuds[vecNode[j]].voisins.size();
                    if (nodeNbVoisins > bestNbVoisins) {
                        bestNbVoisinsPlaces = nodeNbVoisinsPlaces;
                        bestNbVoisins = _noeuds[vecNode[j]].voisins.size();
                        bestNodeId = j;
                    }
                    else if (nodeNbVoisins == bestNbVoisins) {
                        nbRencontre++;
                        if (generateRand(nbRencontre) == 1) {
                            bestNbVoisinsPlaces = nodeNbVoisinsPlaces;
                            bestNbVoisins = _noeuds[vecNode[j]].voisins.size();
                            bestNodeId = j;
                        }
                    }
                }
            }
        }
        marque[bestNodeId] = true;
        int meilleurEmplacement = getMeilleurEmplacementScore(_noeuds[vecNode[bestNodeId]]);
        _noeuds[vecNode[bestNodeId]].setEmplacement(&_emplacements[meilleurEmplacement]);
        for (const int& areteId : _noeuds[vecNode[bestNodeId]]._aretes) {
            if (_aretes[areteId].estPlace()) {
                initAreteCellule(areteId);
            }
        }
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::completePlacementAleatoire() {
    #if defined(DEBUG_GRAPHE)
        std::cout << "Placement aleatoire" << std::endl;
    #endif
    for (int i = 0; i < _noeuds.size(); ++i) {
        if (_noeuds[i]._emplacement == nullptr) {
            int emplacementId = generateRand(_emplacements.size() - 1);
            while (!_emplacements[emplacementId].estDisponible()) {
                emplacementId = (emplacementId + 1) % _emplacements.size();
            }
            _noeuds[i].setEmplacement(&_emplacements[emplacementId]);
        }
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::completePlacementAleatoireScore(std::vector<int>& vecNode, int tailleMax) {
    for (int i=0;i<tailleMax;i++) {
        int emplacementId = generateRand(_emplacements.size() - 1);
        while (!_emplacements[emplacementId].estDisponible()) {
            emplacementId = (emplacementId + 1) % _emplacements.size();
        }
        _noeuds[vecNode[i]].setEmplacement(&_emplacements[emplacementId]);
    }
    isNombreCroisementUpdated = false;
    isNodeScoreUpdated = false;
    isIntersectionVectorUpdated = false;
}

void Graphe::stressMajorization(std::vector<std::vector<double>> customParam, int methode, bool useClosest) {
    if (!estPlace()) { placementAleatoire(); }
    for (int i=0;i<_noeuds.size();i++) {
        _noeuds[i].stressX = (double)_noeuds[i].getEmplacement()->getX();
        _noeuds[i].stressY = (double)_noeuds[i].getEmplacement()->getY();
    }
    if (_sm.G == nullptr) { _sm.G = this; }
    bool useGrille = _emplacements.size() >= _noeuds.size() * 2;
    #if defined(DEBUG_GRAPHE)
        if (useGrille) { std::cout << "Debut Stress Majorization avec grille.\n"; }
        else { std::cout << "Debut Stress Majorization sans grille.\n"; }
    #endif
    if (useGrille) {
        _sm.m_useGrille = true;
        initGrilleCarre();
        registerSlotsInGridNoMove();
    }
    else {
        _sm.m_useGrille = false;
    }
    _sm.m_useClosest = useClosest;
    if (customParam.size() > 0) {
        for (std::vector<double>& param : customParam) {
            if (param.size() > 0) {
                if (param[0] == 1) {
                    _sm.m_edgeCosts = param[1];
                }
                if (param[0] == 2) {
                    _sm.m_iterations = param[1];
                }
            }
        }
    }
    if (methode == 0) {
        _sm.runAlgo();
    }
    else if (methode == 1) {
        _sm.runAlgoDynStress();
    }
    else if (methode == 2) {
        _sm.runAlgoDynCross();
    }
    else if (methode == 3) {
        _sm.runAlgoDynDichStress();
    }
    if (useGrille) { deleteGrille(); }
    #if defined(DEBUG_GRAPHE)
        std::cout << "Fin Stress Majorization\n";
    #endif
}

void Graphe::stressMajorizationReel() {
    _sm.G = this;
    if ((_noeuds[0]._xreel == -12345)&&(_noeuds[0]._yreel == -12345)) { placementAleatoireReel(); }
    #if defined(DEBUG_GRAPHE)
        std::cout << "Debut Stress Majorization Reel.\n";
    #endif
    _sm.runAlgoReel();
    #if defined(DEBUG_GRAPHE)
        std::cout << "Fin Stress Majorization\n";
    #endif
}

void Graphe::stepStressMajorization(std::vector<std::vector<double>> customParam, int edgeCost) {
    if (_sm.G == nullptr) { 
        _sm.G = this;
        if (!estPlace()) { 
            placementAleatoire();
        }
        for (int i=0;i<_noeuds.size();i++) {
            _noeuds[i].stressX = (double)_noeuds[i].getEmplacement()->getX();
            _noeuds[i].stressY = (double)_noeuds[i].getEmplacement()->getY();
        }
        if (customParam.size() > 0) {
            for (std::vector<double>& param : customParam) {
                if (param.size() > 0) {
                    if (param[0] == 1) {
                        _sm.m_edgeCosts = param[1];
                    }
                }
            }
        }
        _sm.m_edgeCosts = edgeCost;
    }
    if (((int)_noeuds[0].stressX != _noeuds[0].getEmplacement()->getX())||((int)_noeuds[0].stressY != _noeuds[0].getEmplacement()->getY())) {
        for (int i=0;i<_noeuds.size();i++) {
            _noeuds[i].stressX = (double)_noeuds[i].getEmplacement()->getX();
            _noeuds[i].stressY = (double)_noeuds[i].getEmplacement()->getY();
        }
    }
    if (grillePtr.size() == 0) {
        bool useGrille = _emplacements.size() >= _noeuds.size() * 5;
        if (useGrille) {
            initGrilleCarre();
            registerSlotsInGridNoMove();
            _sm.m_useGrille = true;
        }
        else {
            _sm.m_useGrille = false;
        }
    }
    _sm.runStepAlgo();
}

void Graphe::placementPivotMDS(std::vector<std::vector<double>> customParam, int edgeCost, int nbPivot) {
    if (_pmds.G == nullptr) { _pmds.G = this; }
    _pmds.m_edgeCosts = edgeCost;
    _pmds.m_numberOfPivots = nbPivot;
    bool useGrille = _emplacements.size() > _noeuds.size() * 5;
    if (useGrille) {
        initGrilleCarre();
        registerSlotsInGridNoMove();
        _pmds.m_useGrille = true;
    }
    else {
        _pmds.m_useGrille = false;
    }
    _pmds.runAlgo();
    if (useGrille) { deleteGrille(); }
}
