#include "graphe.hpp"
#include "intersection.hpp"
#include "geometrie.hpp"
#include <iostream>

void Graphe::recalculateIllegalIntersections() {
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisent(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) {
                        areteIll.insert(&_aretes[i]);
                        areteIll.insert(&_aretes[j]);
                    }
                    else {
                        areteInter.insert(&_aretes[i]);
                        areteInter.insert(&_aretes[j]);
                    }
                }
            }
            else {
                if (surSegment(_aretes[i], *commonNodeEdges[j][i])) {
                    areteIllSelf.insert(&_aretes[i]);
                    areteIllSelf.insert(&_aretes[j]);
                }
                else if (surSegment(_aretes[j], *commonNodeEdges[i][j])) {
                    areteIllSelf.insert(&_aretes[i]);
                    areteIllSelf.insert(&_aretes[j]);
                }
            }
        }
    }
}

void Graphe::recalculateIllegalIntersectionsReel() {
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisentReel(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) {
                        areteIll.insert(&_aretes[i]);
                        areteIll.insert(&_aretes[j]);
                    }
                    else {
                        areteInter.insert(&_aretes[i]);
                        areteInter.insert(&_aretes[j]);
                    }
                }
            }
            else {
                if (surSegmentReel(_aretes[i], *commonNodeEdges[j][i])) {
                    areteIllSelf.insert(&_aretes[i]);
                    areteIllSelf.insert(&_aretes[j]);
                }
                else if (surSegmentReel(_aretes[j], *commonNodeEdges[i][j])) { 
                    areteIllSelf.insert(&_aretes[i]);
                    areteIllSelf.insert(&_aretes[j]);
                }
            }
        }
    }
}

// Calcule le score d'intersection du graphe et le met a jour.
long Graphe::getNbCroisement() {
    long total = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisent(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) { total += PENALITE_MAX; }
                    else { ++total; }
                }
            }
            else {
                if (surSegment(_aretes[i], *commonNodeEdges[j][i])) { total += PENALITE_MAX_SELF; }
                else if (surSegment(_aretes[j], *commonNodeEdges[i][j])) { total += PENALITE_MAX_SELF; }
            }
        }
    }
    nombreCroisement = total;
    isNombreCroisementUpdated = true;
    return total;
}

// Calcule le score d'intersection du graphe et le met a jour.
long Graphe::getNbCroisementReel() {
    long total = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisentReel(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) { total += PENALITE_MAX; }
                    else { ++total; }
                }
            }
            else {
                if (surSegmentReel(_aretes[i], *commonNodeEdges[j][i])) { total += PENALITE_MAX_SELF; }
                else if (surSegmentReel(_aretes[j], *commonNodeEdges[i][j])) { total += PENALITE_MAX_SELF; }
            }
        }
    }
    nombreCroisement = total;
    isNombreCroisementUpdated = true;
    return total;
}

// Calcule le score d'intersection pour un graphe qui n'est pas forcement entierement place
// Ne met pas a jour le nombre de croisement.
long Graphe::getNbCroisementGlouton() {
    long total = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        if (_aretes[i].estPlace()) {
            for (int j = i + 1; j < _aretes.size(); ++j) {
                if (_aretes[j].estPlace()) {
                    if (commonNodeEdges[i][j] == nullptr) {
                        bool isIllegal = false;
                        if (seCroisent(_aretes[i], _aretes[j],isIllegal)) {
                            if (isIllegal) { total += PENALITE_MAX; }
                            else { ++total; }
                        }
                    }
                    else {
                        if (surSegment(_aretes[i], *commonNodeEdges[j][i])) { total += PENALITE_MAX_SELF; }
                        else if (surSegment(_aretes[j], *commonNodeEdges[i][j])) { total += PENALITE_MAX_SELF; }
                    }
                }
            }
        }
    }
    return total;
}

// Calcule le score d'intersection pour un noeud dans un graphe qui n'est pas forcement entierement place
// Ne met pas a jour le nombre de croisement
long Graphe::getNbCroisementGloutonScore(int nodeId) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    for (const int& index : _noeuds[nodeId]._aretes) {
            if (_aretes[index].estPlace()) {
            for (int j = 0; j < _aretes.size(); ++j) {
                if (_aretes[j].estPlace()) {
                    if ((index != j) && (!indexPasse[j])) {
                        if (commonNodeEdges[index][j] == nullptr) {
                            bool isIllegal = false;
                            if (seCroisent(_aretes[index], _aretes[j],isIllegal)) {
                                if (isIllegal) { score += PENALITE_MAX; }
                                else { score++; }
                            }
                        }
                        else {
                            if (surSegment(_aretes[index], *commonNodeEdges[j][index])) { score += PENALITE_MAX_SELF; }
                            else if (surSegment(_aretes[j], *commonNodeEdges[index][j])) { score += PENALITE_MAX_SELF; }
                        }
                    }
                }
            }
            indexPasse[index] = true;
        }
    }
    return score;
}

// Calcule le score du noeud en fonction de ses vecteur d'intersections
long Graphe::getScoreCroisementNodeFromArrays(int nodeIndex) {
    long score = 0;
    std::vector<int> passedIndex;
    for (int i=0;i<_noeuds[nodeIndex]._aretes.size();i++) {
        score += _aretes[_noeuds[nodeIndex]._aretes[i]].intersections.size();
        score += _aretes[_noeuds[nodeIndex]._aretes[i]].intersectionsIll.size() * PENALITE_MAX;
        for (const int& idArray : _aretes[_noeuds[nodeIndex]._aretes[i]].intersectionsIllSelf) {
            if (!isInVector(passedIndex,idArray)) {
                score += PENALITE_MAX_SELF;
            }
        }
        passedIndex.push_back(_noeuds[nodeIndex]._aretes[i]);
    }
    return score;
}

// Calcule le score de l'enfant en fonction des aretes passées dans areteVec
long Graphe::getNodeScoreEnfant(Graphe& G, std::vector<int>& areteVec, int nodeIndex) {
    long score = 0;
    for (int i = 0; i < G._noeuds[nodeIndex]._aretes.size(); ++i) {
        int index = G._noeuds[nodeIndex]._aretes[i];
        if (G._aretes[index].estPlace()) {
            for (int j = 0; j < areteVec.size(); ++j) {
                int index2 = areteVec[j];
                if (!(G._aretes[index].contains(_aretes[index2].getNoeud1()) || G._aretes[index].contains(_aretes[index2].getNoeud2()))) {
                    bool isIllegal = false;
                    if (seCroisent(G._aretes[index], _aretes[index2],isIllegal)) {
                        if (isIllegal) {
                            score += PENALITE_MAX;
                        }
                        else {
                            score++;
                        }
                    }
                }
                else {
                    Noeud* nodeNotInCommon = _aretes[index2].nodeNotInCommon(&G._aretes[index]);
                    if (surSegment(G._aretes[index], *nodeNotInCommon)) {
                        score += PENALITE_MAX_SELF;
                    }
                    else {
                        nodeNotInCommon = G._aretes[index].nodeNotInCommon(&_aretes[index2]);
                        if (surSegment(_aretes[index2], *nodeNotInCommon)) {
                            score += PENALITE_MAX_SELF;
                        }
                    }
                }
            }
        }
    }
    for (int i=0;i<G._noeuds[nodeIndex]._aretes.size()-1;i++) {
        int index = G._noeuds[nodeIndex]._aretes[i];
        if (G._aretes[index].estPlace()) {
            for (int j=i+1;j<G._noeuds[nodeIndex]._aretes.size();j++) {
                int index2 = G._noeuds[nodeIndex]._aretes[j];
                if (G._aretes[index2].estPlace()) {
                    Noeud* nodeNotInCommon = G._aretes[index2].nodeNotInCommon(&G._aretes[index]);
                    if (surSegment(G._aretes[index], *nodeNotInCommon)) {
                        score += PENALITE_MAX_SELF;
                    }
                    else {
                        nodeNotInCommon = G._aretes[index].nodeNotInCommon(&G._aretes[index2]);
                        if (surSegment(G._aretes[index2], *nodeNotInCommon)) {
                            score += PENALITE_MAX_SELF;
                        }
                    }
                }
            }
        }
    }
    return score;
}

// Calcule le score de l'enfant en fonction des aretes passées dans areteVec
long Graphe::getNodeScoreEnfantGrille(Graphe& G, int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    for (int i = 0; i < G._noeuds[nodeIndex]._aretes.size(); ++i) {
        int index = G._noeuds[nodeIndex]._aretes[i];
        if (G._aretes[index].estPlace()) {
            std::vector<bool> indexPasseCellule(_aretes.size(),false);
            for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
                std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
                for (int k=0; k < vecId.size();k++) {
                    if ((index != vecId[k]) && (!indexPasse[vecId[k]]) && (!indexPasseCellule[vecId[k]])) {
                        if (!(G._aretes[index].contains(_aretes[vecId[k]].getNoeud1()) || G._aretes[index].contains(_aretes[vecId[k]].getNoeud2()))) {
                            bool isIllegal = false;
                            if (seCroisent(G._aretes[index], _aretes[vecId[k]],isIllegal)) {
                                if (isIllegal) {
                                    score += PENALITE_MAX;
                                }
                                else {
                                    score++;
                                }
                            }
                        }
                        else {
                            Noeud* nodeNotInCommon = _aretes[vecId[k]].nodeNotInCommon(&G._aretes[index]);
                            if (surSegment(G._aretes[index], *nodeNotInCommon)) {
                                score += PENALITE_MAX_SELF;
                            }
                            else {
                                nodeNotInCommon = G._aretes[index].nodeNotInCommon(&_aretes[vecId[k]]);
                                if (surSegment(_aretes[vecId[k]], *nodeNotInCommon)) {
                                    score += PENALITE_MAX_SELF;
                                }
                            }
                        }
                        indexPasseCellule[vecId[k]] = true;
                    }
                }
            }
            indexPasse[index] = true;
        }
    }
    for (int i=0;i<G._noeuds[nodeIndex]._aretes.size()-1;i++) {
        int index = G._noeuds[nodeIndex]._aretes[i];
        if (G._aretes[index].estPlace()) {
            for (int j=i+1;j<G._noeuds[nodeIndex]._aretes.size();j++) {
                int index2 = G._noeuds[nodeIndex]._aretes[j];
                if (G._aretes[index2].estPlace()) {
                    Noeud* nodeNotInCommon = G._aretes[index2].nodeNotInCommon(&G._aretes[index]);
                    if (surSegment(G._aretes[index], *nodeNotInCommon)) {
                        score += PENALITE_MAX_SELF;
                    }
                    else {
                        nodeNotInCommon = G._aretes[index].nodeNotInCommon(&G._aretes[index2]);
                        if (surSegment(G._aretes[index2], *nodeNotInCommon)) {
                            score += PENALITE_MAX_SELF;
                        }
                    }
                }
            }
        }
    }
    return score;
}

// Calcule le score du noeud en parametre.
long Graphe::getScoreCroisementNode(int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        for (int j = 0; j < _aretes.size(); ++j) {
            if ((index != j) && (!indexPasse[j])) {
                if (commonNodeEdges[index][j] == nullptr) {
                    bool isIllegal = false;
                    if (seCroisent(_aretes[index], _aretes[j],isIllegal)) {
                        if (isIllegal) { score += PENALITE_MAX; }
                        else { score++; }
                    }
                }
                else {
                    if (surSegment(_aretes[index], *commonNodeEdges[j][index])) { score += PENALITE_MAX_SELF; }
                    else if (surSegment(_aretes[j], *commonNodeEdges[index][j])) { score += PENALITE_MAX_SELF; }
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

// Calcule le score du noeud en parametre.
long Graphe::getScoreCroisementNodeReel(int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        for (int j = 0; j < _aretes.size(); ++j) {
            if ((index != j) && (!indexPasse[j])) {
                if (commonNodeEdges[index][j] == nullptr) {
                    bool isIllegal = false;
                    if (seCroisentReel(_aretes[index], _aretes[j],isIllegal)) {
                        if (isIllegal) { score += PENALITE_MAX; }
                        else { score++; }
                    }
                }
                else {
                    if (surSegmentReel(_aretes[index], *commonNodeEdges[j][index])) { score += PENALITE_MAX_SELF; }
                    else if (surSegmentReel(_aretes[j], *commonNodeEdges[index][j])) { score += PENALITE_MAX_SELF; }
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

// Calcule le score du noeud en parametre. Le graphe peut ne pas etre placé entierement
long Graphe::getScoreCroisementNodeGlouton(int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (_aretes[index].estPlace()) {
            for (int j = 0; j < _aretes.size(); ++j) {
                if (_aretes[j].estPlace()&&(index != j) && (!indexPasse[j])) {
                    if (commonNodeEdges[index][j] == nullptr) {
                        bool isIllegal = false;
                        if (seCroisent(_aretes[index], _aretes[j],isIllegal)) {
                            if (isIllegal) { score += PENALITE_MAX; }
                            else { score++; }
                        }
                    }
                    else {
                        if (surSegment(_aretes[index], *commonNodeEdges[j][index])) {
                            score += PENALITE_MAX_SELF;
                        }
                        else if (surSegment(_aretes[j], *commonNodeEdges[index][j])) { score += PENALITE_MAX_SELF; }
                    }
                }
            }
            indexPasse[index] = true;
        }
    }
    return score;
}

// Calcule le score du noeud nodeIndex sans ajouter le score produit par le noeud swapIndex.
long Graphe::getScoreCroisementNode(int nodeIndex, int swapIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (!_aretes[index].contains(swapIndex)) {
            for (int j = 0; j < _aretes.size(); ++j) {
                if ((index != j) && (!indexPasse[j])) {
                    if (!_aretes[j].contains(swapIndex)) {
                        if (commonNodeEdges[index][j] == nullptr) {
                            bool isIllegal = false;
                            if (seCroisent(_aretes[index], _aretes[j],isIllegal)) {
                                if (isIllegal) { score += PENALITE_MAX; }
                                else { score++; }
                            }
                        }
                        else {
                            if (surSegment(_aretes[index], *commonNodeEdges[j][index])) { score += PENALITE_MAX_SELF; }
                            else if (surSegment(_aretes[j], *commonNodeEdges[index][j])) { score += PENALITE_MAX_SELF; }
                        }
                    }
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

// Calcule le score du noeud en parametre.
long Graphe::getScoreCroisementNodeGrid(int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (index != _noeuds[nodeIndex]._aretes[0]) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
        for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
            std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
            for (const int& index2 : vecId) {
                if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2])) {
                    if (commonNodeEdges[index][index2] == nullptr) {
                        bool isIllegal = false;
                        if (seCroisent(_aretes[index], _aretes[index2],isIllegal)) {
                            if (isIllegal) { score += PENALITE_MAX; }
                            else { score++; }
                        }
                    }
                    else {
                        if (surSegment(_aretes[index], *commonNodeEdges[index2][index])) { score += PENALITE_MAX_SELF; }
                        else if (surSegment(_aretes[index2], *commonNodeEdges[index][index2])) { score += PENALITE_MAX_SELF; }
                    }
                    indexPasseCellule[index2] = true;
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

// Calcule le score du noeud en parametre.
long Graphe::getScoreCroisementNodeGridReel(int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (index != _noeuds[nodeIndex]._aretes[0]) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
        for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
            std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
            for (const int& index2 : vecId) {
                if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2])) {
                    if (commonNodeEdges[index][index2] == nullptr) {
                        bool isIllegal = false;
                        if (seCroisentReel(_aretes[index], _aretes[index2],isIllegal)) {
                            if (isIllegal) { score += PENALITE_MAX; }
                            else { score++; }
                        }
                    }
                    else {
                        if (surSegmentReel(_aretes[index], *commonNodeEdges[index2][index])) { score += PENALITE_MAX_SELF; }
                        else if (surSegmentReel(_aretes[index2], *commonNodeEdges[index][index2])) { score += PENALITE_MAX_SELF; }
                    }
                    indexPasseCellule[index2] = true;
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

long Graphe::getScoreCroisementNodeGridReelThread(int nodeIndex, bool isFirstThread) {
    int typeArreteToIgnore;
    if (isFirstThread) { typeArreteToIgnore = 2; }
    else { typeArreteToIgnore = 1; }
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (index != _noeuds[nodeIndex]._aretes[0]) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
        for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
            std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
            for (const int& index2 : vecId) {
                if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2]) && (_aretes[index2].typeArrete != typeArreteToIgnore)) {
                    if (!(_aretes[index].contains(_aretes[index2].getNoeud1()) || _aretes[index].contains(_aretes[index2].getNoeud2()))) {
                        bool isIllegal = false;
                        if (seCroisentReel(_aretes[index], _aretes[index2],isIllegal)) {
                            if (isIllegal) { score += PENALITE_MAX; }
                            else { score++; }
                        }
                    }
                    else {
                        Noeud* nodeNotInCommon = _aretes[index2].nodeNotInCommon(&_aretes[index]);
                        if (surSegmentReel(_aretes[index], *nodeNotInCommon)) { score += PENALITE_MAX_SELF; }
                        else {
                            nodeNotInCommon = _aretes[index].nodeNotInCommon(&_aretes[index2]);
                            if (surSegmentReel(_aretes[index2], *nodeNotInCommon)) { score += PENALITE_MAX_SELF; }
                        }
                    }
                    indexPasseCellule[index2] = true;
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

long Graphe::getScoreCroisementNodeGridReelNThread(int nodeIndex, int tid) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (index != _noeuds[nodeIndex]._aretes[0]) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
        for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
            std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
            for (const int& index2 : vecId) {
                if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2]) && ((_aretes[index2].typeArrete == 0)||(_aretes[index2].typeArrete == tid))) {
                    if (!(_aretes[index].contains(_aretes[index2].getNoeud1()) || _aretes[index].contains(_aretes[index2].getNoeud2()))) {
                        bool isIllegal = false;
                        if (seCroisentReel(_aretes[index], _aretes[index2],isIllegal)) {
                            if (isIllegal) { score += PENALITE_MAX; }
                            else { score++; }
                        }
                    }
                    else {
                        Noeud* nodeNotInCommon = _aretes[index2].nodeNotInCommon(&_aretes[index]);
                        if (surSegmentReel(_aretes[index], *nodeNotInCommon)) { score += PENALITE_MAX_SELF; }
                        else {
                            nodeNotInCommon = _aretes[index].nodeNotInCommon(&_aretes[index2]);
                            if (surSegmentReel(_aretes[index2], *nodeNotInCommon)) { score += PENALITE_MAX_SELF; }
                        }
                    }
                    indexPasseCellule[index2] = true;
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

// Calcule le score du noeud nodeIndex sans ajouter le score produit par le noeud swapIndex.
long Graphe::getScoreCroisementNodeGrid(int nodeIndex, int swapIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (!_aretes[index].contains(swapIndex)) {
            if (index != _noeuds[nodeIndex]._aretes[0]) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
            for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
                std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
                for (const int& index2 : vecId) {
                    if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2])) {
                        if (!_aretes[index2].contains(swapIndex)) {
                            if (commonNodeEdges[index][index2] == nullptr) {
                                bool isIllegal = false;
                                if (seCroisent(_aretes[index], _aretes[index2],isIllegal)) {
                                    if (isIllegal) { score += PENALITE_MAX; }
                                    else { score++; }
                                }
                            }
                            else {
                                if (surSegment(_aretes[index], *commonNodeEdges[index2][index])) { score += PENALITE_MAX_SELF; }
                                else if (surSegment(_aretes[index2], *commonNodeEdges[index][index2])) { score += PENALITE_MAX_SELF; }
                            }
                            indexPasseCellule[index2] = true;
                        }
                    }
                }
            }
        }
        indexPasse[index] = true;
    }
    return score;
}

// Calcule le score du noeud en parametre.
long Graphe::getScoreCroisementNodeGloutonGrid(int nodeIndex) {
    long score = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (const int& index : _noeuds[nodeIndex]._aretes) {
        if (_aretes[index].estPlace()) {
            if (index != _noeuds[nodeIndex]._aretes[0]) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
            for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
                std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
                for (const int& index2 : vecId) {
                    if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2])) {
                        if (commonNodeEdges[index][index2] == nullptr) {
                            bool isIllegal = false;
                            if (seCroisent(_aretes[index], _aretes[index2],isIllegal)) {
                                if (isIllegal) { score += PENALITE_MAX; }
                                else { score++; }
                            }
                        }
                        else {
                            if (surSegment(_aretes[index], *commonNodeEdges[index2][index])) { score += PENALITE_MAX_SELF; }
                            else if (surSegment(_aretes[index2], *commonNodeEdges[index][index2])) { score += PENALITE_MAX_SELF; }
                        }
                        indexPasseCellule[index2] = true;
                    }
                }
            }
            indexPasse[index] = true;
        }
    }
    return score;
}

// Indique aux aretes du noeuds que leur liste d'intersection n'est pas à jours.
void Graphe::changeUpdateValue(int idNode) {
    for (int i=0;i<_noeuds[idNode]._aretes.size();i++) {
        _aretes[_noeuds[idNode]._aretes[i]].isUpdated = false;
    }
}

// Recalcule les intersections d'une arete et met a jour sa liste d'intersection uniquement
void Graphe::recalculateInterArray(int idArray) {
    _aretes[idArray].clearIntersectionsVector();
    for (int i = 0; i < _aretes.size(); ++i) {
        if (i != idArray) {
            if (!(_aretes[idArray].contains(_aretes[i].getNoeud1()) || _aretes[idArray].contains(_aretes[i].getNoeud2()))) {
                bool isIllegal = false;
                if (seCroisent(_aretes[idArray], _aretes[i],isIllegal)) {
                    if (surSegment(_aretes[idArray], *_aretes[i].getNoeud1()) || surSegment(_aretes[idArray], *_aretes[i].getNoeud2())) {
                        _aretes[idArray].intersectionsIll.insert(i);
                    }
                    else if (surSegment(_aretes[i], *_aretes[idArray].getNoeud1()) || surSegment(_aretes[i], *_aretes[idArray].getNoeud2())) {
                        _aretes[idArray].intersectionsIll.insert(i);
                    }
                    else {
                        _aretes[idArray].intersections.insert(i);
                    }
                }
            }
            else {
                Noeud* nodeNotInCommon = _aretes[i].nodeNotInCommon(&_aretes[idArray]);
                if (surSegment(_aretes[idArray], *nodeNotInCommon)) {
                    _aretes[idArray].intersectionsIllSelf.insert(i);
                }
                else {
                    nodeNotInCommon = _aretes[idArray].nodeNotInCommon(&_aretes[i]);
                    if (surSegment(_aretes[i], *nodeNotInCommon)) {
                        _aretes[idArray].intersectionsIllSelf.insert(i);
                    }
                }
            }
        }
    }
}

// Met a jour le score du noeud et met a jour la liste d'intersection de ses aretes
// Modifie le score des noeuds affectes par la liste
void Graphe::updateNodeScore(int idNode) {
    for (int i=0;i<_noeuds[idNode]._aretes.size();i++) {
        int idOtherNode = _aretes[_noeuds[idNode]._aretes[i]].getNoeud1()->getId();
        if (idNode == idOtherNode) {
            idOtherNode = _aretes[_noeuds[idNode]._aretes[i]].getNoeud2()->getId();
        }
        if (!_aretes[_noeuds[idNode]._aretes[i]].isUpdated) {
            std::unordered_set<int> oldIntersections = _aretes[_noeuds[idNode]._aretes[i]].intersections;
            std::unordered_set<int> oldIntersectionsIll = _aretes[_noeuds[idNode]._aretes[i]].intersectionsIll;
            std::unordered_set<int> oldIntersectionsIllSelf = _aretes[_noeuds[idNode]._aretes[i]].intersectionsIllSelf;
            recalculateInterArray(_noeuds[idNode]._aretes[i]);
            for (const int& elem: oldIntersections) {
                if (_aretes[_noeuds[idNode]._aretes[i]].intersections.count(elem) == 0) {
                    _aretes[elem].getNoeud1()->score--;
                    _aretes[elem].getNoeud2()->score--;
                    _aretes[elem].intersections.erase(_noeuds[idNode]._aretes[i]);
                    _noeuds[idOtherNode].score--;
                    _noeuds[idNode].score--;
                    nombreCroisement--;
                }
            }
            for (const int& elem: _aretes[_noeuds[idNode]._aretes[i]].intersections) {
                if (oldIntersections.count(elem) == 0) {
                    _aretes[elem].getNoeud1()->score++;
                    _aretes[elem].getNoeud2()->score++;
                    _aretes[elem].intersections.insert(_noeuds[idNode]._aretes[i]);
                    _noeuds[idOtherNode].score++;
                    _noeuds[idNode].score++;
                    nombreCroisement++;
                }
            }
            for (const int& elem: oldIntersectionsIll) {
                if (_aretes[_noeuds[idNode]._aretes[i]].intersectionsIll.count(elem) == 0) {
                    _aretes[elem].getNoeud1()->score -= PENALITE_MAX;
                    _aretes[elem].getNoeud2()->score -= PENALITE_MAX;
                    _aretes[elem].intersectionsIll.erase(_noeuds[idNode]._aretes[i]);
                    _noeuds[idOtherNode].score -= PENALITE_MAX;
                    _noeuds[idNode].score -= PENALITE_MAX;
                    nombreCroisement -= PENALITE_MAX;
                }
            }
            for (const int& elem: _aretes[_noeuds[idNode]._aretes[i]].intersectionsIll) {
                if (oldIntersectionsIll.count(elem) == 0) {
                    _aretes[elem].getNoeud1()->score += PENALITE_MAX;
                    _aretes[elem].getNoeud2()->score += PENALITE_MAX;
                    _aretes[elem].intersectionsIll.insert(_noeuds[idNode]._aretes[i]);
                    _noeuds[idOtherNode].score += PENALITE_MAX;
                    _noeuds[idNode].score += PENALITE_MAX;
                    nombreCroisement += PENALITE_MAX;
                }
            }
            for (const int& elem: oldIntersectionsIllSelf) {
                if (_aretes[_noeuds[idNode]._aretes[i]].intersectionsIllSelf.count(elem) == 0) {
                    _aretes[elem].getNoeud1()->score -= PENALITE_MAX_SELF;
                    _aretes[elem].getNoeud2()->score -= PENALITE_MAX_SELF;
                    _aretes[elem].intersectionsIllSelf.erase(_noeuds[idNode]._aretes[i]);
                    _aretes[_noeuds[idNode]._aretes[i]].nodeNotInCommon(&_aretes[elem])->score -= PENALITE_MAX_SELF;
                    nombreCroisement -= PENALITE_MAX_SELF;
                }
            }
            for (const int& elem: _aretes[_noeuds[idNode]._aretes[i]].intersectionsIllSelf) {
                if (oldIntersectionsIllSelf.count(elem) == 0) {
                    _aretes[elem].getNoeud1()->score += PENALITE_MAX_SELF;
                    _aretes[elem].getNoeud2()->score += PENALITE_MAX_SELF;
                    _aretes[elem].intersectionsIllSelf.insert(_noeuds[idNode]._aretes[i]);
                    _aretes[_noeuds[idNode]._aretes[i]].nodeNotInCommon(&_aretes[elem])->score += PENALITE_MAX_SELF;
                    nombreCroisement += PENALITE_MAX_SELF;
                }
            }
            _aretes[_noeuds[idNode]._aretes[i]].isUpdated = true;
        }
    }
}

// Ne modifie pas le score
long Graphe::getNbCroisementConst() const {
    long total = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            //Aretes aretes1 = _aretes[i], aretes2 = _aretes[j];
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisent(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) { total += PENALITE_MAX; }
                    else { ++total; }
                }
            }
            else {
                if (surSegment(_aretes[i], *commonNodeEdges[j][i])) { total += PENALITE_MAX_SELF; }
                else if (surSegment(_aretes[j], *commonNodeEdges[i][j])) { total += PENALITE_MAX_SELF; }
            }
        }
    }
    return total;
}

// Ne modifie pas le score
long Graphe::getNbCroisementReelConst() const {
    long total = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisentReel(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) { total += PENALITE_MAX; }
                    else { ++total; }
                }
            }
            else {
                if (surSegmentReel(_aretes[i], *commonNodeEdges[j][i])) { total += PENALITE_MAX_SELF; }
                else if (surSegmentReel(_aretes[j], *commonNodeEdges[i][j])) { total += PENALITE_MAX_SELF; }
            }
        }
    }
    return total;
}

// Ne modifie pas le score
long Graphe::getNbCroisementDiff() {
    nombreInter = 0;
    nombreInterIll = 0;
    nombreInterIllSelf = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisent(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) { nombreInterIll++; }
                    else { nombreInter++; }
                }
            }
            else {
                if (surSegment(_aretes[i], *commonNodeEdges[j][i])) { nombreInterIllSelf++; }
                else if (surSegment(_aretes[j], *commonNodeEdges[i][j])) {nombreInterIllSelf++; }
            }
        }
    }
    return nombreInter+nombreInterIll+nombreInterIllSelf;
}

// Ne modifie pas le score
long Graphe::getNbCroisementDiffReel() {
    nombreInter = 0;
    nombreInterIll = 0;
    nombreInterIllSelf = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisentReel(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) { nombreInterIll++; }
                    else { nombreInter++; }
                }
            }
            else {
                if (surSegmentReel(_aretes[i], *commonNodeEdges[j][i])) { nombreInterIllSelf++; }
                else if (surSegmentReel(_aretes[j], *commonNodeEdges[i][j])) { nombreInterIllSelf++; }
            }
        }
    }
    return nombreInter+nombreInterIll+nombreInterIllSelf;
}

long Graphe::getNbCroisementDiffGrid() {
    nombreInter = 0;
    nombreInterIll = 0;
    nombreInterIllSelf = 0;
    std::vector<bool> indexPasse(_aretes.size(),false);
    std::vector<bool> indexPasseCellule(_aretes.size(),false);
    for (int index = 0; index < _aretes.size() - 1; ++index) {
        if (index != 0) { indexPasseCellule.assign(indexPasseCellule.size(), false); }
        for (int j = 0; j < _aretes[index].vecIdCellules.size(); ++j) {
            std::vector<int>& vecId = grillePtr[_aretes[index].vecIdCellules[j]]->vecAreteId;
            for (const int& index2 : vecId) {
                if ((index != index2) && (!indexPasse[index2]) && (!indexPasseCellule[index2])) {
                    if (commonNodeEdges[index][index2] == nullptr) {
                        bool isIllegal = false;
                        if (seCroisent(_aretes[index], _aretes[index2],isIllegal)) {
                            if (isIllegal) { nombreInterIll++; }
                            else { nombreInter++; }
                        }
                    }
                    else {
                        if (surSegment(_aretes[index], *commonNodeEdges[index2][index])) { nombreInterIllSelf++; }
                        else if (surSegment(_aretes[index2], *commonNodeEdges[index][index2])) { nombreInterIllSelf++; }
                    }
                    indexPasseCellule[index2] = true;
                }
            }
        }
        indexPasse[index] = true;
    }
    return nombreInter+nombreInterIll+nombreInterIllSelf;
}

long Graphe::getNbCroisementOldMethodConst() const {
    long total = 0;
    for (int i = 0; i < _aretes.size() - 1; ++i)
    {
        for (int j = i + 1; j < _aretes.size(); ++j)
        {
            //Aretes aretes1 = _aretes[i], aretes2 = _aretes[j];
            if (!(_aretes[i].contains(_aretes[j].getNoeud1())
                || _aretes[i].contains(_aretes[j].getNoeud2())))
            {
                if (oldSeCroisent(_aretes[i], _aretes[j]))
                {
                    if (oldSurSegment(_aretes[i], *_aretes[j].getNoeud1()) || surSegment(_aretes[i], *_aretes[j].getNoeud2()))
                    {
                        total += PENALITE_MAX;
                    }
                    else if (oldSurSegment(_aretes[j], *_aretes[i].getNoeud1()) || surSegment(_aretes[j], *_aretes[i].getNoeud2())) {
                        total += PENALITE_MAX;
                    }
                    else {
                        ++total;
                    }
                }
            }
            else {
                Noeud* nodeNotInCommon = _aretes[j].nodeNotInCommon(&_aretes[i]);
                if (oldSurSegment(_aretes[i], *nodeNotInCommon))
                {
                    total += PENALITE_MAX_SELF;
                }
                else {
                    nodeNotInCommon = _aretes[i].nodeNotInCommon(&_aretes[j]);
                    if (oldSurSegment(_aretes[j], *nodeNotInCommon)) {
                        total += PENALITE_MAX_SELF;
                    }
                }
            }
        }
    }
    return total;
}

// Ne met pas a jour le nombre de croisement du graphe
long Graphe::getNbCroisementArray() {
    long total = 0;
    for (int i=0;i<_aretes.size();i++) {
        total += _aretes[i].intersections.size();
        total += _aretes[i].intersectionsIll.size() * PENALITE_MAX;
        total += _aretes[i].intersectionsIllSelf.size() * PENALITE_MAX_SELF;
    }
    return total/2;
}

// Met a jour le score du graphe et de ses noeuds ainsi que les vecteurs d'intersections des aretes
void Graphe::initGraphAndNodeScoresAndCrossings() {
    long total = 0;
    for (int i=0;i<_aretes.size();i++) {
        _aretes[i].clearIntersectionsVector();
        _aretes[i].isUpdated = true;
    }
    for (int i = 0; i < _aretes.size() - 1; ++i) {
        for (int j = i + 1; j < _aretes.size(); ++j) {
            if (commonNodeEdges[i][j] == nullptr) {
                bool isIllegal = false;
                if (seCroisent(_aretes[i], _aretes[j],isIllegal)) {
                    if (isIllegal) {
                        total += PENALITE_MAX;
                        _aretes[i].intersectionsIll.insert(j);
                        _aretes[j].intersectionsIll.insert(i);
                    }
                    else {
                        ++total;
                        _aretes[i].intersections.insert(j);
                        _aretes[j].intersections.insert(i);
                    }
                }
            }
            else {
                if (surSegment(_aretes[i], *commonNodeEdges[j][i])) {
                    total += PENALITE_MAX_SELF;
                    _aretes[i].intersectionsIllSelf.insert(j);
                    _aretes[j].intersectionsIllSelf.insert(i);
                }
                else if (surSegment(_aretes[j], *commonNodeEdges[i][j])) {
                    total += PENALITE_MAX_SELF;
                    _aretes[i].intersectionsIllSelf.insert(j);
                    _aretes[j].intersectionsIllSelf.insert(i);
                }
            }
        }
    }
    nombreCroisement = total;
    isNombreCroisementUpdated = true;
    isNodeScoreUpdated = true;
    isIntersectionVectorUpdated = true;
    for (int i=0;i<_noeuds.size();i++) {
        _noeuds[i].score = getScoreCroisementNodeFromArrays(i);
        int nombreVoisin = _noeuds[i]._aretes.size();
        if (nombreVoisin > maxVoisin) {
            maxVoisin = nombreVoisin;
        }
    }
    //std::cout << "Max Voisin dans le graphe: " << maxVoisin << std::endl;
}

// Renvoie le score du noeud nodeId1 sans prendre en compte le score d'intersection avec le noeud nodeId2
long Graphe::getNodeLinkedScore(int nodeId1, int nodeId2) {
    long score = _noeuds[nodeId1].score;
    for (const int& node1AreteId : _noeuds[nodeId1]._aretes) {
        for (const int& node2AreteId : _noeuds[nodeId2]._aretes) {
            if (_aretes[node1AreteId].intersections.count(node2AreteId) > 0) {
                score--;
            }
            if (_aretes[node1AreteId].intersectionsIll.count(node2AreteId) > 0) {
                score -= PENALITE_MAX;
            }
            if (_aretes[node1AreteId].intersectionsIllSelf.count(node2AreteId) == 0) {
                score -= PENALITE_MAX_SELF;
            }
        }
    }
    return score;
}
