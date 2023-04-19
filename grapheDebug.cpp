#include "graphe.hpp"
#include <iostream>

void Graphe::afficherLiens(std::string nom) {
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Affichage DEBUG Aretes: " << nomGraphe << " " << nom << std::endl;
    std::cout << "Nb Aretes: " << _aretes.size() << std::endl;
    for (int i = 0; i < _aretes.size(); i++) {
        std::cout << "id: " << _aretes[i]._id << " idnode1: " << _aretes[i].getNoeud1()->getId() << " idnode2: " << _aretes[i].getNoeud2()->getId();
        if (!useCoordReel) std::cout << " x1: " << _aretes[i].getNoeud1()->getX() << " y1: " << _aretes[i].getNoeud1()->getY();
        if (!useCoordReel) std::cout << " x2: " << _aretes[i].getNoeud2()->getX() << " y2: " << _aretes[i].getNoeud2()->getY();
        std::cout << std::endl;
    }
    std::cout << "-----------------------------------------------" << std::endl;
}

void Graphe::afficherLiensEmp(std::string nom) {
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Affichage DEBUG Aretes: " << nomGraphe << " " << nom << std::endl;
    std::cout << "Nb Aretes: " << _aretes.size() << std::endl;
    for (int i = 0; i < _aretes.size(); i++) {
        std::cout << "id: " << _aretes[i]._id << " idemp1: " << _aretes[i].getNoeud1()->getEmplacement()->_id << " idemp2: " << _aretes[i].getNoeud2()->getEmplacement()->_id;
        std::cout << std::endl;
    }
    std::cout << "-----------------------------------------------" << std::endl;
}

void Graphe::afficherNoeuds(std::string nom) {
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Affichage DEBUG Noeuds: " << nomGraphe << " " << nom << std::endl;
    std::cout << "Nb Noeuds: " << _noeuds.size() << std::endl;
    for (int i = 0; i < _noeuds.size(); i++) {
        std::cout << "id: " << _noeuds[i].getId() << " empid: ";
        if (_noeuds[i].getEmplacement() == nullptr) {
            std::cout << " aucun" << " voisins: " << _noeuds[i].voisinString() <<  std::endl;
        }
        else {
            std::cout << _noeuds[i].getEmplacement()->_id << " x: " << _noeuds[i].getX() << " y: " << _noeuds[i].getY() << " score: " << _noeuds[i].score << " voisins: " << _noeuds[i].voisinString() << std::endl;
        }
    }
    std::cout << "-----------------------------------------------" << std::endl;
}

void Graphe::afficherNoeudsReel(std::string nom) {
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Affichage DEBUG Noeuds: " << nomGraphe << " " << nom << std::endl;
    std::cout << "Nb Noeuds: " << _noeuds.size() << std::endl;
    for (int i = 0; i < _noeuds.size(); i++) {
        std::cout << "id: " << _noeuds[i].getId() << " xreel: " << _noeuds[i]._xreel << " yreel: " << _noeuds[i]._yreel << std::endl;
    }
    std::cout << "-----------------------------------------------" << std::endl;
}

void Graphe::afficherEmplacement(std::string nom) {
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Affichage DEBUG Emplacements: " << nomGraphe << " " << nom << std::endl;
    std::cout << "Nb Emplacements: " << _emplacements.size() << std::endl;
    for (int i = 0; i < _emplacements.size(); i++) {
        std::cout << "id: " << _emplacements[i]._id << " x: " << _emplacements[i].getX() << " y: " << _emplacements[i].getY() << " idnode: ";
        if (_emplacements[i]._noeud == nullptr) {
            std::cout << "aucun: " << _emplacements[i].estDisponible() << std::endl;
        }
        else {
            std::cout << _emplacements[i]._noeud->getId() << std::endl;
        }
    }
    std::cout << "-----------------------------------------------" << std::endl;
}

void Graphe::afficherInfo(std::string nom) {
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Information sur le graphe: " << nomGraphe << " " << nom << std::endl;
    std::cout << "Nombre de noeud: " << _noeuds.size() << std::endl;
    std::cout << "Nombre d'emplacement: " << _emplacements.size() << std::endl;
    std::cout << "Nombre d'aretes: " << _aretes.size() << std::endl;
    std::cout << "Max Voisin: " << maxVoisin << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
}

int Graphe::afficherAreteDouble(bool display, std::string nom) {
    int retour = 1;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Arrete Double: " << nomGraphe << " " << nom << std::endl;
    }
    int nbAreteDouble = 0;
    for (int i = 0; i < _aretes.size(); i++) {
        int id1 = _aretes[i].getNoeud1()->getId();
        int id2 = _aretes[i].getNoeud2()->getId();
        for (int j = i + 1; j < _aretes.size(); j++) {
            int id12 = _aretes[j].getNoeud1()->getId();
            int id22 = _aretes[j].getNoeud2()->getId();
            if (((id1 == id12) && (id2 == id22)) || ((id1 == id22) && (id2 == id12))) {
                std::cout << "Arete double: " << i << " & " << j << " Noeud A1: " << id1 << " & " << id2 << " Noeud A2: " << id12 << " & " << id22 << std::endl;
                nbAreteDouble++;
            }
        }
    }
    if (nbAreteDouble == 0) {
        if (display) { std::cout << "Aucune" << std::endl; }
        retour=0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::afficherNoeudDouble(bool display, std::string nom) {
    int retour = 2;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Noeud Double: " << nomGraphe << " " << nom << std::endl;
    }
    int nbNoeudDouble = 0;
    for (int i = 0; i < _noeuds.size(); i++) {
        if (_noeuds[i].getEmplacement() != nullptr) {
            int id1 = _noeuds[i].getEmplacement()->getId();
            for (int j = i + 1; j < _noeuds.size(); j++) {
                if (_noeuds[j].getEmplacement() != nullptr) {
                    int id2 = _noeuds[j].getEmplacement()->getId();
                    if (id1 == id2) {
                        std::cout << "Noeud double: " << i << " & " << j << " Emplacement: " << id1 << std::endl;
                        nbNoeudDouble++;
                    }
                }
            }
        }
    }
    if (nbNoeudDouble == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour=0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::afficherEmplacementDouble(bool display, std::string nom) {
    int retour = 4;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Emplacement Double: " << nomGraphe << " " << nom << std::endl;
    }
    std::vector<std::vector<int>> grilleEmp;
    for (int i=0;i<=gridHeight;i++) {
        std::vector<int> grilleTmp(gridWidth+1,-1);
        grilleEmp.push_back(grilleTmp);
    }
    int nbEmplacementDouble = 0;
    for (int i=0;i<_emplacements.size();i++) {
        int x = _emplacements[i].getX();
        int y = _emplacements[i].getY();
        if (grilleEmp[y][x] == -1) { grilleEmp[y][x] = i; }
        else {
            std::cout << "Emplacement double: " << grilleEmp[y][x] << " & " << i << std::endl;
            nbEmplacementDouble++;
        }
    }
    if (nbEmplacementDouble == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour=0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::afficherNoeudSeul(bool display, std::string nom) {
    int retour = 10;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Noeud Seul: " << nomGraphe << " " << nom << std::endl;
    }
    int nbNoeudSeul = 0;
    for (int i = 0; i < _noeuds.size(); i++) {
        if (_noeuds[i]._aretes.size() == 0) {
            std::cout << "Noeud Seul: " << i << std::endl;
            nbNoeudSeul++;
        }
    }
    if (nbNoeudSeul == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugNoeudNonPlace(bool display, std::string nom) {
    int retour = 20;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Noeud non place: " << nomGraphe << " " << nom << std::endl;
    }
    long nbFail = 0;
    for (int i = 0; i < _noeuds.size(); i++) {
        if (_noeuds[i].getEmplacement() == nullptr) {
            std::cout << "Noeud non place: " << i << std::endl;
            nbFail++;
        }
    }
    if (nbFail == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugDesyncNoeudEmplacement(bool display, std::string nom) {
    int retour = 40;
    if (display) { 
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Sync Noeud Emplacement: " << nomGraphe << " " << nom << std::endl;
    }
    long nbWrongSync = 0;
    for (int i=0;i<_noeuds.size();i++) {
        if (_noeuds[i]._emplacement != nullptr) {
            if (_noeuds[i]._id != _emplacements[_noeuds[i]._emplacement->_id]._noeud->_id) {
                std::cout << "Sync Noeud: " << i << " NodeEmpId: " << _noeuds[i]._emplacement->_id << " EmpNodeId: " << _emplacements[_noeuds[i]._emplacement->_id]._noeud->_id << std::endl;
                nbWrongSync++;
            }
        }
    }
    for (int i=0;i<_emplacements.size();i++) {
        if (!_emplacements[i].estDisponible()) {
            if (_emplacements[i]._id != _noeuds[_emplacements[i]._noeud->_id]._emplacement->_id) {
                std::cout << "Sync Emplacement: " << i << " EmpNodeId: " << _emplacements[_noeuds[i]._emplacement->_id]._noeud->_id << " NodeEmpId: " << _noeuds[i]._emplacement->_id << std::endl;
                nbWrongSync++;
            }
        }
    }
    if (nbWrongSync == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugOldCroisement(bool display, std::string nom) {
    int retour = 100;
    if (display) { 
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Score Methode Graphe: " << nomGraphe << " " << nom << std::endl;
    }
    bool scoreFaux = true;
    long scoreActuel = getNbCroisementConst();
    long scoreOld = getNbCroisementOldMethodConst();
    if (scoreActuel != scoreOld) {
        std::cout << "Actuel: " << scoreActuel << " old: " << scoreOld << std::endl;
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
        scoreFaux = false;
    }
    if (scoreFaux) {
        if (display) { std::cout << "Score Correct" << std::endl; }
        retour = 0;
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugScoreNoeud(bool display, std::string nom) {
    int retour = 200;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Score Noeud: " << nomGraphe << " " << nom << std::endl;
    }
    long nbWrongScore = 0;
    for (int i = 0; i < _noeuds.size(); i++) {
        if (_noeuds[i].getEmplacement() != nullptr) {
            long scoreReel = getScoreCroisementNode(i);
            long scoreArrays = getScoreCroisementNodeFromArrays(i);
            if ((scoreReel != _noeuds[i].score)||(scoreReel != scoreArrays)) {
                std::cout << "Noeud: " << i << " Score:" << _noeuds[i].score << " Score Reel: " << scoreReel << " Score Arrays: " << scoreArrays << std::endl;
                nbWrongScore++;
            }
        }
    }
    if (nbWrongScore == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugScoreGraphe(bool display, bool useArray, std::string nom) {
    int retour = 400;
    if (display) { 
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Score Graphe: " << nomGraphe << " " << nom << std::endl;
    }
    bool scoreFaux = true;
    long scoreReel;
    if (useCoordReel) { scoreReel = getNbCroisementReelConst(); }
    else { scoreReel = getNbCroisementConst(); }

    if (useArray) {
        long scoreArray = getNbCroisementArray();
        if ((nombreCroisement != scoreReel)||(nombreCroisement != scoreArray)) {
            std::cout << "Croisement: " << nombreCroisement << " reel: " << scoreReel << " array: " << scoreArray << std::endl;
            if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
            scoreFaux = false;
        }
    }
    else {
        if (nombreCroisement != scoreReel) {
            std::cout << "Croisement: " << nombreCroisement << " reel: " << scoreReel << std::endl;
            if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
            scoreFaux = false;
        }
    }
    if (scoreFaux) {
        if (display) { std::cout << "Score Correct" << std::endl; }
        retour = 0;
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugInterArrays(bool display, std::string nom) {
    int retour = 1000;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG Inter Arrays: " << nomGraphe << " " << nom << std::endl;
    }
    long nbWrongArray = 0;
    for (int i = 0; i < _aretes.size(); i++) {
        for (const int& idOtherArray : _aretes[i].intersections) {
            if (_aretes[idOtherArray].intersections.count(i) == 0) {
                std::cout << "Inter: " << i << "&" << idOtherArray << std::endl;
                nbWrongArray++;
            }
            if (idOtherArray == i) {
                std::cout << "Inter: " << i << " self in array: " << idOtherArray << std::endl;
                nbWrongArray++;
            }
        }
        for (const int& idOtherArray : _aretes[i].intersectionsIll) {
            if (_aretes[idOtherArray].intersectionsIll.count(i) == 0) {
                std::cout << "InterIll: " << i << "&" << idOtherArray << std::endl;
                nbWrongArray++;
            }
            if (idOtherArray == i) {
                std::cout << "InterIll: " << i << " self in array: " << idOtherArray << std::endl;
                nbWrongArray++;
            }
        }
        for (const int& idOtherArray : _aretes[i].intersectionsIllSelf) {
            if (_aretes[idOtherArray].intersectionsIllSelf.count(i) == 0) {
                std::cout << "InterIllSelf: " << i << "&" << idOtherArray << std::endl;
                nbWrongArray++;
            }
            if (idOtherArray == i) {
                std::cout << "InterIllSelf: " << i << " self in array: " << idOtherArray << std::endl;
                nbWrongArray++;
            }
        }
    }
    if (nbWrongArray == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugDesyncEmplacementCell(bool display, std::string nom) {
    int retour = 2000;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG desync emplacement cellule: " << nomGraphe << " " << nom << std::endl;
    }
    long nbFail = 0;
    for (int i = 0; i < _emplacements.size(); i++) {
        int id = _emplacements[i].idCellule;
        if (id != -1) {
            if (!isInVector(grillePtr[id]->vecEmplacementId,i)) {
                std::cout << "Emplacement: " << i << " Cellule: " << id << std::endl;
                nbFail++;
            }
        }
    }
    for (int i=0;i<grillePtr.size();i++) {
        for (const int& id : grillePtr[i]->vecEmplacementId) {
            if (_emplacements[id].idCellule != i) {
                std::cout << "Cellule: " << i << " Emplacement: " << id << std::endl;
                nbFail++;
            }
        }
    }
    if (nbFail == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugDesyncAreteCell(bool display, std::string nom) {
    int retour = 4000;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Affichage DEBUG desync emplacement cellule: " << nomGraphe << " " << nom << std::endl;
    }
    long nbFail = 0;
    for (int i = 0; i < _aretes.size(); i++) {
        for (const int& id : _aretes[i].vecIdCellules) {
            if (!isInVector(grillePtr[id]->vecAreteId,i)) {
                std::cout << "Arete: " << i << " Cellule: " << id << std::endl;
                nbFail++;
            }
        }
    }
    for (int i=0;i<grillePtr.size();i++) {
        for (const int& id : grillePtr[i]->vecAreteId) {
            if (!isInVector(_aretes[id].vecIdCellules,i)) {
                std::cout << "Cellule: " << i << " Arete: " << id << std::endl;
                nbFail++;
            }
        }
    }
    if (nbFail == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugOrdreIdVector(bool display, std::string nom) {
    int retour = 10000;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Debug Ordre Id Noeud Arete Emplacement: " << nomGraphe << " " << nom << std::endl;
    }
    long nbWrongId = 0;
    for (int i = 0; i < _noeuds.size(); i++) {
        if (i!=_noeuds[i]._id) {
            std::cout << "i: " << i << " Noeud Id: " << _noeuds[i]._id << std::endl;
            nbWrongId++;
        }
    }
    for (int i=0;i<_aretes.size();i++) {
        if (i!=_aretes[i]._id) {
            std::cout << "i: " << i << " Arete Id: " << _aretes[i]._id << std::endl;
            nbWrongId++;
        }
    }
    for (int i=0;i<_emplacements.size();i++) {
        if (i!=_emplacements[i]._id) {
            std::cout << "i: " << i << " Emplacement Id: " << _emplacements[i]._id << std::endl;
            nbWrongId++;
        }
    }
    if (nbWrongId == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugSelfLoop(bool display, std::string nom) {
    int retour = 20000;
    if (display) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "Debug Self Loop: " << nomGraphe << " " << nom << std::endl;
    }
    long nbLoop = 0;
    for (int i = 0; i < _noeuds.size(); i++) {
        for (const Noeud* voisin : _noeuds[i].voisins) {
            if (voisin->_id == _noeuds[i]._id) {
                std::cout << "Self Loop Node Id: " << voisin->_id << std::endl;
                nbLoop++;
            }
        }
    }
    if (nbLoop == 0) {
        if (display) { std::cout << "Aucun" << std::endl; }
        retour = 0;
    }
    else {
        if (!display) { std::cout << "-----------------------------------------------" << std::endl; }
    }
    if (display) { std::cout << "-----------------------------------------------" << std::endl; }
    return retour;
}

int Graphe::debugEverything(bool displayOther, bool displaySelf) {
    int total = 0;
    #if defined(DEBUG)
    if (displaySelf) { std::cout << "Debut debug arete double\n"; }
    total += afficherAreteDouble(displayOther);
    if (!useCoordReel) {
        if (displaySelf) { std::cout << "Debut debug noeud double\n"; }
        total += afficherNoeudDouble(displayOther);
        if (displaySelf) { std::cout << "Debut debug emplacement double\n"; }
        total += afficherEmplacementDouble(displayOther);
        if (displaySelf) { std::cout << "Debut debug noeud seul\n"; }
        total += afficherNoeudSeul(displayOther);
        if (displaySelf) { std::cout << "Debut debug noeud non place\n"; }
        total += debugNoeudNonPlace(displayOther);
        if (displaySelf) { std::cout << "Debut debug desync noeud emplacement\n"; }
        total += debugDesyncNoeudEmplacement(displayOther);
        if (displaySelf) { std::cout << "Debut debug methode intersection\n"; }
        total += debugOldCroisement(displayOther);
    }
    if (isNodeScoreUpdated) {
        if (displaySelf) { std::cout << "Debut debug score noeud\n"; }
        total += debugScoreNoeud(displayOther);
    }
    if (isNombreCroisementUpdated) {
        if (isIntersectionVectorUpdated) {
            if (displaySelf) { std::cout << "Debut debug score graphe avec array\n"; }
            total += debugScoreGraphe(displayOther,true);
        }
        else {
            if (displaySelf) { std::cout << "Debut debug score graphe\n"; }
            total += debugScoreGraphe(displayOther);
        }
    }
    if (isIntersectionVectorUpdated) {
        if (displaySelf) { std::cout << "Debut debug inter array\n"; }
        total += debugInterArrays(displayOther);
    }
    if (grille.size() > 0) {
        if (!useCoordReel) {
            if (displaySelf) { std::cout << "Debut debug desync emplacement cellule\n"; }
            total += debugDesyncEmplacementCell(displayOther);
        }
        if (displaySelf) { std::cout << "Debut debug desync arete cellule\n"; }
        total += debugDesyncAreteCell(displayOther);
    }
    if (displaySelf) { std::cout << "Debug Ordre Id Noeud Arete Emplacement\n"; }
    total += debugOrdreIdVector(displayOther);
    if (displaySelf) { std::cout << "Debug Self Loop\n"; }
    total += debugSelfLoop(displayOther);
    if (displaySelf) { std::cout << "Debug fini\n"; }
    #endif
    return total;
}