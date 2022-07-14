#ifndef CALCEDGELENGTH_HPP
#define CALCEDGELENGTH_HPP
#include <ogdf/basic/GridLayout.h>
#include "NodeBend.hpp"
#include "EdgeMap.hpp"

using namespace ogdf;

double calcEdgeLength(const edge& e, const GridLayout& GL) {
    node source = e->source();
    node target = e->target();
    double length = 0.0;
    double sourceX = GL.x(source);
    double sourceY = GL.y(source);
    double targetX, targetY;
    IPolyline bends = GL.bends(e);
    if (bends.size() > 0) {
        for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
            targetX = (*i).m_x;
            targetY = (*i).m_y;
            //alternantive ???
            //if (sourceX == targetX) { length += abs(targetY - sourceY); }
            //else if (sourceY == sourceX) { length += abs(targetX - sourceX); }
            //else
            length += sqrt(pow((targetX - sourceX), 2) + pow((targetY - sourceY), 2));
            sourceX = targetX;
            sourceY = targetY;
        }
    }
    targetX = GL.x(target);
    targetY = GL.y(target);
    //alternantive ???
    //if (sourceX == targetX) { length += abs(targetY - sourceY); }
    //else if (sourceY == sourceX) { length += abs(targetX - sourceX); }
    //else
    length += sqrt(pow((targetX - sourceX), 2) + pow((targetY - sourceY), 2));
    return length;
}

double calcTmpEdgeLength(const adjEntry& ad, int srcX, int srcY, const GridLayout& GL) {
    edge e = ad->theEdge();
    // Si la source de l'edge est la source de l'adjEntry
    int sourceX, sourceY;
    node source = e->source();
    if (source == ad->theNode()) {
        sourceX = srcX;
        sourceY = srcY;
    }
    else {
        sourceX = GL.x(source);
        sourceY = GL.y(source);
    }
    double length = 0.0;
    int targetX, targetY;
    IPolyline bends = GL.bends(e);
    if (bends.size() > 0) {
        for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
            targetX = (*i).m_x;
            targetY = (*i).m_y;
            length += sqrt(pow((targetX - sourceX), 2) + pow((targetY - sourceY), 2));
            sourceX = targetX;
            sourceY = targetY;
        }
    }
    node target = e->target();
    // Si la target de l'edge est le noeud qu'on deplace
    if (target == ad->theNode()) {
        targetX = srcX;
        targetY = srcY;
    }
    else {
        targetX = GL.x(target);
        targetY = GL.y(target);
    }
    length += sqrt(pow((targetX - sourceX), 2) + pow((targetY - sourceY), 2));
    return length;
}

double calcTmpEdgeLengthBends(const edge& e, NodeBend* n, int bendX, int bendY, const GridLayout& GL) {
    node source = e->source();
    node target = e->target();
    double length = 0.0;
    int sourceX = GL.x(source);
    int sourceY = GL.y(source);
    int targetX, targetY;
    IPolyline bends = GL.bends(e);
    if (bends.size() > 0) {
        int k = 0;
        for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++,k++) {
            if (k == n->numero) {
                targetX = bendX;
                targetY = bendY;
            }
            else {
                targetX = (*i).m_x;
                targetY = (*i).m_y;
            }
            length += sqrt(pow((targetX - sourceX), 2) + pow((targetY - sourceY), 2));
            sourceX = targetX;
            sourceY = targetY;
        }
    }
    targetX = GL.x(target);
    targetY = GL.y(target);
    length += sqrt(pow((targetX - sourceX), 2) + pow((targetY - sourceY), 2));
    return length;
}

// Recupere la somme des longueurs autour d'un noeud apres un déplacement
double totalLengthAroundNodeBend(NodeBend* n, GridLayout& GL, int srcX, int srcY) {
    double totalLength = 0;
    SListPure<adjEntry> adjEntries;
    if (n->isNode) {
        n->getNode()->allAdjEntries(adjEntries);
    }
    else {
        adjEntries.pushBack(n->getAdjEntry());
    }
    // Si le deplacement est aucun déplacement on récupere juste les données dans la map
    if ((srcX != n->getX()) || (srcY != n->getY())) {
        for (auto it = adjEntries.begin(); it.valid(); it++) {
            if (n->isNode) {
                totalLength += calcTmpEdgeLength((*it), srcX, srcY, GL);
            }
            else {
                totalLength += calcTmpEdgeLengthBends((*it)->theEdge(), n, srcX, srcY, GL);
            }
        }
    }
    else {
        for (auto it = adjEntries.begin(); it.valid(); it++) {
            auto it2 = mapEdgeLength.find((*it)->theEdge());
            totalLength += it2->second;
        }
    }
    return totalLength;
}
#endif