#ifndef JSONIO
#define JSONIO

#include <string>
#include <ogdf/basic/GridLayout.h>
#include <ogdf/fileformats/GraphIO.h>
#include <nlohmann/json.hpp>
#include <ogdf/basic/GraphCopy.h>

#include "calcEdgeLength.hpp"
#include "EdgeMap.hpp"
#include "NodeMap.hpp"

using std::string;
using nlohmann::json;
using namespace ogdf;

int lastEdgeIndex;

// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
void readFromJson(string input, Graph& G, GridLayout& GL, int& gridWidth, int& gridHeight, int& maxBends) {
    std::ifstream inp(input);
    json j;
    inp >> j;
    gridWidth = j["width"];
    gridHeight = j["height"];
    maxBends = j["bends"];
    std::cout << "gridWidth: " << gridWidth << " gridHeight: " << gridHeight << " Max Bends: " << maxBends << std::endl;

    // BOOLEEN A CHANGER POUR AJOUTER LE MAXIMUM DE BENDS A CHAQUE EDGE
    bool setMaxBends = false;
    if (j["nodes"] == nullptr) {
        exit(1);
    }

    // REMPLIR LES MAP ET VECTOR UNIQUEMENT SI GRAPHE PLANAIRE,
    // VERIF PAR EXEMPLE SI (j["nodes"][0]["x"]!=j["nodes"][1]["x"])&&(j["nodes"][0]["y"]!=j["nodes"][1]["y"])

    int nodeNumber = static_cast<int>(j["nodes"].size());
    node* nodeTab = new node[nodeNumber];
    for (int i = 0; i < nodeNumber; i++) {
        nodeTab[i] = G.newNode();
        GL.x(nodeTab[i]) = j["nodes"][i]["x"];
        GL.y(nodeTab[i]) = j["nodes"][i]["y"];
        int id = j["nodes"][i]["id"];
        mapIndexId.insert(std::make_pair(nodeTab[i]->index(), id));
        mapIdNode.insert(std::make_pair(id, nodeTab[i]));
    }
    int edgeNumber = static_cast<int>(j["edges"].size());
    lastEdgeIndex = edgeNumber;
    edge* edgeTab = new edge[edgeNumber];
    int id1, id2;
    for (int i = 0; i < edgeNumber; i++) {
        id1 = j["edges"][i]["source"];
        id2 = j["edges"][i]["target"];
        auto itnode1 = mapIdNode.find(id1);
        auto it2node2 = mapIdNode.find(id2);
        node node1 = (*itnode1).second;
        node node2 = (*it2node2).second;
        edgeTab[i] = G.newEdge(node1, node2);

        if (j["edges"][i]["bends"] != nullptr) {
            IPolyline& p = GL.bends(edgeTab[i]);
            int bendsNumber = static_cast<int>(j["edges"][i]["bends"].size());
            int k = 0;
            for (; k < bendsNumber; k++) {
                int bendX = j["edges"][i]["bends"][k]["x"];
                int bendY = j["edges"][i]["bends"][k]["y"];
                p.pushBack(IPoint(bendX, bendY));
                // On ajoute les bends dans la nodemap:
            }
            // ON AJOUTE LES BENDS SUPPLEMENTAIRE SUR LE NODE TARGET A LA FIN DES BENDS ORIGINAUX
            if (setMaxBends) {
                int bendX = GL.x(nodeTab[j["edges"][i]["target"]]);
                int bendY = GL.y(nodeTab[j["edges"][i]["target"]]);
                for (; k < maxBends; k++) {
                    p.pushBack(IPoint(bendX, bendY));
                }
            }
        }
        else if (setMaxBends) {
            IPolyline& p = GL.bends(edgeTab[i]);
            int bendX = GL.x(nodeTab[j["edges"][i]["target"]]);
            int bendY = GL.y(nodeTab[j["edges"][i]["target"]]);
            for (int k=0; k < maxBends; k++) {
                p.pushBack(IPoint(bendX, bendY));
            }
        }
        //recuperer longueur edge
        double length = calcEdgeLength(edgeTab[i], GL);
        mapEdgeLength.insert(std::pair<edge, double>(edgeTab[i], length));
        std::map<double, std::set<edge>>::iterator it2 = mapLengthEdgeSet.begin();
        it2 = mapLengthEdgeSet.find(length);
        // La valeur est déja présente, on ajoute dans le set
        if (it2 != mapLengthEdgeSet.end()) {
            it2->second.insert(edgeTab[i]);
        }
        // La valeur n'est pas présente, on créer un nouveau set.
        else {
            std::set<edge> tmpSet;
            tmpSet.insert(edgeTab[i]);
            mapLengthEdgeSet.insert(std::pair<double, std::set<edge>>(length, tmpSet));
        }

    }
    delete[] nodeTab;
    delete[] edgeTab;
}

// ----- ECRITURE D'UN Graph DANS UN FICHIER JSON -----
void writeToJson(string output, const Graph& G, const GridLayout& GL, int gridWidth, int gridHeight, int maxBends) {
    json j2;
    j2["width"] = gridWidth;
    j2["height"] = gridHeight;
    j2["bends"] = maxBends;

    node n = G.firstNode();
    int m = 0;
    while (n != nullptr) {
        auto it = mapIndexId.find(n->index());
        j2["nodes"][m]["id"] = (*it).second;
        j2["nodes"][m]["x"] = (int)GL.x(n);
        j2["nodes"][m]["y"] = (int)GL.y(n);
        n = n->succ();
        m++;
    }

    edge e = G.firstEdge();
    m = 0;
    while (e != nullptr) {
        auto it1 = mapIndexId.find(e->source()->index());
        j2["edges"][m]["source"] = (*it1).second;
        auto it2 = mapIndexId.find(e->target()->index());
        j2["edges"][m]["target"] = (*it2).second;
        IPolyline bends = GL.bends(e);
        if (bends.size() > 0) {
            int l = 0;
            for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
                j2["edges"][m]["bends"][l]["x"] = (int)(*i).m_x;
                j2["edges"][m]["bends"][l]["y"] = (int)(*i).m_y;
                l++;
            }
        }
        e = e->succ();
        m++;
    }

    std::ofstream o(output);
    o << std::setw(4) << j2 << std::endl;
}

// ----- ECRITURE D'UN Graph DANS UN FICHIER JSON -----
void writeToJsonDouble(string output, const Graph& G, const GraphAttributes& GA, int gridWidth, int gridHeight, int maxBends) {
    json j2;
    j2["width"] = gridWidth;
    j2["height"] = gridHeight;
    j2["bends"] = maxBends;

    node n = G.firstNode();
    int m = 0;
    while (n != nullptr) {
        auto it = mapIndexId.find(n->index());
        j2["nodes"][m]["id"] = (*it).second;
        j2["nodes"][m]["x"] = (double)GA.x(n);
        j2["nodes"][m]["y"] = (double)GA.y(n);
        n = n->succ();
        m++;
    }

    edge e = G.firstEdge();
    m = 0;
    while (e != nullptr) {
        auto it1 = mapIndexId.find(e->source()->index());
        j2["edges"][m]["source"] = (*it1).second;
        auto it2 = mapIndexId.find(e->target()->index());
        j2["edges"][m]["target"] = (*it2).second;
        DPolyline bends = GA.bends(e);
        if (bends.size() > 0) {
            int l = 0;
            for (ListIterator<DPoint> i = bends.begin(); i.valid(); i++) {
                j2["edges"][m]["bends"][l]["x"] = (double)(*i).m_x;
                j2["edges"][m]["bends"][l]["y"] = (double)(*i).m_y;
                l++;
            }
        }
        e = e->succ();
        m++;
    }

    std::ofstream o(output);
    o << std::setw(4) << j2 << std::endl;
}

// Sauvegarde dans plusieurs fichiers json les différentes composantes connexes
void saveAllConnectedComponents(string output, const Graph& G, const GridLayout& GL, int gridWidth, int gridHeight, int maxBends) {
    ogdf::Graph::CCsInfo CCS(G);
    json j2;
    for (int i = 0; i < CCS.numberOfCCs(); i++) {
        j2.clear();
        j2["width"] = gridWidth;
        j2["height"] = gridHeight;
        j2["bends"] = maxBends;

        std::cout << i << ": " << CCS.numberOfEdges(i) << " nnode: " << CCS.numberOfNodes(i) << std::endl;
        
        int nstartindex = CCS.startNode(i);
        int nendindex = CCS.stopNode(i);
        node n = CCS.v(nstartindex);
        std::cout << nstartindex << " " << nendindex << std::endl;
        int m = 0;
        while ((n != nullptr)&&(n->index() != nendindex)) {
            auto it = mapIndexId.find(n->index());
            j2["nodes"][m]["id"] = (*it).second;
            j2["nodes"][m]["x"] = (int)GL.x(n);
            j2["nodes"][m]["y"] = (int)GL.y(n);
            n = n->succ();
            m++;
        }

        if (CCS.numberOfEdges(i) > 0) {
            edge e = G.firstEdge();
            int estartindex = CCS.startEdge(i);
            int eendindex = CCS.stopEdge(i);
            m = 0;
            for (; e->index() != estartindex; e = e->succ());
            while ((e != nullptr) && (e->index() != eendindex) && (e->index() < lastEdgeIndex)) {
                auto it1 = mapIndexId.find(e->source()->index());
                j2["edges"][m]["source"] = (*it1).second;
                auto it2 = mapIndexId.find(e->target()->index());
                j2["edges"][m]["target"] = (*it2).second;
                IPolyline bends = GL.bends(e);
                if (bends.size() > 0) {
                    int l = 0;
                    for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
                        j2["edges"][m]["bends"][l]["x"] = (int)(*i).m_x;
                        j2["edges"][m]["bends"][l]["y"] = (int)(*i).m_y;
                        l++;
                    }
                }
                e = e->succ();
                m++;
            }
        }
        string new_output = "tmp/" + output + "_CC" + to_string(i) + ".json";
        std::ofstream o(new_output);
        o << std::setw(4) << j2 << std::endl;
    }
}

// Sauvegarde dans plusieurs fichiers json les différentes composantes connexes
void saveAllConnectedComponentsv2(string output, const Graph& G, const GridLayout& GL, int gridWidth, int gridHeight, int maxBends) {
    ogdf::Graph::CCsInfo CCS(G);
    json j2;
    GraphCopy* GC = new GraphCopy(G);
    EdgeArray<edge> listEdge;
    for (int i = 0; i < CCS.numberOfCCs(); i++) {
        GC->clear();
        GC->initByCC(CCS, i, listEdge);
        j2.clear();
        j2["width"] = gridWidth;
        j2["height"] = gridHeight;
        j2["bends"] = maxBends;

        std::cout << i << ": " << CCS.numberOfEdges(i) << " nnode: " << CCS.numberOfNodes(i) << std::endl;
        node n = GC->firstNode();
        int m = 0;
        while (n != nullptr) {
            node orig = GC->original(n);
            auto it = mapIndexId.find(orig->index());
            j2["nodes"][m]["id"] = (*it).second;
            j2["nodes"][m]["x"] = (int)GL.x(orig);
            j2["nodes"][m]["y"] = (int)GL.y(orig);
            n = n->succ();
            m++;
        }

        edge e = GC->firstEdge();
        m = 0;
        while (e != nullptr) {
            edge origEdge = GC->original(e);
            auto it1 = mapIndexId.find(origEdge->source()->index());
            j2["edges"][m]["source"] = (*it1).second;
            auto it2 = mapIndexId.find(origEdge->target()->index());
            j2["edges"][m]["target"] = (*it2).second;
            IPolyline bends = GL.bends(origEdge);
            if (bends.size() > 0) {
                int l = 0;
                for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
                    j2["edges"][m]["bends"][l]["x"] = (int)(*i).m_x;
                    j2["edges"][m]["bends"][l]["y"] = (int)(*i).m_y;
                    l++;
                }
            }
            e = e->succ();
            m++;
        }
        string new_output = "tmp/" + output + "_CC" + to_string(i) + ".json";
        std::ofstream o(new_output);
        o << std::setw(4) << j2 << std::endl;
    }
}

// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
void combinesFromJson(string input, int number, Graph& G, GridLayout& GL, int& gridWidth, int& gridHeight, int& maxBends) {
    json j;
    int maxX = 0;
    int oldMaxX = 0;
    for (int numero = 0; numero < number; numero++) {
        j.clear();
        string new_nom = input + to_string(numero) + ".json";
        std::ifstream inp(new_nom);
        inp >> j;
        gridWidth = j["width"];
        gridHeight = j["height"];
        maxBends = j["bends"];
        std::cout << "Numero: " << numero << " gridWidth: " << gridWidth << " gridHeight: " << gridHeight << " Max Bends: " << maxBends << std::endl;

        // BOOLEEN A CHANGER POUR AJOUTER LE MAXIMUM DE BENDS A CHAQUE EDGE
        bool setMaxBends = false;
        if (j["nodes"] == nullptr) {
            exit(1);
        }
        int currentMaxX = 0;
        int currentMinX = 9999999;
        int nodeNumber = static_cast<int>(j["nodes"].size());
        node* nodeTab = new node[nodeNumber];
        for (int i = 0; i < nodeNumber; i++) {
            nodeTab[i] = G.newNode();
            GL.x(nodeTab[i]) = j["nodes"][i]["x"];
            GL.y(nodeTab[i]) = j["nodes"][i]["y"];
            int id = j["nodes"][i]["id"];
            mapIndexId.insert(std::make_pair(nodeTab[i]->index(), id));
            mapIdNode.insert(std::make_pair(id, nodeTab[i]));
            if (j["nodes"][i]["x"] > maxX) {
                maxX = j["nodes"][i]["x"];
            }
            if (numero > 0) {
                if (j["nodes"][i]["x"] > currentMaxX) {
                    currentMaxX = j["nodes"][i]["x"];
                }
                if (j["nodes"][i]["x"] < currentMinX) {
                    currentMinX = j["nodes"][i]["x"];
                }
            }
        }
        if (numero > 0) {
            int diffX = oldMaxX + 1 - currentMinX;
            for (int i = 0; i < nodeNumber; i++) {
                GL.x(nodeTab[i]) = GL.x(nodeTab[i]) + diffX;
                if (GL.x(nodeTab[i]) > maxX) {
                    maxX = GL.x(nodeTab[i]);
                }
            }
        }
        oldMaxX = maxX;
        if (j["edges"] != nullptr) {
            int edgeNumber = static_cast<int>(j["edges"].size());
            lastEdgeIndex = edgeNumber;
            edge* edgeTab = new edge[edgeNumber];
            int id1, id2;
            for (int i = 0; i < edgeNumber; i++) {
                id1 = j["edges"][i]["source"];
                id2 = j["edges"][i]["target"];
                auto itnode1 = mapIdNode.find(id1);
                auto it2node2 = mapIdNode.find(id2);
                node node1 = (*itnode1).second;
                node node2 = (*it2node2).second;
                edgeTab[i] = G.newEdge(node1, node2);
                //recuperer longueur edge
                double length = calcEdgeLength(edgeTab[i], GL);
                mapEdgeLength.insert(std::pair<edge, double>(edgeTab[i], length));
                std::map<double, std::set<edge>>::iterator it2 = mapLengthEdgeSet.begin();
                it2 = mapLengthEdgeSet.find(length);
                // La valeur est déja présente, on ajoute dans le set
                if (it2 != mapLengthEdgeSet.end()) {
                    it2->second.insert(edgeTab[i]);
                }
                // La valeur n'est pas présente, on créer un nouveau set.
                else {
                    std::set<edge> tmpSet;
                    tmpSet.insert(edgeTab[i]);
                    mapLengthEdgeSet.insert(std::pair<double, std::set<edge>>(length, tmpSet));
                }

            }
            delete[] nodeTab;
            delete[] edgeTab;
        }
    }
}

// ----- ECRITURE D'UN Graph DANS UN FICHIER JSON -----
void saveInOrder(string output, const Graph& G, const GridLayout& GL, int gridWidth, int gridHeight, int maxBends) {
    json j2;
    j2["width"] = gridWidth;
    j2["height"] = gridHeight;
    j2["bends"] = maxBends;

    int m = 0;
    for (int i = 0; i < G.numberOfNodes(); i++) {
        auto it = mapIdNode.find(i);
        node n = (*it).second;
        j2["nodes"][m]["id"] = i;
        j2["nodes"][m]["x"] = (int)GL.x(n);
        j2["nodes"][m]["y"] = (int)GL.y(n);
        n = n->succ();
        m++;
    }

    edge e = G.firstEdge();
    m = 0;
    while (e != nullptr) {
        auto it1 = mapIndexId.find(e->source()->index());
        j2["edges"][m]["source"] = (*it1).second;
        auto it2 = mapIndexId.find(e->target()->index());
        j2["edges"][m]["target"] = (*it2).second;
        IPolyline bends = GL.bends(e);
        if (bends.size() > 0) {
            int l = 0;
            for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++) {
                j2["edges"][m]["bends"][l]["x"] = (int)(*i).m_x;
                j2["edges"][m]["bends"][l]["y"] = (int)(*i).m_y;
                l++;
            }
        }
        e = e->succ();
        m++;
    }

    std::ofstream o(output);
    o << std::setw(4) << j2 << std::endl;
}

#endif