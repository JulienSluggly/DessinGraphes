#ifndef JSONIO
#define JSONIO

#include <string>
#include <ogdf/basic/GridLayout.h>
#include <ogdf/fileformats/GraphIO.h>
#include <nlohmann/json.hpp>

#include "calcEdgeLength.hpp"
#include "EdgeMap.hpp"
#include "NodeMap.hpp"

using std::string;
using nlohmann::json;
using namespace ogdf;

// ----- CREATION D'UN Graph A PARTIR D'UN FICHIER JSON -----
void readFromJson(string input, Graph& G, GridLayout& GL, int& gridWidth, int& gridHeight, int& maxBends) {
    std::ifstream i(input);
    json j;
    i >> j;
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
    }
    int edgeNumber = static_cast<int>(j["edges"].size());
    edge* edgeTab = new edge[edgeNumber];
    for (int i = 0; i < edgeNumber; i++) {
        edgeTab[i] = G.newEdge(nodeTab[j["edges"][i]["source"]], nodeTab[j["edges"][i]["target"]]);

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
        j2["nodes"][m]["id"] = n->index();
        j2["nodes"][m]["x"] = (int)GL.x(n);
        j2["nodes"][m]["y"] = (int)GL.y(n);
        n = n->succ();
        m++;
    }

    edge e = G.firstEdge();
    m = 0;
    while (e != nullptr) {
        j2["edges"][m]["source"] = e->source()->index();
        j2["edges"][m]["target"] = e->target()->index();
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