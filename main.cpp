#include <ogdf/basic/GridLayout.h>

#include <ogdf/planarlayout/PlanarStraightLayout.h>
#include <ogdf/planarity/EmbedderMinDepth.h>
#include <ogdf/planarity/SimpleEmbedder.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <string>


#include "jsonIO.hpp"
#include "dispOpenGL.hpp"
#include "EdgeMap.hpp"
#include "NodeMap.hpp"
#include "graphFunctions.hpp"
#include "noDispRun.hpp"

using ogdf::Graph;
using ogdf::GridLayout;
using std::cout, std::endl;

int main() {
    
    Graph G;
    GridLayout GL{G};

    int gridWidth, gridHeight, maxBends;

    // ----- LECTURE D'UN FICHIER JSON DANS UN Graph -----
    string file = "F:/The World/Cours/M1S2/Graphe/GitHub/ProjetGrapheM1-BinaryHeap/ProjetGrapheM1/exemples/exemple11.json";
    std::cout << "File: " << file << std::endl;
    readFromJson(file, G, GL, gridWidth, gridHeight, maxBends);
    writeToJson("output.json", G, GL, gridWidth, gridHeight, maxBends);
    
    int maxX = 0, maxY = 0, minX = 100000, minY = 100000;

    bool planarize = false;
    if (planarize) {
        std::cout << "Planarizing..." << std::endl;
        PlanarStraightLayout PL;
        PL.separation(-19);
        PL.callGrid(G, GL);
        node n = G.firstNode();
        while (n != nullptr) {
            if (GL.x(n) > maxX) maxX = GL.x(n);
            if (GL.x(n) < minX) minX = GL.x(n);
            if (GL.y(n) > maxY) maxY = GL.x(n);
            if (GL.y(n) < minY) minY = GL.x(n);
            n = n->succ();
        }
    }

    std::cout << "Embedding..." << std::endl;
    embedderCarte(G, GL);
    std::cout << "Embeded: " << G.representsCombEmbedding() << std::endl;

    //GraphIO::write(GL, "output-ERDiagram2.svg", GraphIO::drawSVG);
    std::cout << "Connexe: " << isConnected(G) << std::endl;
    std::cout << "Planaire: " << isPlanar(G) << std::endl;

    // Affichage des maps
    
    /*std::map<edge, double>::iterator it;
    for (it = mapEdgeLength.begin(); it != mapEdgeLength.end(); it++) {
        std::cout << "MapEdgeLength: " << it->second << std::endl;
    }

    std::map<double, std::set<edge>>::iterator it2;
    for (it2 = mapLengthEdgeSet.begin(); it2 != mapLengthEdgeSet.end(); it2++) {
        std::cout << "mapLengthEdgeSet: " << it2->first << std::endl;
    }*/

    // Ajout des node dans le vector
    node n = G.firstNode();
    while (n != nullptr) {
        NodeBend tmpNodeBend(n, GL);
        vectorNodeBends.push_back(tmpNodeBend);
        n = n->succ();
    }

    // Ajout des bend dans le vector
    edge e = G.firstEdge();
    while (e != nullptr) {
        IPolyline& bends = GL.bends(e);
        int k = 0;
        for (ListIterator<IPoint> i = bends.begin(); i.valid(); i++,k++) {
            NodeBend tmpNodeBend((*i),e,k);
            vectorNodeBends.push_back(tmpNodeBend);
        }
        e = e->succ();
    }
    // On melange le vecteur de nodebend pour affecter de l'aléatoire sur certains algo
    auto rd = std::random_device{};
    auto rng = std::default_random_engine{ rd() };
    std::shuffle(std::begin(vectorNodeBends), std::end(vectorNodeBends), rng);

    bool useOpenGL = true;

    // OpenGL
    srand(static_cast<unsigned int>(time(NULL)));
    if (useOpenGL) {
        dispOpenGL(G, GL, gridWidth, gridHeight, maxX, maxY, maxBends);
    }
    else {
        runAlgo(2, G, GL, gridWidth, gridHeight, maxX, maxY, maxBends);
    }
    return 0;
}