#ifndef PIVOT_HPP
#define PIVOT_HPP

#include <vector>

class Graphe;
class PivotMDS {
public:
    Graphe* G = nullptr;

    PivotMDS(){}
    PivotMDS(Graphe* _g){ G = _g; }

    int m_numberOfPivots;
    int m_edgeCosts;
    int m_iterations = 400;
    double m_epsilon = 1 - 1e-10;
    double m_factor = -0.5;

    unsigned int m_seed = 0;

    std::vector<std::vector<double>> pivDistMatrix;

    bool m_useGrille = true;
    bool areVectorsSetup = false;

    int totalIterationDone = 0;

    void runAlgo();

    void getPivotDistanceMatrix();

    void bfs_SPSS(int nodeId, std::vector<double>& shortestPathSingleSource, int edgeCosts);

    void centerPivotmatrix();

    void selfProduct(const std::vector<std::vector<double> >& d, std::vector<std::vector<double> >& result);

    double prod(const std::vector<double>& x, const std::vector<double>& y);

    double normalize(std::vector<double>& x);

    void randomize(std::vector<std::vector<double> >& matrix);

    void eigenValueDecomposition(std::vector<std::vector<double> >& K, std::vector<std::vector<double> >& eVecs, std::vector<double>& eValues);

    void singularValueDecomposition(std::vector<std::vector<double> >& eVecs, std::vector<double>& eVals);

};

#endif