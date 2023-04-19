#ifndef FMMM_HPP
#define FMMM_HPP

#include <vector>

#include "graphe.hpp"

class GalaxyMultilevel {
public:
    using NearSunList = std::vector<std::pair<int,int>>;

    struct LevelNodeInfo {
        float mass;
        float radius;
        int parent;
        NearSunList nearSuns;
    };

    struct LevelEdgeInfo { float length; };

    explicit GalaxyMultilevel(Graphe* pGraph) {
        m_pFinerMultiLevel = nullptr;
        m_pCoarserMultiLevel = nullptr;
        m_pGraph = pGraph;
        m_pNodeInfo = new std::vector<LevelNodeInfo>();
        m_pEdgeInfo = new std::vector<LevelEdgeInfo>();
        for (int i=0;i<m_pGraph->_noeuds.size();i++) {
            (*m_pNodeInfo)[i].mass = 1.0;
        }
        levelNumber = 0;
    }

    GalaxyMultilevel(GalaxyMultilevel* prev) {
        m_pCoarserMultiLevel = nullptr;
        m_pFinerMultiLevel = prev;
        m_pFinerMultiLevel->m_pCoarserMultiLevel = this;
        m_pGraph = nullptr;
        m_pNodeInfo = nullptr;
        levelNumber = prev->levelNumber + 1;
    }

    ~GalaxyMultilevel() { }

    GalaxyMultilevel* m_pFinerMultiLevel;
    GalaxyMultilevel* m_pCoarserMultiLevel;
    Graphe* m_pGraph;
    std::vector<LevelNodeInfo>* m_pNodeInfo;
    std::vector<LevelEdgeInfo>* m_pEdgeInfo;
    int levelNumber;
};


class GalaxyMultilevelBuilder {
public:
    struct LevelNodeState {
        int lastVisitor;
        double sysMass;
        int label;
        float edgeLengthFromSun;
    };

    struct NodeOrderInfo {
        int theNode;
    };

    GalaxyMultilevel* build(GalaxyMultilevel* pMultiLevel);

private:
    void computeSystemMass();
    void sortNodesBySystemMass();
    void createResult(GalaxyMultilevel* pMultiLevelResult);
    void labelSystem(int u, int v, int d, float df);
    void labelSystem();
    Graphe* m_pGraph = nullptr;
    Graphe* m_pGraphResult = nullptr;
    std::vector<int> m_sunNodeList;
    std::vector<int> m_interSystemEdges;
    std::vector<GalaxyMultilevel::LevelNodeInfo>* m_pNodeInfo = nullptr;
    std::vector<GalaxyMultilevel::LevelEdgeInfo>* m_pEdgeInfo = nullptr;
    std::vector<GalaxyMultilevel::LevelNodeInfo>* m_pNodeInfoResult = nullptr;
    std::vector<GalaxyMultilevel::LevelEdgeInfo>* m_pEdgeInfoResult = nullptr;
    std::vector<LevelNodeState> m_nodeState;
    NodeOrderInfo* m_nodeMassOrder = nullptr;
    std::vector<int>* m_pRandomSet = nullptr;
    int m_dist = 0;
};
  
  
class NodeMassComparer {
public:
    explicit NodeMassComparer(const std::vector< GalaxyMultilevelBuilder::LevelNodeState>& nodeState) : m_nodeState(nodeState) { }

    // used for std::sort
    inline bool operator()(const GalaxyMultilevelBuilder::NodeOrderInfo& a, const GalaxyMultilevelBuilder::NodeOrderInfo& b) const {
        return m_nodeState[a.theNode].sysMass < m_nodeState[b.theNode].sysMass;
    }

private:
    const std::vector< GalaxyMultilevelBuilder::LevelNodeState >& m_nodeState;
};

struct FMEGlobalOptions {
    float preProcTimeStep;              
    float preProcEdgeForceFactor;       
    uint32_t preProcMaxNumIterations;   

    float timeStep;                     
    float edgeForceFactor;              
    float repForceFactor;               
    float normEdgeLength;               
    float normNodeSize;                 
    uint32_t maxNumIterations;          
    uint32_t minNumIterations;          

    bool doPrepProcessing;              
    bool doPostProcessing;

    double stopCritForce;               
    double stopCritAvgForce;                
    double stopCritConstSq;             

    uint32_t multipolePrecision;
};

class FastMultipoleEmbedder {
public:
    FastMultipoleEmbedder();
    void call(Graphe& G);
    void call(
        const Graphe& G,
        std::vector<float>& nodeXPosition,
        std::vector<float>& nodeYPosition,
        const std::vector<float>& edgeLength,
        const std::vector<float>& nodeSize);

    void call(Graphe& G, const std::vector<float>& edgeLength, const std::vector<float>& nodeSize);
    void setNumIterations(uint32_t numIterations) { m_numIterations = numIterations; }
    void setMultipolePrec(uint32_t precision) { m_precisionParameter = precision; }
    void setRandomize(bool b) { m_randomize = b; }
    void setDefaultEdgeLength(float edgeLength) { m_defaultEdgeLength = edgeLength; }
    void setDefaultNodeSize(float nodeSize) { m_defaultNodeSize = nodeSize; }


private:
    void initOptions();
    void runMultipole();
    void runSingle();
    void run(uint32_t numIterations);
    void allocate();
    void deallocate();
    uint32_t m_numIterations;
    Graphe* m_pGraph;
    uint32_t m_precisionParameter;
    bool m_randomize;
    float m_defaultEdgeLength;
    float m_defaultNodeSize;
    FMEGlobalOptions* m_pOptions;
};

class FMMM {
public:
    Graphe* G = nullptr;

    FMMM(){}
    FMMM(Graphe* _g){ G = _g; }

    void runAlgo();
    void createMultiLevelGraphs();
    void initCurrentLevel();
    void layoutCurrentLevel();
    void nextLevel();
    void assignPositionsFromPrevLevel();
    void writeCurrentToGraphAttributes();
    void deleteMultiLevelGraphs();

    int m_iMaxNumThreads;
    int m_iNumLevels;
    int m_multiLevelNumNodesBound;
  
    GalaxyMultilevel* m_pCurrentLevel = nullptr;
    GalaxyMultilevel* m_pFinestLevel = nullptr;
    GalaxyMultilevel* m_pCoarsestLevel = nullptr;
  
    Graphe* m_pCurrentGraph = nullptr;
    std::vector<float>* m_pCurrentNodeXPos = nullptr;
    std::vector<float>* m_pCurrentNodeYPos = nullptr;
    std::vector<float>* m_pCurrentEdgeLength = nullptr;
    std::vector<float>* m_pCurrentNodeSize = nullptr;
    std::vector<float> m_adjustedNodeSize;
    int m_iCurrentLevelNr;
  
    std::vector<float>* m_pLastNodeXPos = nullptr;
    std::vector<float>* m_pLastNodeYPos = nullptr;

};

#endif