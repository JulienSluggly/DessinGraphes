#include "fmmm.hpp"
#include <algorithm>

void FMMM::createMultiLevelGraphs() {
    m_pCurrentLevel = new GalaxyMultilevel(G);
	m_pFinestLevel = m_pCurrentLevel;
	m_iNumLevels = 1;
	m_iCurrentLevelNr = 0;

	GalaxyMultilevelBuilder builder;
	while (m_pCurrentLevel->m_pGraph->_noeuds.size() > m_multiLevelNumNodesBound) {
		GalaxyMultilevel* newLevel = builder.build(m_pCurrentLevel);
		m_pCurrentLevel = newLevel;
		m_iNumLevels++;
		m_iCurrentLevelNr++;
	}
	m_pCoarsestLevel = m_pCurrentLevel;
	m_pCurrentGraph = m_pCurrentLevel->m_pGraph;
}

void FMMM::initCurrentLevel() {
    m_pCurrentGraph = m_pCurrentLevel->m_pGraph;
    m_pCurrentNodeXPos->clear();
    m_pCurrentNodeYPos->clear();
    m_pCurrentNodeSize->clear();
    for (int i=0;i<m_pCurrentGraph->_noeuds.size();i++) {
        m_pCurrentNodeXPos->push_back(0.0f);
        m_pCurrentNodeYPos->push_back(0.0f);
        m_pCurrentNodeSize->push_back(1.0f);
    }
    m_pCurrentEdgeLength->clear();
    for (int i=0;i<m_pCurrentGraph->_aretes.size();i++) {
        m_pCurrentEdgeLength->push_back(1.0f);
    }
	const Graphe& G = *(m_pCurrentLevel->m_pGraph);

    for (int v=0;v<G._noeuds.size();v++) {
        GalaxyMultilevel::LevelNodeInfo& nodeInfo = (*(m_pCurrentLevel->m_pNodeInfo))[v];
		(*m_pCurrentNodeSize)[v] = ((float)nodeInfo.radius);
    }

    for (int e=0;e<G._aretes.size();e++) {
        GalaxyMultilevel::LevelEdgeInfo& edgeInfo = (*(m_pCurrentLevel->m_pEdgeInfo))[e];
		(*m_pCurrentEdgeLength)[e] = edgeInfo.length*0.25f;
    }
}

void FMMM::layoutCurrentLevel() {
    FastMultipoleEmbedder fme;
	fme.setNumIterations(200*(m_iCurrentLevelNr+1)*(m_iCurrentLevelNr+1));
	fme.call((*m_pCurrentGraph), (*m_pCurrentNodeXPos), (*m_pCurrentNodeYPos), (*m_pCurrentEdgeLength), (*m_pCurrentNodeSize));
}

void FMMM::nextLevel() {
    m_pCurrentLevel = m_pCurrentLevel->m_pFinerMultiLevel;
	std::swap(m_pLastNodeXPos, m_pCurrentNodeXPos);
	std::swap(m_pLastNodeYPos, m_pCurrentNodeYPos);
	m_iCurrentLevelNr--;
}

void FMMM::assignPositionsFromPrevLevel() {
    float scaleFactor = 1.4f;// 1.4f;//1.4f; //1.4f
	// init m_pCurrent Pos from m_pLast Pos
	const Graphe& G = *(m_pCurrentLevel->m_pGraph);

	for(int v=0;v<G._noeuds.size();v++) {
		GalaxyMultilevel::LevelNodeInfo& nodeInfo = (*(m_pCurrentLevel->m_pNodeInfo))[v];
		float x = (float)((*m_pLastNodeXPos)[nodeInfo.parent] + (float)generateDoubleRand(2.0)-1.0);
		float y = (float)((*m_pLastNodeYPos)[nodeInfo.parent] + (float)generateDoubleRand(2.0)-1.0);
		(*m_pCurrentNodeXPos)[v] = x*scaleFactor;
		(*m_pCurrentNodeYPos)[v] = y*scaleFactor;
	}
}

void FMMM::writeCurrentToGraphAttributes() {
    for (int i=0;i<m_pCurrentGraph->_noeuds.size();i++) {
        G->_noeuds[i]._xreel = (*m_pCurrentNodeXPos)[i];
        G->_noeuds[i]._yreel = (*m_pCurrentNodeYPos)[i];
    }
}

void FMMM::deleteMultiLevelGraphs() {
    GalaxyMultilevel* level = m_pCoarsestLevel;
	GalaxyMultilevel* toDelete;
	while (level) {
		toDelete = level;
		level = level->m_pFinerMultiLevel;
		delete toDelete->m_pNodeInfo;
		delete toDelete->m_pEdgeInfo;
		if (toDelete != m_pFinestLevel)
			delete toDelete->m_pGraph;
		delete toDelete;
	}
}

void FMMM::runAlgo() {
	// too lazy for new, delete
	std::vector<float> nodeXPos1;
	std::vector<float> nodeYPos1;
	std::vector<float> nodeXPos2;
	std::vector<float> nodeYPos2;
	std::vector<float> edgeLength1;
	std::vector<float> nodeSize1;

	m_pCurrentNodeXPos	= &nodeXPos1;
	m_pCurrentNodeYPos	= &nodeYPos1;
	m_pLastNodeXPos		= &nodeXPos2;
	m_pLastNodeYPos		= &nodeYPos2;
	m_pCurrentEdgeLength= &edgeLength1;
	m_pCurrentNodeSize	= &nodeSize1;

	// create all multilevels
	createMultiLevelGraphs();
	// init the coarsest level
	initCurrentLevel();

	// layout the current level
	layoutCurrentLevel();

	//proceed with remaining levels
	while (m_iCurrentLevelNr > 0) {
		// move to finer level
		nextLevel();
		// init the arrays for current level
		initCurrentLevel();
		// assign positions from last to current
		assignPositionsFromPrevLevel();
		// layout the current level
		layoutCurrentLevel();
	}
	// the finest level is processed
	// assumes m_pCurrentGraph == GA.constGraph

	writeCurrentToGraphAttributes();

	// clean up multilevels
	deleteMultiLevelGraphs();

	//for(edge e : GA.constGraph().edges) {
	//	GA.bends(e).clear();
	//}
}

GalaxyMultilevel* GalaxyMultilevelBuilder::build(GalaxyMultilevel* pMultiLevel) {
	m_dist = 2;
	m_pGraph = pMultiLevel->m_pGraph;
	m_pNodeInfo = pMultiLevel->m_pNodeInfo;
	m_pEdgeInfo = pMultiLevel->m_pEdgeInfo;

	this->computeSystemMass();
	this->sortNodesBySystemMass();
	this->labelSystem();
	GalaxyMultilevel* pMultiLevelResult = new GalaxyMultilevel(pMultiLevel);
	this->createResult(pMultiLevelResult);

	return pMultiLevelResult;
}

void GalaxyMultilevelBuilder::computeSystemMass() {
	for (int i=0;i<m_pGraph->_noeuds.size();i++) {
		m_nodeState[i].sysMass = (*m_pNodeInfo)[i].mass;
		m_nodeState[i].label = 0;
		m_nodeState[i].lastVisitor = i;
	}

    for (int i=0;i<m_pGraph->_noeuds.size();i++) {
        for (const Noeud* noeudVoisin : m_pGraph->_noeuds[i].voisins) {
            int idVoisin = noeudVoisin->_id;
            m_nodeState[i].sysMass += (*m_pNodeInfo)[idVoisin].mass;
        }
        if (m_pGraph->_noeuds[i].voisins.size() == 1) {
            m_nodeState[i].sysMass *= m_pGraph->_noeuds.size();
        }
    }
}


void GalaxyMultilevelBuilder::sortNodesBySystemMass() {
	int i = 0;
	int v = -1;
    std::vector<int> randomVecId;
    for (int i=0;i<m_pGraph->_noeuds.size();i++) {
        randomVecId.push_back(i);
    }
    auto rng = std::default_random_engine {};
	std::shuffle(std::begin(randomVecId), std::end(randomVecId), rng);
	for (i=0;i<randomVecId.size();i++) {
		v = randomVecId[i];
		m_nodeMassOrder[i].theNode = v;
	}

	delete m_pRandomSet;
	std::sort(m_nodeMassOrder, m_nodeMassOrder+(m_pGraph->_noeuds.size()), NodeMassComparer( m_nodeState ));
}


void GalaxyMultilevelBuilder::labelSystem(int u, int v, int d, float df) {
	if (d>0) {
        for (Noeud* noeudVoisin : m_pGraph->_noeuds[v].voisins) {
            int w = noeudVoisin->_id;
            int areteCommune = m_pGraph->_noeuds[v].areteCommune(noeudVoisin);
            if (m_nodeState[w].label < d) {
				float currDistFromSun = (*m_pEdgeInfo)[areteCommune].length + df;
				// check if we relabeling by a new sun
				if (m_nodeState[w].lastVisitor != u) {
					// maybe this node has never been labeled
					m_nodeState[w].lastVisitor = u;
					m_nodeState[w].edgeLengthFromSun = currDistFromSun;
				}
				// finally relabel it
                if (currDistFromSun < m_nodeState[w].edgeLengthFromSun) { m_nodeState[w].edgeLengthFromSun = currDistFromSun; }
				m_nodeState[w].label = d;
				labelSystem(u, w, d-1, currDistFromSun);
			}
        }
	}
}


void GalaxyMultilevelBuilder::labelSystem() {
	m_sunNodeList.clear();
    for (int i=0;i<m_pGraph->_noeuds.size();i++) {
        m_nodeState[i].sysMass = 0;
		m_nodeState[i].label = 0;
		m_nodeState[i].lastVisitor = i;
    }

	for (int i=0; i < m_pGraph->_noeuds.size(); i++) {
		int v = m_nodeMassOrder[i].theNode;
		if (m_nodeState[v].label == 0) {
			m_sunNodeList.push_back(v);
			m_nodeState[v].label = (m_dist+1);
			m_nodeState[v].edgeLengthFromSun = 0.0;//(*m_pNodeInfo)[v].radius;
			labelSystem(v, v, m_dist, m_nodeState[v].edgeLengthFromSun);
		}
	}
}


void GalaxyMultilevelBuilder::createResult(GalaxyMultilevel* pMultiLevelResult) {
	pMultiLevelResult->m_pGraph = new Graphe();
	m_pGraphResult = pMultiLevelResult->m_pGraph;

	std::vector<int> toResultNode(m_pGraphResult->_noeuds.size(),-1);
	// create all sun nodes
    for (const int& v : m_sunNodeList) {
        int vResult = m_pGraphResult->_noeuds.size();
        m_pGraphResult->_noeuds.push_back(Noeud(vResult));
		toResultNode[v] = vResult;
    }

	pMultiLevelResult->m_pNodeInfo = new std::vector<GalaxyMultilevel::LevelNodeInfo>(m_pGraphResult->_noeuds.size());
	m_pNodeInfoResult = pMultiLevelResult->m_pNodeInfo;

	// calculate the real system mass. this may not be the same as calculated before
    for (int i=0;i<m_pGraphResult->_noeuds.size();i++) {
        (*m_pNodeInfoResult)[i].radius = 0.0f;
		(*m_pNodeInfoResult)[i].mass = 0.0f;
    }
    for(int i=0;m_pGraph->_noeuds.size();i++) {
		int uSun = m_nodeState[i].lastVisitor;
		int uSunResult = toResultNode[uSun];
		(*m_pNodeInfo)[i].parent = uSunResult;
		(*m_pNodeInfoResult)[uSunResult].mass +=((*m_pNodeInfo)[i].mass);
        if (m_nodeState[i].edgeLengthFromSun > (*m_pNodeInfoResult)[uSunResult].radius) { (*m_pNodeInfoResult)[uSunResult].radius = m_nodeState[i].edgeLengthFromSun;}
	}

	pMultiLevelResult->m_pEdgeInfo = new std::vector<GalaxyMultilevel::LevelEdgeInfo>(m_pGraphResult->_aretes.size());
	m_pEdgeInfoResult = pMultiLevelResult->m_pEdgeInfo;

    for (int e=0;e<m_pGraph->_aretes.size();e++) {
        int v = m_pGraph->_aretes[e]._noeud1->_id;
        int w = m_pGraph->_aretes[e]._noeud2->_id;
		int vSun = m_nodeState[v].lastVisitor;
		int wSun = m_nodeState[w].lastVisitor;
		if (vSun != wSun) {
			int vSunResult = toResultNode[vSun];
			int wSunResult = toResultNode[wSun];
            int eResult = m_pGraphResult->_aretes.size();
            m_pGraphResult->_aretes.push_back(Aretes(&m_pGraphResult->_noeuds[vSunResult],&m_pGraphResult->_noeuds[wSunResult],eResult));
			(*m_pEdgeInfoResult)[eResult].length = m_nodeState[v].edgeLengthFromSun + (*m_pEdgeInfo)[e].length + m_nodeState[w].edgeLengthFromSun;
		}
    }

	// make fast parallel free
	std::vector<int> lastVisit(m_pGraphResult->_noeuds.size(),-1);
    for (int v=0;v<m_pGraphResult->_noeuds.size();v++) {
        if (m_pGraphResult->_noeuds[v].voisins.size()>1) {
            for (Noeud* noeudVoisin : m_pGraphResult->_noeuds[v].voisins) {
                int w = noeudVoisin->_id;
                int e = m_pGraphResult->_noeuds[v].areteCommune(noeudVoisin);
                if (lastVisit[w] ==v)
					m_pGraphResult->supprimerArete(e);
				else
					lastVisit[w] = v;
            }
		}
    }
}

FastMultipoleEmbedder::FastMultipoleEmbedder() {
	m_precisionParameter = 5;
	m_defaultEdgeLength = 1.0;
	m_defaultNodeSize = 1.0;
	m_numIterations = 100;
	m_randomize = true;
	m_pGraph = nullptr;
	m_pOptions = nullptr;
}

void FastMultipoleEmbedder::initOptions() {
	m_pOptions->preProcTimeStep = 0.5;
	m_pOptions->preProcMaxNumIterations = 20;
	m_pOptions->preProcEdgeForceFactor = 0.5;
	m_pOptions->timeStep = 0.25;
	m_pOptions->edgeForceFactor = 1.0;
	m_pOptions->repForceFactor = 2.0;
	m_pOptions->stopCritConstSq = 2000400;
	m_pOptions->stopCritAvgForce = 0.1f;
	m_pOptions->minNumIterations = 4;
	m_pOptions->multipolePrecision = m_precisionParameter;
}

void readFrom(Graphe* m_pG, const Graphe& G, std::vector<float>& xPos, std::vector<float>& yPos, const std::vector<float>& edgeLength, const std::vector<float>& nodeSize) {
	for (int i=0;i<G._noeuds.size();i++) {
		m_pG->_noeuds.push_back(Noeud(i));
		m_pG->_noeuds[i]._xreel = (float)xPos[i];
		m_pG->_noeuds[i]._yreel = (float)yPos[i];
	}
	for (int i=0;i<G._aretes.size();i++) {
		int id1 = G._aretes[i]._noeud1->_id;
		int id2 = G._aretes[i]._noeud2->_id;
		m_pG->_aretes.push_back(Aretes(&m_pG->_noeuds[id1],&m_pG->_noeuds[id2],i));
		m_pG->_aretes[i].edgeLength = (float)edgeLength[i];
	}
}

void writeTo(Graphe* m_pG, std::vector<float>& xPos, std::vector<float>& yPos){
	for (int i=0;i<m_pG->_noeuds.size();i++) {
		xPos[i] = m_pG->_noeuds[i]._xreel;
		yPos[i] = m_pG->_noeuds[i]._yreel;
	}
}

void FastMultipoleEmbedder::call(const Graphe& G,std::vector<float>& nodeXPosition, std::vector<float>& nodeYPosition,const std::vector<float>& edgeLength, const std::vector<float>& nodeSize) {
	allocate();
	readFrom(m_pGraph, G, nodeXPosition, nodeYPosition, edgeLength, nodeSize);
	run(m_numIterations);
	writeTo(m_pGraph, nodeXPosition, nodeYPosition);
	deallocate();
}

void FastMultipoleEmbedder::call(Graphe &G) {
	std::vector<float> edgeLength(G._aretes.size());
	std::vector<float> nodeSize(G._noeuds.size());

	for(int v=0;v<G._noeuds.size();v++) {
		nodeSize[v] = 1.0f;
	}

	for(int e=0;e<G._aretes.size();e++) {
		edgeLength[e] = 2.0f;
	}
	call(G, edgeLength, nodeSize);
}

void FastMultipoleEmbedder::call(Graphe &GA, const std::vector<float>& edgeLength, const std::vector<float>& nodeSize) {
	allocate();
	//m_pGraph->readFrom(GA, edgeLength, nodeSize);
	run(m_numIterations);
	//m_pGraph->writeTo(GA);
	deallocate();

	//for(edge e : GA.constGraph().edges) {
	//	GA.bends(e).clear();
	//}
}

void FastMultipoleEmbedder::run(uint32_t numIterations) {
	if (m_pGraph->_noeuds.size() == 0) return;
	if (m_pGraph->_noeuds.size() == 1) {
		m_pGraph->_noeuds[0]._xreel = 0.0f;
		m_pGraph->_noeuds[0]._yreel = 0.0f;
		return;
	}

	double avgNodeSize = 1.0;
	if (m_randomize) {
		for (int i = 0; i < m_pGraph->_noeuds.size(); i++) {
			m_pGraph->_noeuds[i]._xreel = (float)(generateDoubleRand((double)m_pGraph->_noeuds.size()*avgNodeSize*3)-(double)m_pGraph->_noeuds.size());
			m_pGraph->_noeuds[i]._yreel = (float)(generateDoubleRand((double)m_pGraph->_noeuds.size()*avgNodeSize*3)-(double)m_pGraph->_noeuds.size());
		}
	}

	m_pOptions->maxNumIterations = numIterations;
	m_pOptions->stopCritForce = (((float)m_pGraph->_noeuds.size())*((float)m_pGraph->_noeuds.size())*avgNodeSize) / m_pOptions->stopCritConstSq;
	if (m_pGraph->_noeuds.size() < 100)
		runSingle();
	else
		runMultipole();
}


void FastMultipoleEmbedder::runMultipole() {
	//FMEGlobalContext* pGlobalContext = FMEMultipoleKernel::allocateContext(m_pGraph, m_pOptions, m_threadPool->numThreads());
	//m_threadPool->runKernel<FMEMultipoleKernel>(pGlobalContext);
	//FMEMultipoleKernel::deallocateContext(pGlobalContext);
}


void FastMultipoleEmbedder::runSingle() {
	//FMESingleKernel kernel;
	//kernel(*m_pGraph, m_pOptions->timeStep, m_pOptions->minNumIterations, m_pOptions->maxNumIterations, m_pOptions->stopCritForce);
}


void FastMultipoleEmbedder::allocate() {
	m_pOptions = new FMEGlobalOptions();
	m_pGraph = new Graphe();
	initOptions();
}


void FastMultipoleEmbedder::deallocate() {
	delete m_pGraph;
	delete m_pOptions;
}
