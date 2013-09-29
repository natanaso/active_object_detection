#ifndef PolicyGraphGenerator_H
#define PolicyGraphGenerator_H

#include <vector>
#include <ostream>
#include <string>
#include <set>
#include "MOMDP.h"
#include "BeliefTreeNode.h"
using namespace std;
using namespace momdp;

namespace momdp
{
    class AlphaVectorPolicy;
    class BeliefForest;
    class BeliefTreeNode;
    class BeliefCache;
    class Sample;
    class SolverParams;

    class NodeRelation
    {
	public:
	    BeliefTreeNode *srcNode;
	    BeliefTreeNode *destNode;
	    int X;
	    double xProb;

	    int o;
	    double oProb;

    };

    class PolicyGraphParam
    {
	public:
	    bool useLookahead;
	    int depth;
	    double probThreshold;		//only show edges with probability above probThreshold
	    int maxEdge;			//max number of outgoing edges from a node	

	    PolicyGraphParam()
	    {
		depth = 0; 		//depth of the graph, 0 for no limit
		useLookahead = true;
		probThreshold = 0; 	
		maxEdge = 0; 		
	    }

    };

    class PolicyGraphGenerator
    {
	private:
	    applSharedPointer<MOMDP> problem;
	    applSharedPointer<AlphaVectorPolicy> policy;
	    BeliefForest* beliefForest;
	    Sample* sampleEngine;
	    vector <BeliefCache *>  beliefCacheSet;
	    PolicyGraphParam graphParam;

	    set<BeliefTreeNode*> nodesList;		//list of node used in graph
	    vector<NodeRelation> nodeRelationsList;	//list of edges

	    void expandNode(BeliefTreeNode* curNode, int bestAction, vector<NodeRelation>& expansion);
	    void generateNodesDot(ostream& output, BeliefTreeNode* node);
	    string formatTuple(string name, int index, map<string, string> dataMap);
	    void convertStCacheIndex(ostream& output, cacherow_stval& stRowIndex);
	    void generateNodesRelation(ostream& output, vector<NodeRelation>& nodeRelations, set<BeliefTreeNode*> firstLevelNodes);
	    void drawRootNodeDot(ostream& output, applSharedPointer<SparseVector> initialBeliefY, applSharedPointer<DenseVector> initialBeliefX, int bestAction);
	    void dfsBeliefForest(BeliefTreeNode* curNode, vector<BeliefTreeNode*>& path ,int level);

	public:
	    PolicyGraphGenerator(applSharedPointer<MOMDP> _problem, applSharedPointer<AlphaVectorPolicy> _policy, PolicyGraphParam _graphParam);
	    void generateGraph(ostream& output);	//generate policy graph in DOT format
	    ~PolicyGraphGenerator(void);
    };

}
#endif

