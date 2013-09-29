#ifndef BeliefTransitionMOMDP_H
#define BeliefTransitionMOMDP_H


#include "BeliefTransition.h"
#include "BeliefWithState.h"

using namespace std;
using namespace momdp;
namespace momdp 
{
class BeliefTransitionMOMDP :
	public BeliefTransition
{
public:
	BeliefTransitionMOMDP(void);
	virtual ~BeliefTransitionMOMDP(void);

	// Where DenseVector (b_x) and belief_vector (b_y) are given as input instead of a BeliefWithState ((x,b_y)), it's for the case where the initial distribution over x is a belief (and not a delta)
	virtual applSharedPointer<BeliefWithState> nextBelief(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX );
	virtual applSharedPointer<BeliefWithState> nextBelief2(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX, applSharedPointer<SparseVector>& jspv );
	virtual applSharedPointer<BeliefWithState> nextBelief(applSharedPointer<belief_vector>& belY, DenseVector& belX, int a, int o, int obsX); // SYL07292010 

};
}

#endif
