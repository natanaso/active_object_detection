#ifndef BeliefTransitionMOMDPLite_H
#define BeliefTransitionMOMDPLite_H


#include "BeliefTransition.h"
#include "BeliefWithState.h"
using namespace std;
using namespace momdp;
namespace momdp 
{
class BeliefTransitionMOMDPLite :
	public BeliefTransition
{
public:
	BeliefTransitionMOMDPLite(void);
	virtual ~BeliefTransitionMOMDPLite(void);

	virtual applSharedPointer<BeliefWithState> nextBelief(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX );
	virtual applSharedPointer<BeliefWithState> nextBelief2(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX, applSharedPointer<SparseVector>& jspv );
	virtual applSharedPointer<BeliefWithState> nextBelief(applSharedPointer<belief_vector>& belY, DenseVector& belX, int a, int o, int obsX); // SYL07292010 

};
}

#endif
