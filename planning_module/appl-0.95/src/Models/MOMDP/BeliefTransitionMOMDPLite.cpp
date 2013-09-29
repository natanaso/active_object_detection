#include "BeliefTransitionMOMDPLite.h"
#include "exception" 
#include <stdexcept>
using namespace std;

BeliefTransitionMOMDPLite::BeliefTransitionMOMDPLite(void)
{
}

BeliefTransitionMOMDPLite::~BeliefTransitionMOMDPLite(void)
{
}


applSharedPointer<BeliefWithState> BeliefTransitionMOMDPLite::nextBelief(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX )
{
	throw runtime_error("not implemented");
}
applSharedPointer<BeliefWithState> BeliefTransitionMOMDPLite::nextBelief2(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX, applSharedPointer<SparseVector>& jspv )
{
	throw runtime_error("not implemented");
}

