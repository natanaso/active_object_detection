#ifndef BeliefTransition_H
#define BeliefTransition_H

#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MObject.h"
#include "BeliefWithState.h"
#include "SparseVector.h"

using namespace momdp;
namespace momdp 
{
	class BeliefTransition
	{
	public:
		BeliefTransition(void)
		{
		}
		virtual ~BeliefTransition(void) 
		{
		}
		applSharedPointer<MObject> problem;
		virtual applSharedPointer<BeliefWithState> nextBelief(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX ) = 0;

		// TODO: somehow eliminate jspv?
		virtual applSharedPointer<BeliefWithState> nextBelief2(applSharedPointer<BeliefWithState> bp, int a, int o, int obsX, applSharedPointer<SparseVector>& jspv ) = 0;

		virtual applSharedPointer<BeliefWithState> nextBelief(applSharedPointer<belief_vector>& belY, DenseVector& belX, int a, int o, int obsX)=0; // SYL07292010 

	};
}


#endif

