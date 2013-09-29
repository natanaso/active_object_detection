#ifndef BackupBeliefValuePairMOMDP_H
#define BackupBeliefValuePairMOMDP_H

#include "Backup.h"
#include "BeliefValuePair.h"
#include "BeliefValuePairPool.h"
#include "BeliefValuePairPoolSet.h"
#include "IndexedTuple.h"
using namespace momdp;
namespace momdp 
{
	class BeliefValuePairPool;
	class MOMDP;

	class BackupBeliefValuePairMOMDP : public Backup<BeliefValuePair>
	{
	public:
		//REMOVE:: IndexedTuple<BeliefValuePairPoolDataTuple> *dataTable;, use individual BeliefValuePairPools's datatable
		applSharedPointer<MOMDP> problem;
		BeliefValuePairPoolSet *boundSet;
		


		BackupBeliefValuePairMOMDP(void);
		virtual ~BackupBeliefValuePairMOMDP(void);

		void setProblem(applSharedPointer<MOMDP> p)
		{
			problem = p;
		}
		void setSolver(PointBasedAlgorithm *p)
		{
			//solver = p;
		}
		void setBound(PointBasedAlgorithm *p)
		{

		}

		virtual applSharedPointer<BeliefValuePair> backup(BeliefTreeNode * node);
		virtual	double getNewUBValueQ(BeliefTreeNode& cn, int a);
		virtual double getNewUBValueSimple(BeliefTreeNode& cn, int* maxUBActionP);
		virtual double getNewUBValueUseCache(BeliefTreeNode& cn, int* maxUBActionP);
		virtual double getNewUBValue(BeliefTreeNode& cn, int* maxUBActionP);

	};
}


#endif

