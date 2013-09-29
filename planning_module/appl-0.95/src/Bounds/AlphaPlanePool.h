#ifndef AlphaPlanePool_H
#define AlphaPlanePool_H

#include "Bound.h"
#include "Backup.h"
#include "AlphaPlane.h"
#include "IndexedTuple.h"
#include "PruneAlphaPlane.h"

#include <exception>
#include <list>
#include <vector>
#include <stdexcept>
using namespace std;
using namespace momdp;
namespace momdp 
{
	class AlphaPlanePoolDataTuple :public Tuple
	{
	public:
		int ALPHA_TIME_STAMP;
		list<applSharedPointer<AlphaPlane> >* ALPHA_PLANES; /*alpha planes which dominate at this applSharedPointer<Belief> */
	};


	class AlphaPlanePool :	public Bound<AlphaPlane>
	{
	public:
		AlphaPlanePool(Backup<AlphaPlane> *_backupEngine)
		{
			this->setBackupEngine(_backupEngine);
		}
		PruneAlphaPlane* pruneEngine;

		virtual ~AlphaPlanePool(void)
		{
		}



		virtual applSharedPointer<AlphaPlane> backup(BeliefTreeNode * node)
		{
			applSharedPointer<AlphaPlane> result = backupEngine->backup(node);
			for(size_t i = 0 ; i < onBackup.size(); i++)
			{
				(*onBackup[i])(solver, node, result);
			}
			throw runtime_error("Not finished...");
			return result;
		}


		applSharedPointer<MOMDP> problem;
		void setProblem(applSharedPointer<MOMDP> p)
		{
			problem = p;
		}
		void setSolver(PointBasedAlgorithm *p)
		{
			//solver = p;
		}
		BeliefCache *beliefCache;
		void setBeliefCache(BeliefCache *p)
		{
			beliefCache = p;
		}


		IndexedTuple<AlphaPlanePoolDataTuple> *dataTable;
		void setDataTable(IndexedTuple<AlphaPlanePoolDataTuple> *p)
		{
			dataTable = p;
		}

		applSharedPointer<AlphaPlane> getBestAlphaPlane(applSharedPointer<belief_vector>& b);
		applSharedPointer<AlphaPlane> getBestAlphaPlane(BeliefTreeNode& cn);	
		applSharedPointer<AlphaPlane> getBestAlphaPlane1(applSharedPointer<belief_vector>& b);
		applSharedPointer<AlphaPlane> getBestAlphaPlane( applSharedPointer<belief_vector>& b, int index );  //  belief, belief index
	
		virtual double getValue(applSharedPointer<belief_vector>& belief);
		// TODO:: Phase out this
		virtual applSharedPointer<AlphaPlane> getValueAlpha(applSharedPointer<Belief>& belief);

		virtual double getValue(applSharedPointer<belief_vector>& belief, applSharedPointer<AlphaPlane>* bestAlpha);

		list<applSharedPointer<AlphaPlane> > planes;
		void addAlphaPlane(applSharedPointer<AlphaPlane> plane);
	private:
		
		
	};
}

#endif 

