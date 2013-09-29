#ifndef BeliefCache_H
#define BeliefCache_H


#include "MObjectManager.h"
#include "Belief.h"
#include "UniqueBeliefHeap.h"
#include "BeliefCacheRow.h"

using namespace momdp;
namespace momdp 
{
	class BeliefCache
	{
	private: 
		std::vector<BeliefCacheRow *> belCache;
		UniqueBeliefHeap* lookupTable;

		void validateRowNumber(int row) ;

	public:
		BeliefCache(void);
		virtual ~BeliefCache(void);

		virtual void localDelete(applSharedPointer<Belief> pointer);



		/**
		* @brief Helper Method to dump belief cache
		* @author Xan
		*/
		ofstream& writeBeliefVector(std::string fileName);
		ofstream& writeBeliefFunction(std::string fileName);



		int currentRowCount;
		BeliefCacheRow* getRow(int row);

		bool hasBelief( applSharedPointer<belief_vector>& bel);

		int addBeliefRow( applSharedPointer<belief_vector>& bel);
		int addBeliefRowWithoutCheck( applSharedPointer<belief_vector>& bel);
		int getBeliefRowIndex( applSharedPointer<belief_vector>& bel);

		int size()
		{
			return belCache.size();
		}


	};
}

#endif

