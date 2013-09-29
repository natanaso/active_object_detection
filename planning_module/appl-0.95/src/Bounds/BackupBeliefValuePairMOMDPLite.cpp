#include "BackupBeliefValuePairMOMDPLite.h"
#include "exception" 
#include <stdexcept>


using namespace std;
BackupBeliefValuePairMOMDPLite::BackupBeliefValuePairMOMDPLite(void)
{
}

BackupBeliefValuePairMOMDPLite::~BackupBeliefValuePairMOMDPLite(void)
{
}

applSharedPointer<BeliefValuePair> BackupBeliefValuePairMOMDPLite::backup(BeliefTreeNode * node)
{
	throw runtime_error("not implemented");
}
