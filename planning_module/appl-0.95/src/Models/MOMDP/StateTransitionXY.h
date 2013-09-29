#ifndef StateTransitionXY_H
#define StateTransitionXY_H

#include "Const.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MathLib.h"
#include "VariableRelation.h"
#include "StateTransitionY.h"

using namespace std;
using namespace momdp;
namespace momdp 
{
	class MOMDP;
	class StateTransitionXY : public StateTransitionY
	{
		friend class MOMDP;
	private:
		vector<vector<applSharedPointer<SparseMatrix> > > matrix;
		vector<vector<applSharedPointer<SparseMatrix> > > matrixTr;

	public:
		StateTransitionXY(void);
		virtual ~StateTransitionXY(void);

		virtual applSharedPointer<SparseMatrix> getMatrix(int a, int x, int xp);
		virtual applSharedPointer<SparseMatrix> getMatrixTr(int a, int x, int xp);
	};
}

#endif

