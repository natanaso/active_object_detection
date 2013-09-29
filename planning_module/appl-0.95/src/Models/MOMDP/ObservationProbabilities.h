#ifndef ObservationProbabilities_H
#define ObservationProbabilities_H

#include "Const.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MathLib.h"
#include "VariableRelation.h"

using namespace std;
using namespace momdp;
namespace momdp 
{
	class MOMDP;
	class ObservationProbabilities : public MObject
	{
		friend class MOMDP;
	private:
		vector<vector<applSharedPointer<SparseMatrix> > > matrix;
		vector<vector<applSharedPointer<SparseMatrix> > > matrixTr;

	public:
		ObservationProbabilities(void);
		virtual ~ObservationProbabilities(void);

		REAL_VALUE prob(Observations::iterator& o, States::iterator& x, States::iterator& y, Actions::iterator& a);


		virtual applSharedPointer<SparseMatrix> getMatrix(int a, int x);
		virtual applSharedPointer<SparseMatrix> getMatrixTr(int a, int x);
		
	};
}

#endif




