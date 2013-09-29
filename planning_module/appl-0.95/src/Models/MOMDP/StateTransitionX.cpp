#include "StateTransitionX.h"
#include "MOMDP.h"

StateTransitionX::StateTransitionX(void)
{
}

StateTransitionX::~StateTransitionX(void)
{
}

// (unobserved states, observed states)
applSharedPointer<SparseMatrix> StateTransitionX::getMatrix(int a, int x)
{
	return matrix[a][x];
}
applSharedPointer<SparseMatrix> StateTransitionX::getMatrixTr(int a, int x)
{
	return matrixTr[a][x];
}
