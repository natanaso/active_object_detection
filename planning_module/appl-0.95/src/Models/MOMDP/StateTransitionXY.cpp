#include "StateTransitionXY.h"

StateTransitionXY::StateTransitionXY(void)
{
}

StateTransitionXY::~StateTransitionXY(void)
{
}

applSharedPointer<SparseMatrix> StateTransitionXY::getMatrix(int a, int x, int xp)
{
	return matrix[a][x];
}
applSharedPointer<SparseMatrix> StateTransitionXY::getMatrixTr(int a, int x, int xp)
{
	return matrixTr[a][x];
}
