#include "StateTransitionXXpY.h"

StateTransitionXXpY::StateTransitionXXpY(void)
{
}

StateTransitionXXpY::~StateTransitionXXpY(void)
{
}

applSharedPointer<SparseMatrix> StateTransitionXXpY::getMatrix(int a, int x, int xp)
{
  return matrix[a][x][xp];
}
applSharedPointer<SparseMatrix> StateTransitionXXpY::getMatrixTr(int a, int x, int xp)
{
  return matrixTr[a][x][xp];
}
