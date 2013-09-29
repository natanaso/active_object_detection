#include "ObservationProbabilities.h"

ObservationProbabilities::ObservationProbabilities(void)
{
}

ObservationProbabilities::~ObservationProbabilities(void)
{
}


applSharedPointer<SparseMatrix> ObservationProbabilities::getMatrix(int a, int x)
{
	return matrix[a][x];
}
applSharedPointer<SparseMatrix> ObservationProbabilities::getMatrixTr(int a, int x)
{
	return matrixTr[a][x];
}

