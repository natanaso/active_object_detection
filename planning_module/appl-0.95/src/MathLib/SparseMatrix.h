#ifndef SparseMatrix_H
#define SparseMatrix_H

#include <vector>
#include <string>
#include <iostream>
#include <cassert>

#include "MObject.h"
#include "SparseVector.h"


using namespace std;
using namespace momdp;
namespace momdp 
{

	class DenseVector;

	class SparseMatrix : public MObject
	{
		friend class SparseVector;
		friend class DenseVector;
	public:
		vector< SparseVector_Entry > data;
		int size1_, size2_;
		vector< int > col_starts;

	private:



	public:
		SparseMatrix(void) : size1_(0), size2_(0) 
		{}

		SparseMatrix(int _size1, int _size2) 
		{
			resize(_size1,_size2); 
		}
		virtual ~SparseMatrix(void)
		{
		}

		REAL_VALUE operator()(int r, int c) const;
		SparseVector& getCol(int c) const;

		int size1(void) const { return size1_; }
		int size2(void) const { return size2_; }
		int filled(void) const { return data.size(); }

		void resize(int _size1, int _size2);

		void push_back(int row, int col, REAL_VALUE value);

		// if resize()/push_back() are used to initialize, you must call
		// canonicalize() before performing any operations with the matrix

		void canonicalize(void);
		bool isColumnEmpty(int c) const ;

		// Arithmetic
		REAL_VALUE getMaxValue();

		// result = A * x
		DenseVector* mult(SparseVector& x);
		DenseVector* mult(DenseVector& x);


		template <class T, class U> void emult_cc_internal(SparseVector& result, T xbegin, T xend,  U ybegin, U yend)
		{
			U yi = ybegin;
			for (T xi = xbegin; xi != xend; xi++) {
				while (1) {
					if (yi == yend) return;
					if (yi->index >= xi->index) {
						if (yi->index == xi->index) {
							result.push_back( xi->index, xi->value * yi->value);
						}
						break;
					}
					yi++;
				}
			}
		}

		applSharedPointer<SparseVector> emult_column(const SparseMatrix& A, int c, const SparseVector& x)
		{
			assert( A.size1() == x.size() );
			assert( 0 <= c && c < A.size2() );

			applSharedPointer<SparseVector> result (new SparseVector( x.size() ));

			emult_cc_internal( *result,
				A.data.begin() + A.col_starts[c],
				A.data.begin() + A.col_starts[c+1],
				x.data.begin(), x.data.end() );

			//result->finalize();
			return result;
		}

		// IO
		void read(std::istream& in);
		ostream& write(std::ostream& out) const;

	};
}

#endif

