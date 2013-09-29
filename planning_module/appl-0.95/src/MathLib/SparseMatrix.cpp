#include "SparseMatrix.h"

#include "DenseVector.h"
using namespace momdp;
namespace momdp
{
	REAL_VALUE SparseMatrix::operator()(int r, int c) const
	{

		vector<SparseVector_Entry>::const_iterator  di;

		vector<SparseVector_Entry>::const_iterator  col_end = data.begin() + col_starts[c+1];

		for (di = data.begin() + col_starts[c]; di != col_end; di++) {
			if (di->index >= r) {
				if (di->index == r) {
					return di->value;
				} else {
					return 0.0;
				}
			}
		}
		return 0.0;
	}
	DenseVector* SparseMatrix::mult(DenseVector& x)
	{
		// 		SparseVector tmp;
		// 		copy(tmp, x);
		// 		mult(result, A, tmp);
		vector<SparseVector_Entry>::const_iterator  Ai, col_end;

		double xval;
		DenseVector *result = new DenseVector( x.size());

		FOR(xind, x.size())
		{
			xval = x.data[xind];
			col_end = data.begin() + col_starts[xind+1];
			for (Ai = data.begin() + col_starts[xind]; Ai != col_end; Ai++)
			{
				(*result)(Ai->index) += xval * Ai->value;
			}
		}

		return result;
	}
	DenseVector* SparseMatrix::mult(SparseVector& x)
	{
		vector<SparseVector_Entry>::const_iterator  Ai, col_end;

		int xind;
		double xval;
		DenseVector *result = new DenseVector( x.size());

		FOREACH(SparseVector_Entry, xi,  x.data) 
		{
			xind = xi->index;
			xval = xi->value;
			col_end = data.begin() + col_starts[xind+1];
			for (Ai = data.begin() + col_starts[xind];
				Ai != col_end;
				Ai++) 
			{
					(*result)(Ai->index) += xval * Ai->value;
			}
		}

		return result;
	}

	void SparseMatrix::resize(int _size1, int _size2)
	{
		size1_ = _size1;
		size2_ = _size2;
		col_starts.resize( size2()+1 );
		FOREACH_NOCONST(int, ci,  col_starts) {
			(*ci) = 0;
		}
		data.clear();
	}

	void SparseMatrix::push_back(int r, int c, REAL_VALUE value)
	{
		//data.push_back( SparseVector_Entry( r, value ) );
		int curSize = data.size();
		data.resize(curSize+ 1);
		data[curSize].index = r;
		data[curSize].value =value;

		col_starts[c+1] = data.size();
	}

	void SparseMatrix::canonicalize(void)
	{
		FOR (i, size2_) 
		{
			if (col_starts[i] > col_starts[i+1])
			{
				col_starts[i+1] = col_starts[i];
			}
		}
	}

	void SparseMatrix::read(std::istream& in)
	{
		// Need to be tested		
		int rows, cols;
		int num_entries;
		int r, c;
		REAL_VALUE value;

		in >> rows >> cols;
		resize( rows, cols );

		in >> num_entries;
		FOR (i, num_entries) 
		{
			in >> r >> c >> value;
			push_back( r, c, value );
		}
	}

	std::ostream& SparseMatrix::write(std::ostream& out) const
	{
		vector<SparseVector_Entry>::const_iterator  di, col_end;


		out << size1_ << " " << size2_ << std::endl;
		out << data.size() << std::endl;
		FOR (c, size2_) {
			col_end = data.begin() + col_starts[c+1];
			for (di = data.begin() + col_starts[c]; di != col_end; di++) {
				out << di->index << " " << c << " " << di->value << std::endl;
			}
		}
		return out;
	}

	REAL_VALUE SparseMatrix::getMaxValue()
	{
		REAL_VALUE maxVal = data.begin()->value;
		REAL_VALUE val;
		FOREACH(SparseVector_Entry, entry,  data) 
		{
			val = entry->value;
			if(val>maxVal){
				maxVal = val;
			}
		}//end FOR_EACH
		return maxVal;
	}

	bool SparseMatrix::isColumnEmpty(int c) const 
	{
		int col_start = col_starts[c];
		int col_end   = col_starts[c+1];

		return col_start == col_end;
	}
}

