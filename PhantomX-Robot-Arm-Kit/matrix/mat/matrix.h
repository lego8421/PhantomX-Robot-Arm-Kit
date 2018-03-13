////////////////////////////////
// Matrix TCL Lite v1.13
// Copyright (c) 1997-2002 Techsoft Pvt. Ltd. (See License.Txt file.)
//
// Matrix.h: Matrix C++ template class include file 
// Web: http://www.techsoftpl.com/matrix/
// Email: matrix@techsoftpl.com
//

#if !defined(__STD_MATRIX_H)
#define __STD_MATRIX_H

#include <cmath>
#include <valarray>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>

using namespace std;

inline void _matrix_error (const char* pErrMsg)
{
	cout << pErrMsg << endl;
	exit(1);
}

template <class T>
class matrix
{
public:
	// Constructors
	matrix (const matrix<T>& m);
	matrix (size_t row = 1, size_t col = 1);

	// Destructor
	~matrix ();

	// Assignment operators
	matrix<T>& operator = (const matrix<T>& m);

	// Value extraction method
	size_t RowNo () const { return _m->Row; }
	size_t ColNo () const { return _m->Col; }

	// Subscript operator
	T& operator () (size_t row, size_t col);
	T  operator () (size_t row, size_t col) const;

	// Unary operators
	matrix<T> operator + () { return *this; }
	matrix<T> operator - ();

	// Combined assignment - calculation operators
	matrix<T>& operator += (const matrix<T>& m);
	matrix<T>& operator -= (const matrix<T>& m);
	matrix<T>& operator *= (const matrix<T>& m);
	matrix<T>& operator *= (const T& c);
	matrix<T>& operator /= (const T& c);
	matrix<T>& operator ^= (const size_t& pow);

	// Miscellaneous -methods
	void Null (const size_t& row, const size_t& col);
	void Null ();
	void Unit (const size_t& row);
	void Unit ();
	void SetSize (size_t row, size_t col);

	// Utility methods
	matrix<T> Solve (const matrix<T>& v) const;
	matrix<T> Adj ();
	matrix<T> Inv ();
	T Det () const;
	T Norm ();
	T Cofact (size_t row, size_t col);
	T Cond ();

	// Type of matrices
	bool IsSquare () { return (_m->Row == _m->Col); } 
	bool IsSingular ();
	bool IsDiagonal ();
	bool IsScalar ();
	bool IsUnit ();
	bool IsNull ();
	bool IsSymmetric ();
	bool IsSkewSymmetric ();
	bool IsUpperTriangular ();
	bool IsLowerTriangular ();

private:
	struct base_mat {
		T **Val;
		T *_Val;	// page365: 메모리 할 당을 두 번만 하도록 하기 위해서 추가함
		size_t Row, Col, RowSiz, ColSiz;
		int Refcnt;

		base_mat (size_t row, size_t col, T** v)
		{
			Row = row; RowSiz = row;
			Col = col; ColSiz = col;
			Refcnt = 1;

			Val = new T* [row];
			_Val = new T [row*col];
			size_t rowlen = col * sizeof(T);

			for (size_t i=0; i < row; i++)
			{
				// Val[i] = new T [col];
				Val[i] = &_Val[i*col];
				if (v) memcpy( Val[i], v[i], rowlen);
			}
		}
		~base_mat ()
		{
			//for (size_t i=0; i < RowSiz; i++)
			//	delete [] Val[i];
			delete [] Val;
			delete [] _Val;
		}
	};
	base_mat *_m;

	void clone ();
	void realloc (size_t row, size_t col);
	int pivot (size_t row);
};

// constructor
template <class T> inline
matrix<T>::matrix (size_t row, size_t col)
{
	_m = new base_mat( row, col, 0);
}

// copy constructor
template <class T> inline
matrix<T>::matrix (const matrix<T>& m)
{
	_m = m._m;
	_m->Refcnt++;
}

// Internal copy constructor
template <class T> inline void
matrix<T>::clone ()
{
	_m->Refcnt--;
	_m = new base_mat( _m->Row, _m->Col, _m->Val);
}

// destructor
template <class T> inline
matrix<T>::~matrix ()
{
	if (--_m->Refcnt == 0) delete _m;
}

// assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator = (const matrix<T>& m)
{
	m._m->Refcnt++;
	if (--_m->Refcnt == 0) delete _m;
	_m = m._m;
	return *this;
}

//  reallocation method
template <class T> inline void 
matrix<T>::realloc (size_t row, size_t col)
{
	if (row == _m->RowSiz && col == _m->ColSiz)
	{
		_m->Row = _m->RowSiz;
		_m->Col = _m->ColSiz;
		return;
	}

	base_mat *m1 = new base_mat( row, col, NULL);
	size_t colSize = min(_m->Col,col) * sizeof(T);
	size_t minRow = min(_m->Row,row);

	for (size_t i=0; i < minRow; i++)
		memcpy( m1->Val[i], _m->Val[i], colSize);

	if (--_m->Refcnt == 0) 
		delete _m;
	_m = m1;

	return;
}

// public method for resizing matrix
template <class T> inline void
matrix<T>::SetSize (size_t row, size_t col)
{
	size_t i,j;
	size_t oldRow = _m->Row;
	size_t oldCol = _m->Col;

	if (row != _m->RowSiz || col != _m->ColSiz)
		realloc( row, col);

	for (i=oldRow; i < row; i++)
		for (j=0; j < col; j++)
			_m->Val[i][j] = T(0);

	for (i=0; i < row; i++)                      
		for (j=oldCol; j < col; j++)
			_m->Val[i][j] = T(0);

	return;
}

// subscript operator to get/set individual elements
template <class T> inline T&
matrix<T>::operator () (size_t row, size_t col)
{
	if (row >= _m->Row || col >= _m->Col)
		_matrix_error( "matrix<T>::operator(): Index out of range!");

	if (_m->Refcnt > 1) clone();

	return _m->Val[row][col];
}

// subscript operator to get/set individual elements
template <class T> inline T
matrix<T>::operator () (size_t row, size_t col) const
{
	if (row >= _m->Row || col >= _m->Col)
		_matrix_error( "matrix<T>::operator(): Index out of range!");

	return _m->Val[row][col];
}

// input stream function
template <class T> inline istream&
operator >> (istream& istrm, matrix<T>& m)
{
	for (size_t i=0; i < m.RowNo(); i++)
		for (size_t j=0; j < m.ColNo(); j++)
		{
			T x;
			istrm >> x;
			m(i,j) = x;
		}

	return istrm;
}

// output stream function
template <class T> inline ostream&
operator << (ostream& ostrm, const matrix<T>& m)
{
	for (size_t i=0; i < m.RowNo(); i++)
	{
		for (size_t j=0; j < m.ColNo(); j++)
		{
			T x = m(i,j);
			ostrm << x << '\t';
		}
		ostrm << endl;
	}
	return ostrm;
}


// logical equal-to operator
template <class T> inline bool
operator == (const matrix<T>& m1, const matrix<T>& m2)
{
	if (m1.RowNo() != m2.RowNo() || m1.ColNo() != m2.ColNo())
		return false;

	for (size_t i=0; i < m1.RowNo(); i++)
		for (size_t j=0; j < m1.ColNo(); j++)
			if (m1(i,j) != m2(i,j))
				return false;

	return true;
}

// logical no-equal-to operator
template <class T> inline bool
operator != (const matrix<T>& m1, const matrix<T>& m2)
{
	return (m1 == m2) ? false : true;
}

// combined addition and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator += (const matrix<T>& m)
{
	if (_m->Row != m._m->Row || _m->Col != m._m->Col)
		_matrix_error( "matrix<T>::operator+= : Inconsistent matrix sizes in addition!");
	if (_m->Refcnt > 1) clone();
	for (size_t i=0; i < m._m->Row; i++)
		for (size_t j=0; j < m._m->Col; j++)
			_m->Val[i][j] += m._m->Val[i][j];

	return *this;
}

// combined subtraction and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator -= (const matrix<T>& m)
{
	if (_m->Row != m._m->Row || _m->Col != m._m->Col)
		_matrix_error( "matrix<T>::operator-= : Inconsistent matrix sizes in subtraction!");
	if (_m->Refcnt > 1) clone();
	for (size_t i=0; i < m._m->Row; i++)
		for (size_t j=0; j < m._m->Col; j++)
			_m->Val[i][j] -= m._m->Val[i][j];

	return *this;
}

// combined scalar multiplication and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator *= (const T& c)
{
	if (_m->Refcnt > 1) clone();
	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			_m->Val[i][j] *= c;

	return *this;
}

// combined matrix multiplication and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator *= (const matrix<T>& m)
{
	if (_m->Col != m._m->Row)
		_matrix_error( "matrix<T>::operator*= : Inconsistent matrix sizes in multiplication!");

	matrix<T> temp(_m->Row,m._m->Col);

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < m._m->Col; j++)
		{
			temp._m->Val[i][j] = T(0);
			for (size_t k=0; k < _m->Col; k++)
				temp._m->Val[i][j] += _m->Val[i][k] * m._m->Val[k][j];
		}
	
	*this = temp;

	return *this;
}

// combined scalar division and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator /= (const T& c)
{
	if (_m->Refcnt > 1) clone();
	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			_m->Val[i][j] /= c;

	return *this;
}

// combined power and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator ^= (const size_t& pow)
{
	matrix<T> temp(*this);

	for (size_t i=2; i <= pow; i++)
		*this = *this * temp;

	return *this;
}

// unary negation operator
template <class T> inline matrix<T>
matrix<T>::operator - ()
{
	matrix<T> temp(_m->Row,_m->Col);

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			temp._m->Val[i][j] = - _m->Val[i][j];

	return temp;
}

// binary addition operator
template <class T> inline matrix<T>
operator + (const matrix<T>& m1, const matrix<T>& m2)
{
	matrix<T> temp = m1;
	temp += m2;
	return temp;
}

// binary subtraction operator
template <class T> inline matrix<T>
operator - (const matrix<T>& m1, const matrix<T>& m2)
{
	matrix<T> temp = m1;
	temp -= m2;
	return temp;
}

// binary scalar multiplication operator
template <class T> inline matrix<T>
operator * (const matrix<T>& m, const T& no)
{
	matrix<T> temp = m;
	temp *= no;
	return temp;
}


// binary scalar multiplication operator
template <class T> inline matrix<T>
operator * (const T& no, const matrix<T>& m)
{
	return (m * no);
}

// binary matrix multiplication operator
template <class T> inline matrix<T>
operator * (const matrix<T>& m1, const matrix<T>& m2)
{
	matrix<T> temp = m1;
	temp *= m2;
	return temp;
}

// binary scalar division operator
template <class T> inline matrix<T>
operator / (const matrix<T>& m, const T& no)
{
	return (m * (T(1) / no));
}


// binary scalar division operator
template <class T> inline matrix<T>
operator / (const T& no, const matrix<T>& m)
{
	return (!m * no);
}

// binary matrix division operator
template <class T> inline matrix<T>
operator / (const matrix<T>& m1, const matrix<T>& m2)
{
	return (m1 * !m2);
}

// binary power operator
template <class T> inline matrix<T>
operator ^ (const matrix<T>& m, const size_t& pow)
{
	matrix<T> temp = m;
	temp ^= pow;
	return temp;
}

// unary transpose operator
template <class T> inline matrix<T>
operator ~ (const matrix<T>& m)
{
	matrix<T> temp(m.ColNo(),m.RowNo());

	for (size_t i=0; i < m.RowNo(); i++)
		for (size_t j=0; j < m.ColNo(); j++)
		{
			T x = m(i,j);
			temp(j,i) = x;
		}

	return temp;
}

// unary inversion operator
template <class T> inline matrix<T>
operator ! (const matrix<T> m)
{
	matrix<T> temp = m;
	return temp.Inv();
}

// inversion function
template <class T> inline matrix<T>
matrix<T>::Inv ()
{
	size_t i,j,k;
	T a1,a2,*rowptr;

	if (_m->Row != _m->Col)
		_matrix_error( "matrix<T>::operator!: Inversion of a non-square matrix");

	matrix<T> temp(_m->Row,_m->Col);
	if (_m->Refcnt > 1) clone();

	temp.Unit();
	
	for (k=0; k < _m->Row; k++)
	{
		int indx = pivot(k);
		if (indx == -1)
			_matrix_error( "matrix<T>::operator!: Inversion of a singular matrix");

		if (indx != 0)
		{
			rowptr = temp._m->Val[k];
			temp._m->Val[k] = temp._m->Val[indx];
			temp._m->Val[indx] = rowptr;
		}
		
		a1 = _m->Val[k][k];
		for (j=0; j < _m->Row; j++)
		{
			_m->Val[k][j] /= a1;
			temp._m->Val[k][j] /= a1;
		}

		for (i=0; i < _m->Row; i++)
			if (i != k)
			{
				a2 = _m->Val[i][k];
				for (j=0; j < _m->Row; j++)
				{
					_m->Val[i][j] -= a2 * _m->Val[k][j];
					temp._m->Val[i][j] -= a2 * temp._m->Val[k][j];
				}
			}
	}

	return temp;
}

// solve simultaneous equation
template <class T> inline matrix<T>
matrix<T>::Solve (const matrix<T>& v) const
{
	size_t i,j,k;
	T a1;

	if (!(_m->Row == _m->Col && _m->Col == v._m->Row))
		_matrix_error( "matrix<T>::Solve():Inconsistent matrices!");

	matrix<T> temp(_m->Row,_m->Col+v._m->Col);
	for (i=0; i < _m->Row; i++)
	{
		for (j=0; j < _m->Col; j++)
			temp._m->Val[i][j] = _m->Val[i][j];
		for (k=0; k < v._m->Col; k++)
			temp._m->Val[i][_m->Col+k] = v._m->Val[i][k];
	}

	for (k=0; k < _m->Row; k++)
	{
		int indx = temp.pivot(k);
		if (indx == -1)
			_matrix_error( "matrix<T>::Solve(): Singular matrix!");

		a1 = temp._m->Val[k][k];
		for (j=k; j < temp._m->Col; j++)
			temp._m->Val[k][j] /= a1;

		for (i=k+1; i < _m->Row; i++)
		{
			a1 = temp._m->Val[i][k];
			for (j=k; j < temp._m->Col; j++)
				temp._m->Val[i][j] -= a1 * temp._m->Val[k][j];
		}
	}

	matrix<T> s(v._m->Row,v._m->Col);
	for (k=0; k < v._m->Col; k++)
		for (int m=int(_m->Row)-1; m >= 0; m--)
		{
			s._m->Val[m][k] = temp._m->Val[m][_m->Col+k];
			for (j=m+1; j < _m->Col; j++)
				s._m->Val[m][k] -= temp._m->Val[m][j] * s._m->Val[j][k];
		}

	return s;
}

// set zero to all elements of this matrix
template <class T> inline void
matrix<T>::Null (const size_t& row, const size_t& col)
{
	if (row != _m->Row || col != _m->Col)
		realloc( row,col);

	if (_m->Refcnt > 1) 
		clone();

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			_m->Val[i][j] = T(0);

	return;
}

// set zero to all elements of this matrix
template <class T> inline void
matrix<T>::Null()
{
	if (_m->Refcnt > 1) clone();   
	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			_m->Val[i][j] = T(0);

	return;
}

// set this matrix to unity
template <class T> inline void
matrix<T>::Unit (const size_t& row)
{
	if (row != _m->Row || row != _m->Col)
		realloc( row, row);

	if (_m->Refcnt > 1) 
		clone();

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			_m->Val[i][j] = i == j ? T(1) : T(0);

	return;
}

// set this matrix to unity
template <class T> inline void
matrix<T>::Unit ()
{
	if (_m->Refcnt > 1) clone();   
	size_t row = min(_m->Row,_m->Col);
	_m->Row = _m->Col = row;

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			_m->Val[i][j] = i == j ? T(1) : T(0);

	return;
}

// private partial pivoting method
template <class T> inline int
matrix<T>::pivot (size_t row)
{
	int k = int(row);
	double amax,temp;

	amax = -1;
	for (size_t i=row; i < _m->Row; i++)
		if ( (temp = abs( _m->Val[i][row])) > amax && temp != 0.0)
		{
			amax = temp;
			k = i;
		}
		if (_m->Val[k][row] == T(0))
			return -1;

		if (k != int(row))
		{
			T* rowptr = _m->Val[k];
			_m->Val[k] = _m->Val[row];
			_m->Val[row] = rowptr;
			return k;
		}
		return 0;
}

// calculate the determinant of a matrix
template <class T> inline T
matrix<T>::Det () const
{
	size_t i,j,k;
	T piv,detVal = T(1);

	if (_m->Row != _m->Col)
		_matrix_error( "matrix<T>::Det(): Determinant a non-square matrix!");

	matrix<T> temp(*this);
	if (temp._m->Refcnt > 1) temp.clone();

	for (k=0; k < _m->Row; k++)
	{
		int indx = temp.pivot(k);
		if (indx == -1)
			return 0;
		if (indx != 0)
			detVal = - detVal;
		detVal = detVal * temp._m->Val[k][k];
		for (i=k+1; i < _m->Row; i++)
		{
			piv = temp._m->Val[i][k] / temp._m->Val[k][k];
			for (j=k+1; j < _m->Row; j++)
				temp._m->Val[i][j] -= piv * temp._m->Val[k][j];
		}
	}
	return detVal;
}

// calculate the norm of a matrix
template <class T> inline T
matrix<T>::Norm ()
{
	T retVal = T(0);

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			retVal += _m->Val[i][j] * _m->Val[i][j];

	retVal = sqrt( retVal);

	return retVal;
}

// calculate the condition number of a matrix
template <class T> inline T
matrix<T>::Cond ()
{
	matrix<T> inv = ! (*this);
	return (Norm() * inv.Norm());
}

// calculate the cofactor of a matrix for a given element
template <class T> inline T
matrix<T>::Cofact (size_t row, size_t col)
{
	size_t i,i1,j,j1;

	if (_m->Row != _m->Col)
		_matrix_error( "matrix<T>::Cofact(): Cofactor of a non-square matrix!");

	if (row > _m->Row || col > _m->Col)
		_matrix_error( "matrix<T>::Cofact(): Index out of range!");

	matrix<T> temp (_m->Row-1,_m->Col-1);

	for (i=i1=0; i < _m->Row; i++)
	{
		if (i == row)
			continue;
		for (j=j1=0; j < _m->Col; j++)
		{
			if (j == col)
				continue;
			temp._m->Val[i1][j1] = _m->Val[i][j];
			j1++;
		}
		i1++;
	}

	T  cof = temp.Det();
	if ((row+col)%2 == 1)
		cof = -cof;

	return cof;
}


// calculate adjoin of a matrix
template <class T> inline matrix<T>
matrix<T>::Adj ()
{
	if (_m->Row != _m->Col)
		_matrix_error( "matrix<T>::Adj(): Adjoin of a non-square matrix.");

	matrix<T> temp(_m->Row,_m->Col);

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			temp._m->Val[j][i] = Cofact(i,j);

	return temp;
}

// Determine if the matrix is singular
template <class T> inline bool
matrix<T>::IsSingular ()
{
	if (_m->Row != _m->Col)
		return false;

	return (Det() == T(0));
}

// Determine if the matrix is diagonal
template <class T> inline bool
matrix<T>::IsDiagonal ()
{
	if (_m->Row != _m->Col)
		return false;

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			if (i != j && _m->Val[i][j] != T(0))
				return false;

	return true;
}

// Determine if the matrix is scalar
template <class T> inline bool
matrix<T>::IsScalar ()
{
	if (!IsDiagonal())
		return false;

	T v = _m->Val[0][0];
	for (size_t i=1; i < _m->Row; i++)
		if (_m->Val[i][i] != v)
			return false;

	return true;
}

// Determine if the matrix is a unit matrix
template <class T> inline bool
matrix<T>::IsUnit ()
{
	if (IsScalar() && _m->Val[0][0] == T(1))
		return true;

	return false;
}

// Determine if this is a null matrix
template <class T> inline bool
matrix<T>::IsNull ()
{
	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			if (_m->Val[i][j] != T(0))
				return false;

	return true;
}

// Determine if the matrix is symmetric
template <class T> inline bool
matrix<T>::IsSymmetric ()
{
	if (_m->Row != _m->Col)
		return false;

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			if (_m->Val[i][j] != _m->Val[j][i])
				return false;

	return true;
}

// Determine if the matrix is skew-symmetric
template <class T> inline bool
matrix<T>::IsSkewSymmetric ()
{
	if (_m->Row != _m->Col)
		return false;

	for (size_t i=0; i < _m->Row; i++)
		for (size_t j=0; j < _m->Col; j++)
			if (_m->Val[i][j] != -_m->Val[j][i])
				return false;

	return true;
}

// Determine if the matrix is upper triangular
template <class T> inline bool
matrix<T>::IsUpperTriangular ()
{
	if (_m->Row != _m->Col)
		return false;
	for (size_t i=1; i < _m->Row; i++)
		for (size_t j=0; j < i-1; j++)
			if (_m->Val[i][j] != T(0))
				return false;

	return true;
}

// Determine if the matrix is lower triangular
template <class T> inline bool
matrix<T>::IsLowerTriangular ()
{
	if (_m->Row != _m->Col)
		return false;

	for (size_t j=1; j < _m->Col; j++)
		for (size_t i=0; i < j-1; i++)
			if (_m->Val[i][j] != T(0))
				return false;

	return true;
}

#endif //__STD_MATRIX_H
