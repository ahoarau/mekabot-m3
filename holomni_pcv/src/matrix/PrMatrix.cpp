/***************************************************************
 * PrMatrix.cpp
 *
 * This implements a row x col matrix class.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 11/12/97: K.C. Chang: created.
 */

#include <stdio.h>
#include <assert.h>
#include "PrMatrix.h"


PrMatrix::PrMatrix(int _row,int _col)
: row_(_row),col_(_col),n_(_row*_col)
{
    assert(row_>0 && col_>0);

    data_ = new Float[n_];
    zero();
}

PrMatrix::~PrMatrix()
{
    delete[] data_;
}

PrMatrix::PrMatrix(PrMatrix const &m)
{
    row_ = m.row_;
    col_ = m.col_;
    n_ = m.n_;
//    assert(data_ == NULL);
    data_ = new Float[n_];
    for(int i=0;i<n_;i++)
      data_[i] = m.data_[i];
}

PrMatrix &PrMatrix::operator=(PrMatrix const &m)
{
    if (this != &m)
      {
	  AssertDimension(m);
	  for(int i=0;i<n_;i++)
	    data_[i] = m.data_[i];
      }
    return (*this);
}

Float *PrMatrix::operator[](int _row)
{
    assert(_row>=0 && _row<row_);
    return (data_+_row*col_);
}

Float const *PrMatrix::operator[](int _row) const
{
    assert(_row>=0 && _row<row_);
    return (data_+_row*col_);
}

Float &PrMatrix::elementAt(int i,int j)
{
    assert(i>=0 && i<row_ && j>=0 && j<col_);
    return data_[i*col_+j];
}

Float const &PrMatrix::elementAt(int i,int j) const
{
    assert(i>=0 && i<row_ && j>=0 && j<col_);
    return data_[i*col_+j];
}

Float &PrMatrix::at(int i,int j)
{
    assert(i>=0 && i<row_ && j>=0 && j<col_);
    return data_[i*col_+j];
}

Float const &PrMatrix::at(int i,int j) const
{
    assert(i>=0 && i<row_ && j>=0 && j<col_);
    return data_[i*col_+j];
}

void PrMatrix::zero()
{
    for(int i=0;i<n_;i++)
      data_[i]=0.0;
}

PrMatrix PrMatrix::operator-() const
{
    PrMatrix tmp(row_,col_);
    for(int i=0;i<n_;i++)
      tmp.data_[i] = -data_[i];
    return tmp;
}

void PrMatrix::negate(PrMatrix &dest) const
{
    AssertDimension(dest);
    for(int i=0;i<n_;i++)
      dest.data_[i] = -data_[i];
}

PrMatrix PrMatrix::operator+(PrMatrix const &m) const
{
    AssertDimension(m);
    PrMatrix tmp(row_,col_);
    for(int i=0;i<n_;i++)
      tmp.data_[i] = data_[i]+m.data_[i];
    return tmp;
}

void PrMatrix::add(PrMatrix const &m,PrMatrix &dest) const
{
    AssertDimension(m);
    AssertDimension(dest);
    for(int i=0;i<n_;i++)
      dest.data_[i] = data_[i]+m.data_[i];
}

PrMatrix PrMatrix::operator-(PrMatrix const &m) const
{
    AssertDimension(m);
    PrMatrix tmp(row_,col_);
    for(int i=0;i<n_;i++)
      tmp.data_[i] = data_[i]-m.data_[i];
    return tmp;
}

void PrMatrix::subtract(PrMatrix const &m,PrMatrix &dest) const
{
    AssertDimension(m);
    AssertDimension(dest);
    for(int i=0;i<n_;i++)
      dest.data_[i] = data_[i]-m.data_[i];
}

// (*this) * m
PrMatrix PrMatrix::operator*(PrMatrix const &m) const
{
    assert(col_ == m.row_);
    PrMatrix tmp(row_,m.col_);
    for(int i=0;i<tmp.row_;i++)
      for(int j=0;j<tmp.col_;j++)
	for(int k=0;k<col_;k++)
	    tmp[i][j] += (*this)[i][k]*m[k][j];

    return tmp;
}

void PrMatrix::multiply(PrMatrix const &m,PrMatrix &dest) const
{
    assert(col_ == m.row_);
    assert(row_ == dest.row_ && m.col_ == dest.col_);
    for(int i=0;i<dest.row_;i++)
      for(int j=0;j<dest.col_;j++)
	{
	    dest[i][j] = 0.0;
	    for(int k=0;k<col_;k++)
	      dest[i][j] += (*this)[i][k]*m[k][j];
	}
}

// (*this) * v
PrVector PrMatrix::operator*(PrVector const &v) const
{
    AssertDimension(v);
    PrVector tmp(row_);

    for(int i=0;i<row_;i++)
      for(int j=0;j<col_;j++)
	tmp[i] += (*this)[i][j]*v[j]; 

    return tmp;
}

void PrMatrix::multiply(PrVector const &v,PrVector &dest) const
{
    AssertDimension(v);
    assert(row_ == dest.size());

    for(int i=0;i<row_;i++)
      {
	  dest[i] = 0.0;
	  for(int j=0;j<col_;j++)
	    dest[i] += (*this)[i][j]*v[j]; 
      }
}

PrMatrix PrMatrix::operator*(Float s) const
{
    PrMatrix tmp(row_,col_);
    for(int i=0;i<n_;i++)
      tmp.data_[i] = data_[i]*s;
    return tmp;
}

void PrMatrix::multiply(Float s,PrMatrix &dest) const
{
    AssertDimension(dest);
    for(int i=0;i<n_;i++)
      dest.data_[i] = data_[i]*s;
}

PrMatrix &PrMatrix::operator+=(PrMatrix const &m)
{
    AssertDimension(m);
    for(int i=0;i<n_;i++)
      data_[i] += m.data_[i];
    return (*this);
}

PrMatrix &PrMatrix::operator-=(PrMatrix const &m)
{
    AssertDimension(m);
    for(int i=0;i<n_;i++)
      data_[i] -= m.data_[i];
    return (*this);
}

// (*this) * m
PrMatrix &PrMatrix::operator*=(PrMatrix const &m)
{
    (*this) = (*this)*m;
    return (*this);
}

PrMatrix &PrMatrix::operator*=(Float s)
{
    for(int i=0;i<n_;i++)
      data_[i] *= s;
    return (*this);
}

// [0 1 2;3 4 5;6 7 8]^T = [0 3 6;1 4 7;2 5 8]  
PrMatrix PrMatrix::transpose() const
{
    PrMatrix tmp(col_,row_);

    for(int i=0;i<row_;i++)
      for(int j=0;j<col_;j++)
	tmp[j][i] = (*this)[i][j];

    return tmp;
}

void PrMatrix::transpose(PrMatrix &dest) const
{
    assert(col_ == dest.row_ && row_ == dest.col_);

    for(int i=0;i<row_;i++)
      for(int j=0;j<col_;j++)
	dest[j][i] = (*this)[i][j];
}

void PrMatrix::diagonal(Float *data)
{
    assert(row_ == col_);
    for(int r = 0; r < row_; r++)
      (*this)[r][r] = data[r];
}

PrVector PrMatrix::diagonal() const
{
    assert(row_ == col_);
    PrVector tmp(row_);
    for(int r = 0; r < row_; r++)
      tmp[r] = (*this)[r][r];
    return tmp;
}

void PrMatrix::diagonal(PrVector &dest) const
{
    assert(row_ == col_);
    AssertDimension(dest);
    for(int r = 0; r < row_; r++)
      dest[r] = (*this)[r][r];
}

void PrMatrix::display(char *name)
{
    if (name != NULL)
      printf("%s =\n",name);

    for(int i=0;i<row_;i++)
      {
	  for(int j=0;j<col_;j++)
	    printf("%.6f ",(*this)[i][j]);
	  printf("\n");
      }
}

PrMatrix PrMatrix::operator~() const
{
  assert(row_ == col_);

  PrMatrix tmpM(row_,col_);
  inverse(tmpM);
  return tmpM;
}

// y = (*this) x
// solve for x given y
void PrMatrix::solve(PrVector const &y,PrVector &x) const
{
  assert(row_ == col_);
  AssertDimension(y);
  AssertDimension(x);

  PrMatrix lu(row_,col_);

  LUdecomp(lu);
  BackSub(lu,y,x);
}

// y = (*this) x
// solve for x given y
//(symmetric and positive definite)
void PrMatrix::solveSPD(PrVector const &y,PrVector &x) const
{
  assert(row_ == col_);
  AssertDimension(y);
  AssertDimension(x);

  PrMatrix lu(row_,col_);

  LUdecompSPD(lu);
  BackSubSPD(lu,y,x);
}

/* Matrix Inverse by Crout's LU decomposition */
/* p275-285, Numerical Methods for Engineers by Chapra and Canale */

/* output: ainv */
void PrMatrix::inverse(PrMatrix &ainv) const
{
  assert(row_ == col_);
  register int i,k,n=row_;
  PrMatrix lu(row_,col_);
  PrVector x(col_),y(col_);

  // LU ainv = I
  // a x = y
  // LU x = y

  LUdecomp(lu);

  for(k=0;k<n;k++)
    {
      y.zero();
      y.elementAt(k) = 1.0;

      BackSub(lu,y,x);
      for(i=0;i<n;i++)
	ainv.elementAt(i,k) = x.elementAt(i);
    }
}

/*
 * Private Function: LUdecomp
 */
// y = LU x
// find x given LU and y 
// diag(U) = [1 1 1 ... 1]
void PrMatrix::BackSub(PrMatrix const &lu,PrVector const &y,PrVector &x) const
{
  assert(row_ == col_);
  AssertDimension(lu);
  AssertDimension(y);
  AssertDimension(x);
  /* LU x = y ----> x */
  /* L d  = y --> fw sub */
  /* U x = d ---> bw sub */
  register int i,j,n=row_;
  Float sum;
  PrVector d(row_);

  d[0] = y.elementAt(0)/lu.elementAt(0,0);   /* L */
  for(i=1;i<n;i++)
    {
      sum = 0.0;
      for(j=0;j<i;j++)
	sum += lu.elementAt(i,j)*d[j]; /* L */
      d[i] = (y.elementAt(i)-sum)/lu.elementAt(i,i);
    }
  x.elementAt(n-1) = d[n-1];
  for(i=n-2;i>=0;i--)
    {
      sum = 0.0;
      for(j=i+1;j<n;j++)
	sum += lu.elementAt(i,j)*x.elementAt(j); /* U */
      x.elementAt(i) = d[i] - sum;
    }
}

/*
 * Private Function: LUdecomp
 */
/* output: lu */
void PrMatrix::LUdecomp(PrMatrix &lu) const
{
  assert(row_ == col_);
  Float sum;
  register int i,j,k,n=row_;
  PrMatrix const &a = *this;

  for(i=0;i<n;i++)
    lu.elementAt(i,0) = a.elementAt(i,0);  /* L */
  for(j=1;j<n;j++)
    lu.elementAt(0,j) = a.elementAt(0,j)/lu.elementAt(0,0);  /* U */
  for(j=1;j<n-1;j++)
    {
      for(i=j;i<n;i++)
	{
	  sum = 0.0;
	  for(k=0;k<j;k++)
	    sum += lu.elementAt(i,k)*lu.elementAt(k,j);
	  lu.elementAt(i,j) = a.elementAt(i,j)-sum;  /* L */
	}
      for(k=j+1;k<n;k++)
	{
	  sum = 0.0;
	  for(i=0;i<j;i++)
	    sum += lu.elementAt(j,i)*lu.elementAt(i,k);
	  lu.elementAt(j,k) = (a.elementAt(j,k)-sum)/lu.elementAt(j,j); /* U */
	}
    }
  sum = 0.0;
  for(k=0;k<n-1;k++)
    sum += lu.elementAt(n-1,k)*lu.elementAt(k,n-1);
  lu.elementAt(n-1,n-1) = a.elementAt(n-1,n-1)-sum;       /* L */
}

/* Matrix Inverse by Cholesky's LU decomposition */
/* p288-290, Numerical Methods for Engineers by Chapra and Canale */

/* output: ainv */
//(symmetric and positive definite)
void PrMatrix::inverseSPD(PrMatrix &ainv) const
{
  assert(row_ == col_);
  AssertDimension(ainv);
  register int i,k,n=row_;
  PrMatrix lu(row_,col_);
  PrVector x(col_),y(col_);

  // LU ainv = I
  // a x = y
  // LU x = y

  LUdecompSPD(lu);

  for(k=0;k<n;k++)
    {
      y.zero();
      y.elementAt(k) = 1.0;

      BackSubSPD(lu,y,x);
      for(i=0;i<n;i++)
	ainv.elementAt(i,k) = x.elementAt(i);
    }
}

// y = LU x
// y = LL' x
// find x given LU and y 
//(symmetric and positive definite)
void PrMatrix::BackSubSPD(PrMatrix const &lu,PrVector const &y,PrVector &x) const
{
  assert(row_ == col_);
  AssertDimension(lu);
  AssertDimension(y);
  AssertDimension(x);
  /* U = L' */
  /* LU = LL' */
  /* LU x = y ----> x */
  /* L d  = y --> fw sub */
  /* U x = d ---> bw sub */
  register int i,j,n=row_;
  Float sum;
  PrVector d(row_);

  d[0] = y.elementAt(0)/lu.elementAt(0,0);        /* fw: L */
  for(i=1;i<n;i++)
    {
      sum = 0.0;
      for(j=0;j<i;j++)
	sum += lu.elementAt(i,j)*d[j];
      d[i] = (y.elementAt(i) - sum)/lu.elementAt(i,i);
    }
  x.elementAt(n-1) = d[n-1]/lu.elementAt(n-1,n-1);        /* bw: U */
  for(i=n-2;i>=0;i--)
    {
      sum = 0.0;
      for(j=i+1;j<n;j++)
	sum += lu.elementAt(i,j)*x.elementAt(j);
      x.elementAt(i) = (d[i] - sum)/lu.elementAt(i,i);
    }
}

//(symmetric and positive definite)
void PrMatrix::LUdecompSPD(PrMatrix &lu) const
{
  assert(row_ == col_);
  AssertDimension(lu);
  Float sum;
  register int i,j,k,n=row_;
  PrMatrix const &a = *this;

  /* U = L' */
  for (k=0;k<n;k++)
    {
      for (i=0;i<k;i++)
	{
	  sum = 0.0;
	  for (j=0;j<i;j++)
	    sum += lu.elementAt(i,j) * lu.elementAt(k,j);
	  lu.elementAt(i,k) = lu.elementAt(k,i) 
	    = (a.elementAt(k,i)-sum)/lu.elementAt(i,i);
	}
      sum = 0.0;
      for (j=0;j<k;j++)
	sum += lu.elementAt(k,j)*lu.elementAt(k,j);
      lu.elementAt(k,k) = sqrt(a.elementAt(k,k)-sum);
    }
}

// private methods

void PrMatrix::AssertDimension(PrMatrix const &m) const
{
    assert(row_ == m.row_ && col_ == m.col_);
}

void PrMatrix::AssertDimension(PrVector const &v) const
{
    assert(col_ == v.size());
}
