/***************************************************************
 * PrVector.cpp
 *
 * This implements a sizex1 vector class.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 11/12/97: K.C. Chang: created.
 */

#include "PrVector.h"
#include "PrMatrix.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>


PrVector::PrVector(int _size):size_(_size)
{
    assert(size_>0);
    data_ = new Float[size_];
    zero();
}

PrVector::PrVector(PrVector const &v)
{
    size_ = v.size_;
//    assert(data_ == NULL);
    data_ = new Float[size_];
    for(int i=0;i<size_;i++)
      data_[i] = v.data_[i];
}

PrVector::~PrVector()
{
    delete[] data_;
}

PrVector &PrVector::operator=(PrVector const &v)
{
    if (this != &v)
      {
	  AssertDimension(v);
	  for(int i=0;i<size_;i++)
	    data_[i] = v.data_[i];
      }
    return (*this);
}

Float &PrVector::operator[](int i)
{
    assert(i>=0 && i<size_);
    return data_[i];
}

Float const &PrVector::operator[](int i) const
{
    assert(i>=0 && i<size_);
    return data_[i];
}

Float &PrVector::elementAt(int i)
{
    assert(i>=0 && i<size_);
    return data_[i];
}

Float const &PrVector::elementAt(int i) const
{
    assert(i>=0 && i<size_);
    return data_[i];
}

Float &PrVector::at(int i)
{
    assert(i>=0 && i<size_);
    return data_[i];
}

Float const &PrVector::at(int i) const
{
    assert(i>=0 && i<size_);
    return data_[i];
}

void PrVector::zero()
{
    for(int i=0;i<size_;i++)
      data_[i] = 0.0;
}

PrVector PrVector::operator-() const
{
    PrVector tmp(size_);
    for(int i=0;i<size_;i++)
      tmp.data_[i] = -data_[i];
    return tmp;
}

void PrVector::negate(PrVector &dest) const
{
    AssertDimension(dest);
    for(int i=0;i<size_;i++)
      dest.data_[i] = -data_[i];
}

PrVector PrVector::operator+(PrVector const &v) const
{
    AssertDimension(v);
    PrVector tmp(size_);
    for(int i=0;i<size_;i++)
      tmp.data_[i] = data_[i] + v.data_[i];
    return tmp;
}

void PrVector::add(PrVector const &v,PrVector &dest) const
{
    AssertDimension(v);
    AssertDimension(dest);
    for(int i=0;i<size_;i++)
      dest.data_[i] = data_[i] + v.data_[i];
}

PrVector PrVector::operator-(PrVector const &v) const
{
    AssertDimension(v);
    PrVector tmp(size_);
    for(int i=0;i<size_;i++)
      tmp.data_[i] = data_[i] - v.data_[i];
    return tmp;
}

void PrVector::subtract(PrVector const &v,PrVector &dest) const
{
    AssertDimension(v);
    AssertDimension(dest);
    for(int i=0;i<size_;i++)
      dest.data_[i] = data_[i] - v.data_[i];
}

PrVector PrVector::operator*(Float s) const
{
    PrVector tmp(size_);
    for(int i=0;i<size_;i++)
      tmp.data_[i] = data_[i] * s;
    return tmp;
}

void PrVector::multiply(Float s,PrVector &dest) const
{
    AssertDimension(dest);
    for(int i=0;i<size_;i++)
      dest.data_[i] = data_[i]*s;
}

PrVector PrVector::operator*(PrVector const &v) const
{
    AssertDimension(v);
    PrVector tmp(size_);
    for(int i=0;i<size_;i++)
      tmp.data_[i] = data_[i] * v.data_[i];
    return tmp;
}

void PrVector::multiply(PrVector const &v,PrVector &dest) const
{
    AssertDimension(v);
    AssertDimension(dest);
    for(int i=0;i<size_;i++)
      dest.data_[i] = data_[i] * v.data_[i];
}

PrVector &PrVector::operator+=(PrVector const &v)
{
    AssertDimension(v);
    for(int i=0;i<size_;i++)
      data_[i] += v.data_[i];
    return (*this);
}

PrVector &PrVector::operator-=(PrVector const &v)
{
    AssertDimension(v);
    for(int i=0;i<size_;i++)
      data_[i] -= v.data_[i];
    return (*this);
}

PrVector &PrVector::operator*=(Float s)
{
    for(int i=0;i<size_;i++)
      data_[i] *= s;
    return (*this);
}

// return mat = (*this) * v^T
PrMatrix PrVector::multiplyTransposed(PrVector const &v) const
{
    PrMatrix tmp(size_,v.size_);

    for(int i=0;i<size_;i++)
      for(int j=0;j<v.size_;j++)
	tmp[i][j] = data_[i]*v.data_[j];
    return tmp;
}

void PrVector::multiplyTransposed(PrVector const &v,PrMatrix &dest) const
{
    assert(size_ == dest.row() && v.size_ == dest.column());

    for(int i=0;i<size_;i++)
      for(int j=0;j<v.size_;j++)
	dest[i][j] = data_[i]*v.data_[j];
}

Float PrVector::dot(PrVector const &v) const
{
    AssertDimension(v);
    Float num=0.0;
    for(int i=0;i<size_;i++)
      num += data_[i]*v.data_[i];
    return num;
}

Float PrVector::magnitude() const
{
    return sqrt(dot(*this));
}

void PrVector::normalize()
{
    (*this) *= 1.0/magnitude();
}

void PrVector::display(char *name)
{
    if (name != NULL)
      printf("%s =\n",name);

    printf("(");
    for(int i=0;i<size_;i++)
      {
	  printf("%.6f ",data_[i]);
      }
    printf(")\n");
}

// private methods
void PrVector::AssertDimension(PrVector const& v) const
{
    assert(size_ == v.size_);
}
