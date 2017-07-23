/***************************************************************
 * PrVector.h
 *
 * This class provides a sizex1 vector.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 11/12/97: K.C. Chang: created.
 */

#ifndef _PrVector_h_
#define _PrVector_h_

#include "../PrGlobalDefn.h"

class PrMatrix;

class PrVector
{
  public:
    PrVector(int _size);  // initialized to [0]
    PrVector(PrVector const &v);
    ~PrVector();

    PrVector &operator=(const PrVector &v);
    Float &operator[](int i);
    Float const &operator[](int i) const;

    Float &elementAt(int i);
    Float &at(int i);

    Float const &elementAt(int i) const;
    Float const &at(int i) const;

    void zero();

    int size() const { return size_; };

    PrVector operator-() const;
    void negate(PrVector &dest) const;

    PrVector operator+(PrVector const &v) const;
    void add(PrVector const &v,PrVector &dest) const;
    PrVector operator-(PrVector const &v) const;
    void subtract(PrVector const &v,PrVector &dest) const;
    PrVector operator*(Float s) const;
    void multiply(Float s,PrVector &dest) const;
    PrVector operator*(PrVector const &v) const;
    void multiply(PrVector const &v,PrVector &dest) const;

    PrVector &operator+=(PrVector const &v);
    PrVector &operator-=(PrVector const &v);
    PrVector &operator*=(Float s);

    PrMatrix multiplyTransposed(PrVector const &v) const;
    void multiplyTransposed(PrVector const &v,PrMatrix &dest) const;

    Float dot(PrVector const &v) const;

    Float magnitude() const;

    void normalize();

    void display(char *name=NULL);

    // USE TO GATHER DATA
    Float * const data() const { return data_; }

  private:
    int size_;
    Float *data_;

    void AssertDimension(PrVector const& v) const;
};

#endif // _PrVector_h_
