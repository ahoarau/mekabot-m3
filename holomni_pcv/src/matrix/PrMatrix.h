/***************************************************************
 * PrMatrix.h
 *
 * This class provides a row x col matrix.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 11/12/97: K.C. Chang: created.
 */

#ifndef _PrMatrix_h_
#define _PrMatrix_h_

#include "../PrGlobalDefn.h"
#include "PrVector.h"

class PrMatrix
{
  public:
    PrMatrix(int _row,int _col); // initialized to [0]
    PrMatrix(PrMatrix const &m);
    ~PrMatrix();

    PrMatrix &operator=(PrMatrix const &m);
    Float *operator[](int _row);
    Float const *operator[](int _row) const;

    Float &elementAt(int i,int j);
    Float &at(int i,int j);
    Float const &elementAt(int i,int j) const;
    Float const &at(int i,int j) const;

    int row() const { return row_; }
    int column() const { return col_; }

    void zero();

    PrMatrix operator-() const;
    void negate(PrMatrix &dest) const;

    PrMatrix operator~() const;
    void inverse(PrMatrix &dest) const;
    //(symmetric and positive definite)
    void inverseSPD(PrMatrix &dest) const;

    // y = (*this) x
    // solve for x given y
    void solve(PrVector const &y,PrVector &x) const;
    //(symmetric and positive definite)
    void solveSPD(PrVector const &y,PrVector &x) const;

    PrMatrix operator+(PrMatrix const &m) const;
    void add(PrMatrix const &m,PrMatrix &dest) const;
    PrMatrix operator-(PrMatrix const &m) const;
    void subtract(PrMatrix const &m,PrMatrix &dest) const;
    PrMatrix operator*(PrMatrix const &m) const;
    void multiply(PrMatrix const &m,PrMatrix &dest) const;
    PrVector operator*(PrVector const &v) const;
    void multiply(PrVector const &v,PrVector &dest) const;
    PrMatrix operator*(Float s) const;
    void multiply(Float s,PrMatrix &dest) const;

    PrMatrix &operator+=(PrMatrix const &m);
    PrMatrix &operator-=(PrMatrix const &m);
    PrMatrix &operator*=(PrMatrix const &m);
    PrMatrix &operator*=(Float s);

    PrMatrix transpose() const;
    void transpose(PrMatrix &dest) const;

    void diagonal(Float *data); // set the diagonal
    PrVector diagonal() const;
    void diagonal(PrVector &dest) const;

    void display(char *name=NULL);

    // USE TO GATHER DATA
    Float *const data() const { return data_; }

  private:
    int row_;
    int col_;
    int n_;   // =row_*col_
    Float *data_;

    void LUdecomp(PrMatrix &lu) const;
    //(symmetric and positive definite)
    void LUdecompSPD(PrMatrix &lu) const;

    // y = LU x
    // find x given LU and y 
    void BackSub(PrMatrix const &lu,PrVector const &y,PrVector &x) const;
    //(symmetric and positive definite)
    void BackSubSPD(PrMatrix const &lu,PrVector const &y,PrVector &x) const;

    void AssertDimension(PrMatrix const &m) const;
    void AssertDimension(PrVector const &v) const;
};

#endif // _PrMatrix_h_
