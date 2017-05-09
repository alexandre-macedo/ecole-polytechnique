/*
  Filename: mathtools.cpp
  Author: Leo Liberti
  Purpose: mathematical functions for the JLL TD
  Source: GNU C++
  History:
    150419 work started
*/

#include <iostream>
#include <vector>
// C library includes wrapped by STL
#include <cstdlib>
#include <cmath>
#include <cassert>

#include "randomsample.hpp"
#include "mathtools.hpp"

extern bool debug;

// compute the squared Euclidean distance between two vector<double>
double sqEuclideanDistance(std::vector<double>& x, std::vector<double>& y) {
  using namespace std;
  assert(x.size() == y.size());
  double sqDist = 0;
  // WARNING: a for loop with index access (x[i]-y[i])
  //   would yield O(n log n), since each vector [] access is O(log n)
  //   The iterator-baed loop below is O(n)
  vector<double>::iterator xi(x.begin());
  vector<double>::iterator yi(y.begin());
  while (xi != x.end()) {
    // don't get confused between the unary dereference operator * and
    //   the binary arithmetic product operator with the same symbol.
    //   C++ can tell them apart - if the computer can, so should you!
    sqDist += (*xi - *yi) * (*xi - *yi);
    xi++;
    yi++;
  }
  return sqDist;
}

// compute the squared Euclidean distance matrix of a set of vectors
std::vector<std::vector<double> > sqEDM(std::vector<std::vector<double> >& S) {
  using namespace std;
  // size of S (set of n column vectors of size d
  int n = S.size();
  int d = S[0].size();
  // we use both iterators and indices
  int i = 0;
  int j = 0;
  // this holds the output squared EDM
  vector<vector<double> > EDM(n);
  // initialize EDM to zero
  for (vector<vector<double> >::iterator ei = EDM.begin(); ei != EDM.end();
       ++ei) {
    for (j = 0; j < n; j++) {
      ei->push_back(0.0);
    }
  }
  // loop over the vectors twice
  for (vector<vector<double> >::iterator vi = S.begin(); vi != S.end(); ++vi) {
    j = 0;
    for (vector<vector<double> >::iterator vj = S.begin(); vj != S.end();
         ++vj) {
      if (i < j) {
        EDM[i][j] = sqEuclideanDistance(*vi, *vj);
      } else {
        EDM[i][j] = EDM[j][i];
      }
      j++;
    }
    i++;
  }
  return EDM;
}

// sample (transposed) Gaussian p x d matrix
//   sequential computation
std::vector<std::vector<double> > JLProjectionMatrix(int d, int projd) {
  using namespace std;
  vector<vector<double> > JL;
  // we are going to store the matrix by rows (since the data matrix
  //   is stored by columns, this is like storing the transpose)
  double sqrtp = sqrt(projd);

  if (debug) {
    cerr << "JLProjectionMatrix(): sequential";
  }

#ifndef TRIVIAL
  // in this version we exploit the fact that Box-Muller outputs two
  //   Gaussian samples, and use them both
  if (debug) {
    cerr << " (nontrivial)" << endl;
  }
  for (int h = 0; h < projd; ++h) {
    vector<double> row;
    // we are going to leverage the fact that the Box-Muller's algorithm
    //   returns two samples, not just one, and use them both
    // d2 contains floor(d/2)
    int d2 = d / 2;
    if (d % 2 == 1) {
      d2 = (d - 1) / 2;
    }
    // storage for the two Gaussian samples
    double g1, g2;
    for (int f = 0; f < d2; ++f) {
      GaussianRandomPair(g1, g2);
      // since this is for JL, multiply by 1/sqrt(p)
      row.push_back(g1 / sqrtp);
      row.push_back(g2 / sqrtp);
    }
    if (d % 2 == 1) {
      // if d is odd, then we must add one last sample for row to be a d-vector
      GaussianRandomPair(g1, g2);
      row.push_back(g1 / sqrtp);
    }
    JL.push_back(row);
  }

#else
  // this is the "trivial" version
  if (debug) {
    cerr << " (trivial)" << endl;
  }
  vector<double> row;
  for (int i = 0; i < projd; i++) {
    row.clear();
    for (int f = 0; f < d; ++f) {
      row.push_back(GaussianRandom() / sqrtp);
    }
    JL.push_back(row);
  }
#endif  // !TRIVIAL

  if (debug) {
    cerr << "JLProjectionMatrix(): JL is " << JL.size() << " x " << JL[0].size()
         << endl;
  }

  // about returning a large object on the stack: all modern C++ compilers
  //   will NOT copy them, but rather just use the original storage. See
  //   http://stackoverflow.com/questions/753312/
  return JL;
}

// matrix product
//   sequential implementation
std::vector<std::vector<double> > MatrixProduct(
    std::vector<std::vector<double> >& JL,
    std::vector<std::vector<double> >& X) {
  using namespace std;
  if (debug) {
    cerr << "MatrixProduct(): sequential" << endl;
  }
  // this will hold the product of JL and X
  vector<vector<double> > projX;
  // matrix dimensions
  int projd = JL.size();
  int n = X.size();
  int d = X[0].size();
  // as matrices, JL is (projd x d) and X is (d x n)
  //   as a vector of vectors, JL holds projd vectors of length d
  //   as a vector of vectors, X holds n vectors of length d

  for (vector<vector<double> >::iterator Xit = X.begin(); Xit != X.end();
       ++Xit) {
    vector<double> projCol;
    for (vector<vector<double> >::iterator JLit = JL.begin(); JLit != JL.end();
         ++JLit) {
      double a = 0;
      for (int j = 0; j < d; j++) {
        a += (*JLit)[j] * (*Xit)[j];
      }
      projCol.push_back(a);
    }
    projX.push_back(projCol);
  }

  return projX;
}

// print a vector<vector<double> >
void printData(std::vector<std::vector<double> >& data) {
  using namespace std;
  // loop over rows
  // WARNING: when using iterators, NEVER EVER write
  //   iterator_name < container_name.end(), always use "!=" instead of "<"
  //   (an iterator is like a pointer, elements of the container
  //    could be stored beyond the end() address)
  for (vector<vector<double> >::iterator vvd = data.begin(); vvd != data.end();
       ++vvd) {
    // loop over columns
    for (vector<double>::iterator vd = vvd->begin(); vd != vvd->end(); ++vd) {
      // separate fields using spaces
      cout << " " << *vd;
    }
    cout << endl;
  }
}
