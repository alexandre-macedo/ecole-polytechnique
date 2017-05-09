/*
  Filename: partition.cpp
  Author: Leo Liberti
  Purpose: implementation of the Partition class methods for the JLL TD
  Source: GNU C++
  History:
    150418 work started
*/

//// globals, defined in JL_kmeans.cpp
// infinity, tolerance
extern double infinity;
extern double zero;
extern bool debug;
extern int multiStartForgy;

// STL includes
#include <iostream>
#include <vector>
#include <algorithm>

#include "randomsample.hpp"
#include "mathtools.hpp"
#include "partition.hpp"

//// constructor
// notice that point, which is a class reference, is initialized even
//   before the constructor actually starts, by a special C++ mechanism
//   designed to cover exactly the case of class references: a single
//   colon between the closing declaration bracket and the opening scope
//   brace introduce a comma-separated list of the form
//     classAttribute1(init_arg1), ..., classAttributeN(init_arg1)
//   where init_args must already be defined, and have the same type
//   as the objects or references classAttributes. In this case
//   the list consists of just one reference (point), so there are no commas
Partition::Partition(std::vector<std::vector<double> >& data, int numClusters)
    : point(data) {
  using namespace std;
  n = data.size();
  d = data[0].size();
  k = numClusters;
  for (int h = 0; h < k; ++h) {
    // by default, we assign to each key an empty vector
    vector<int> emptyCluster;
    cluster.push_back(emptyCluster);
  }
}

//// copy constructor
Partition::Partition(Partition& P) : point(P.getData()) {
  d = P.getDimension();
  n = P.getNumberOfPoints();
  k = P.getNumberOfClusters();
  centroid = P.getCentroids();
  cluster = P.getClusters();
}

//// assignment overloading
Partition& Partition::operator=(Partition& P) {
  d = P.getDimension();
  n = P.getNumberOfPoints();
  k = P.getNumberOfClusters();
  centroid = P.getCentroids();
  cluster = P.getClusters();
  return *this;
}

//// print clusters
void Partition::printClusters(void) {
  using namespace std;
  // not using iterators for the outer loop since k is much smaller than n,
  //   so the consequent time loss is not important. Also, we're
  //   just printing to console, who cares if we're slightly suboptimal?
  //   The console is going to be MUCH slower than this time loss anyhow
  for (int h = 0; h < k; ++h) {
    cout << "  cluster " << h << ":";
    for (vector<int>::iterator vi = cluster[h].begin(); vi != cluster[h].end();
         ++vi) {
      cout << " " << *vi;
    }
    cout << endl;
  }
}

//// Forgy's initialization: use k random points as centroids
void Partition::Forgy(void) {
  using namespace std;
  centroid.clear();
  // this vector holds integers from 0 to n-1 (n is P.n, number of points)
  //   we randomly pick them and remove them from the vector as we go along
  //   this yields a random subsequence of k out of n integers in O(k)
  //   WARNING: if you don't remove them you may get repetitions
  //   WARNING2: if you reject repetitions, you end up with O(n^2) if k=n
  vector<int> pointIndexChoice;
  // initialize this vector to 0,...,n-1
  for (int i = 0; i < n; i++) {
    pointIndexChoice.push_back(i);
  }
  // we keep score of the vector size as we remove elements
  int m = n;
  // we only randomly pick k (=P.k, number of clusters) out of n elements
  for (int h = 0; h < k; ++h) {
    // pick a random index from the m remaining ones
    int i = randomInteger(0, m);
    // map i to the point index using pointIndexChoice,
    //   and select the corresponding centroid
    centroid.push_back(point[pointIndexChoice[i]]);
    // find the iterator of the pointIndexChoice element corresponding to i
    vector<int>::iterator vi = find(
        pointIndexChoice.begin(), pointIndexChoice.end(), pointIndexChoice[i]);
    // remove it from the pointIndexChoice vector and decrease m
    pointIndexChoice.erase(vi);
    m--;
  }
}

//// run Forgy s times, keep best
void Partition::initialize(int s) {
  // we want to run Forgy s times to find the best random assignment
  //   so we initialize the current cost to infinity (meaning: at
  //   this stage we haven't run it yet, so anything's better than
  //   the current situation)
  double cost = infinity;
  double mincost = cost;
  Partition minP(*this);
  for (int i = 0; i < s; ++i) {
    Forgy();
    reassign();
    cost = interClusterDistances();
    // if the cost of this run improves on the current minimum so far,
    //   save the cost and the partition
    if (cost < mincost) {
      mincost = cost;
      minP = *this;
    }
  }
  // re-instate the minimum partition
  *this = minP;
}

//// re-assign points to closest centroids
void Partition::reassign(void) {
  using namespace std;
  // clear the cluster data structures
  for (vector<vector<int> >::iterator mi = cluster.begin(); mi != cluster.end();
       ++mi) {
    //   since mi is a vector<int>, we can apply the clear() method
    //   to remove all its elements
    mi->clear();
  }
  // In the following, we use pointers to loop over STL containers,
  //   but we also need the numeric index corresponding to the iterators,
  //   so we also use integer indices h,i to loop over clusters and points
  int h = 0;
  int i = 0;
  // iterate over points
  for (vector<vector<double> >::iterator p = point.begin(); p != point.end();
       ++p) {
    // we have to find the centroid with minimum distance from this point,
    //   so we initialize this minimum distance to infinity
    double minDist = infinity;
    int closest = -1;
    // ...and loop over centroids to find the closest
    h = 0;
    for (vector<vector<double> >::iterator c = centroid.begin();
         c != centroid.end(); ++c) {
      double dist = sqEuclideanDistance(*p, *c);
      if (dist < minDist) {
        minDist = dist;
        closest = h;
      }
      h++;
    }
    cluster[closest].push_back(i);
    i++;
  }
}

//// update the centroids based on the new cluster assignment
void Partition::updateCentroids(void) {
  for (int i = 0; i < cluster.size(); ++i) {
    for (int j = 0; j < d; ++j) {  // for each dimension
      // Compute the average over all points in the cluster to give the new
      // values of centroids
      double component = 0.0;
      for (int c = 0; c < cluster[i].size(); c++) {
        component += point[cluster[i][c]][j];
      }
      centroid[i][j] = component / cluster[i].size();
    }
  }
}

//// evaluate cost of current partition
double Partition::interClusterDistances(void) {
  double cost = 0.0;
  for (int i = 0; i < cluster.size(); i++) {  // for each cluster
    for (int c = 0; c < cluster[i].size();
         c++) {                      // for each point in the cluster
      for (int j = 0; j < d; ++j) {  // Compute the cost
        cost += pow(point[cluster[i][c]][j] - centroid[i][j], 2);
      }
    }
  }
  return cost;
}

double Partition::iteration(void) {
  using namespace std;
  // we want to find the best clustering with an iterative method, so we
  //   start from an infinity cost, since clustering will be better than
  //   no clustering at all
  double cost = infinity;
  double minCost = 0;
  initialize(multiStartForgy);
  while (fabs(cost - minCost) > zero) {
    // Use the functions above to find the convergence of clusters
    reassign();
    updateCentroids();
    // we don't need to say "if cost < minCost" since this Lloyd's algorithm
    //   is contractive, i.e. no iteration can worsen previous ones
    minCost = cost;
    cost = interClusterDistances();
    if (debug) {
      cerr << "Partition::Lloyd(): current cost = " << cost << endl;
    }
  }
  return cost;
}
