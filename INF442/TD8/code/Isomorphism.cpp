#include "Isomorphism.hpp"
#include "mpi.h"
#include <iostream>
#include <map>
#include <stdexcept>

using namespace std;

// Exercice 3
void extendInjection(Mapping &m) {
  // The argument m is (a reference to) a partially-defined injective
  // function G -> H : it may only have images defined for a subset of G.
  // This procedure recursively attempts to extend m to the whole of G,
  // using a backtracking approach.

  if (m.isFull()) {
    return;
  }
  for (int nidg = 0; nidg < m.getSizeG(); nidg++) {
    for (int nidh = 0; nidh < m.getSizeH(); nidh++) {
      if (m.areMappable(nidg, nidh)) {
        m.addToMap(nidg, nidh);
        extendInjection(m);
        if (m.isFull()) {
          return;
        } else
          m.deleteFromMap(nidg);
      }
    }
  }
  return;
}

void findIsomorphism(Graph G, Graph H) {
  if (G.vertexCount() != H.vertexCount()) {
    std::cout << "Pas d'isomorphisme" << std::endl;
    return;
  }
  Mapping candidate(G, H);
  extendInjection(candidate);
  if (candidate.isFull())
    std::cout << candidate << std::endl;
  else
    std::cout << "Pas d'isomorphisme" << std::endl;
}

// Exercice 4
void findSubIsomorphism(Graph G, Graph H) {
  if (G.vertexCount() >= H.vertexCount()) {
    // ...si aucun sous-isomorphisme trouve,
    std::cout << "Pas de sous-isomorphisme" << std::endl;
    return;
  }
  Mapping candidate(G, H);
  extendInjection(candidate);
  if (candidate.isFull())
    std::cout << candidate << std::endl;
  else
    // ...si aucun sous-isomorphisme trouve,
    std::cout << "Pas de sous-isomorphisme" << std::endl;
}

// Exercice 5
void findSubIsomorphismMPI(Graph G, Graph H) {
  int myrank, size;
  int n = H.vertexCount();

  MPI_Comm_rank(MPI_COMM_WORLD, &myrank);
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  MPI_Status status;

  int next[1];

  if (myrank == 0) {
    // Root
    if (G.vertexCount() > H.vertexCount()) {
      std::cout << "Pas de sous-isomorphisme" << std::endl;
      return;
    }

    int flag[size];
    for (unsigned int i = 0; i < size; ++i) {
      flag[i] = 1;
    }

    next[0] = 0;

    for (unsigned int i = 1; i < size; ++i) {
      if (next[0] >= n)
        break;
      MPI_Isend(next, 1, MPI_INT, i, i, MPI_COMM_WORLD, &request[i]);
      next[0]++;
    }

    while (next[0] < n) {

      for (unsigned int i = 1; i < size; ++i) {
        MPI_Test(&request[i], &flag[i], &status);
        if (flag[i] != 0) {
          MPI_Isend(next, 1, MPI_INT, i, i, MPI_COMM_WORLD, &request[i]);
          next[0]++;
        }
      }
    }
  } else {
    // Not root
    while (true) {
      Mapping candidate(G, H);

      MPI_Recv(next, 1, MPI_INT, 0, myrank, MPI_COMM_WORLD, &status);

      if (candidate.areMappable(0, next[0])) {
        candidate.addToMap(0, next[0]);
        extendInjection(candidate);
        if (candidate.isFull()) {
          std::cout << candidate << std::endl;
        }
      }
    }
  }
  return;
}
