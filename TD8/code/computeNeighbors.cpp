#include <iostream>
#include <vector>

// messages are received from node with highest 1-bit flipped to 0
int computePredecessor(int node) {
  int r;
  int i = 1;

  while (i <= node)
    i *= 2;
  i /= 2;

  r = node - i;
  return r;
}

// compute neighbors to communicate to
std::vector<int> computeOutNeighbors(int node, int numberNodes) {
  std::vector<int> neighbors;
  int flipbit;
  if (node == 0)
    flipbit = 1;
  else
    flipbit = (node - computePredecessor(node)) * 2;

  for (unsigned int i = flipbit; i + node < numberNodes; i *= 2) {
    int k = node + i;
    neighbors.push_back(k);
  }

  return neighbors;
}

// print neighbor list for debugging
void printNeighbors(std::vector<int> neighbors) {
  for (std::vector<int>::iterator iter = neighbors.begin();
       iter != neighbors.end(); iter++) {
    std::cout << *iter << " ";
  }
  std::cout << std::endl;
}
