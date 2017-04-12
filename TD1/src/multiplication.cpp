#include <iostream>

#include <iostream>

int **lireMatrice(int &n, int &m)
{
  std::cin >> n;
  std::cin >> m;
  int **matrice = 0;
  matrice = new int *[n];

  // A FAIRE :
  // 1. creation de la matrice en memoire et lecture de la matrice
  // 2. affichage de la matrice transposee

  for (int i = 0; i < n; i++)
  {
    matrice[i] = new int[m];
  }

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      std::cin >> matrice[i][j];
    }
  }
  return matrice;
}

int main(int argc, char **argv)
{
  // lecture des deux matrices A et B
  int nA, mA;
  int **A = lireMatrice(nA, mA);

  int nB, mB;
  int **B = lireMatrice(nB, mB);

  int **C = 0;
  C = new int *[nA];
  for (unsigned i = 0; i < nA; i++)
    C[i] = new int[mB];

  // affichage de A x B

 for(unsigned i = 0; i < nA; ++i)
        for(unsigned j = 0; j < mB; ++j)
            for(unsigned k = 0; k < mA; ++k)
            {
                C[i][j] += A[i][k] * B[k][j];
            }

  for (unsigned i = 0; i < nA; i++)
  {
    for (unsigned j = 0; j < mB-1; j++)
      std::cout << C[i][j] << " ";
      std::cout << C[i][mB -1];
    std::cout << std::endl;
  }

  // a completer

  return 0;
}