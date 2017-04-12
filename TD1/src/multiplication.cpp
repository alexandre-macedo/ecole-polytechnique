#include <iostream>

#include <iostream>
 
int** lireMatrice(int& n, int& m)
{
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
 
int main(int argc, char** argv)
{
  // lecture des deux matrices A et B
 
  int nA, mA;
  int** A = lireMatrice(nA, mA);
 
  int nB, mB;
  int** B = lireMatrice(nB, mB);
 
  // affichage de A x B
 
  // a completer
 
  return 0;
}
int main(int argc, char **argv)
{
  // lecture de la taille de la matrice

  int n = 0;
  int m = 0;

  std::cin >> n;
  std::cin >> m;

  // lecture de la matrice

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

  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < n-1; j++)
    {
      std::cout << matrice[j][i] << " ";
    }
      std::cout << matrice[n-1][i];
    std::cout << std::endl;
  }
  return 0;
}
