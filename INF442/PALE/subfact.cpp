#include <iostream>
#include <cstring>
#include <cstdlib>
//#include <cmath>

int sous_factoriel(int n)
{
	if (n <= 0)
	{
		return 1;
	};
	
	if (n % 2 == 0)
	{
		return n * sous_factoriel(n - 1) + 1;
	}
	else
	{
		return n * sous_factoriel(n - 1) - 1;
	};
}

int main(int argc, char *argv[])
{

	int n;

	if (argc < 2)
	{
		std::cout << "Syntax : ./subfact n" << std::endl;
		return -1;
	}

	n = atoi(argv[1]);

	std::cout << "n=" << n << std::endl;
	//std::cout << n%2 << std::endl;

	int result = sous_factoriel(n);
	std::cout << "sous-factoriel " << result << std::endl;

	return 0;
}
