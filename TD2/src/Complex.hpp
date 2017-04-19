#include <iostream>
class Complex
{

  public:
  // Constructor
    Complex();
    Complex(double a, double b);

// Methods
	double module();

// Destuctour
    ~Complex();

// Member
    double a;
    double b;
};
std::ostream &operator<<(std::ostream &s, const Complex &c);
