#include "Complex.hpp"
#include <cmath>

Complex::Complex()
{

    a = 0.0;
    b = 0.0;
}

Complex::Complex(double a, double b)
{
    this->a = a;
    this->b = b;
}

double Complex::module()
{
    return sqrt(pow(this->a, 2) + pow(this->b, 2));
}

Complex::~Complex()
{
}

std::ostream &operator<<(std::ostream &s, const Complex &c)
{
    if (c.b == 0)
	s << c.a;
    else if (c.b > 0)
	s << c.a << "+" << c.b << "*i";
	else
	s << c.a << c.b << "*i";

    return s;
}
