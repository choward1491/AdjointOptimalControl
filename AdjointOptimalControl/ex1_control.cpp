#include "ex1_control.hpp"

ex1::control::control():coefs(num_coefs(),0.0)
{
}

void ex1::control::init()
{
	std::fill(coefs.begin(), coefs.end(), 0.0);
}

void ex1::control::getControl(const double & t, const vec & x, vec & u)
{
	u[0] = 0.0;
	for (int i = 0; i < num_coefs(); ++i) {
		u[0] += coefs[i] * basis(t, i);
	}

}

ex1::vec & ex1::control::getCoefs()
{
	return coefs;
}

const ex1::vec & ex1::control::getCoefs() const
{
	return coefs;
}

int ex1::control::num_coefs() const
{
	return 4;
}

int ex1::control::size_u() const
{
	return 1;
}

void ex1::control::includeJacobian(const double & t, const vec & x, const vec & u, const vec & gradu, vec & gradu_coef)
{
	for (int i = 0; i < num_coefs(); ++i) {
		gradu_coef[i] = gradu[0] * basis(t, i);
	}
}

double ex1::control::basis(const double & t, int idx)
{
	const double s = t*1e-0;
	const double s2 = s*s;
	const double s4 = s2*s2;
	switch (idx) {
	case 1:		return s;
	case 2:		return s2;
	case 3:		return s2*s;
	case 4:		return s4;
	case 5:		return s4*s;
	case 6:		return s4*s2;
	default:	return 1.0;
	}
}
