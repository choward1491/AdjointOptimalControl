#include "ex2_control.hpp"

ex2::control::control() :coefs(num_coefs(), 0.0)
{
}

void ex2::control::init()
{
	std::fill(coefs.begin(), coefs.end(), 0.0);
}

void ex2::control::setTimeFrame(const double & t0_, const double & tf_, int Nt_)
{
	t0 = t0_;
	tf = tf_;
}

void ex2::control::getControl(const double & t, const vec & x, vec & u)
{
	int nhalf = num_coefs() / 2;
	u[0] = u[1] = 0.0;
	for (int i = 0; i < nhalf; ++i) {
		u[0] += coefs[i] * basis(t, i);
	}
	for (int i = 0; i < nhalf; ++i) {
		u[1] += coefs[i+nhalf] * basis(t, i);
	}

}

ex2::vec & ex2::control::getCoefs()
{
	return coefs;
}

const ex2::vec & ex2::control::getCoefs() const
{
	return coefs;
}

int ex2::control::num_coefs() const
{
	return 6;
}

int ex2::control::size_u() const
{
	return 2;
}

void ex2::control::includeJacobian(const double & t, const vec & x, const vec & u, const vec & gradu, vec & gradu_coef)
{
	const int nhalf = num_coefs() / 2;
	for (int i = 0; i < nhalf; ++i) {
		gradu_coef[i] = gradu[0] * basis(t, i);
	}
	for (int i = 0; i < nhalf; ++i) {
		gradu_coef[i+nhalf] = gradu[1] * basis(t, i);
	}
}

double ex2::control::basis(const double & t, int idx)
{
	const double s = (tf - t) / (tf - t0);
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

	/* // RBF approach .. doesn't do too great
	const double s = t / (tf - t0);
	const double sigma = 1.0 / static_cast<double>(num_coefs() - 1);
	const double ds = 1.0 / static_cast<double>(num_coefs() - 1);
	const double center = idx*ds;
	const double sn = (s - center) / sigma;
	return exp(-sn*sn);
	*/

}
