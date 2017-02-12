#include "ex1_dynamics.hpp"

void ex1::dynamics::operator()(const double & t, const vec & x, const vec & u, vec & xdot)
{
	xdot[0] = -x[0] + u[0];
}

void ex1::dynamics::grad_x(const double & t, const vec & x, const vec & u, int dim, vec & gradx)
{
	gradx[0] = -1.0;
}

void ex1::dynamics::grad_u(const double & t, const vec & x, const vec & u, int dim, vec & gradu)
{
	gradu[0] = 1.0;
}

int ex1::dynamics::size_u() const
{
	return 1;
}

int ex1::dynamics::size_x() const
{
	return 1;
}
