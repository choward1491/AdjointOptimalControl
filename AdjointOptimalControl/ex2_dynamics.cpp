#include "ex2_dynamics.hpp"

void ex2::dynamics::operator()(const double & t, const vec & x, const vec & u, vec & xdot)
{
	xdot[0] = x[2];
	xdot[1] = x[3];
	xdot[2] = u[0];
	xdot[3] = u[1];
}

void ex2::dynamics::grad_x(const double & t, const vec & x, const vec & u, int dim, vec & gradx)
{
	switch (dim) {
	case 0:
	{
		gradx[0] = gradx[1] = gradx[2] = gradx[3] = 0.0; return;
	}
	case 1:
	{
		gradx[0] = gradx[1] = gradx[2] = gradx[3] = 0.0; return;
	}
	case 2:
	{
		gradx[0] = 1.0;  gradx[1] = gradx[2] = gradx[3] = 0.0; return;
	}
	default:
	{
		gradx[1] = 1.0;  gradx[0] = gradx[2] = gradx[3] = 0.0; return;
	}
	}
}

void ex2::dynamics::grad_u(const double & t, const vec & x, const vec & u, int dim, vec & gradu)
{
	switch (dim) {
	case 0:
	{
		gradu[2] = 1.0;  gradu[1] = gradu[0] = gradu[3] = 0.0; return;
	}
	case 1:
	{
		gradu[3] = 1.0;  gradu[0] = gradu[2] = gradu[1] = 0.0; return;
	}
	}
}

int ex2::dynamics::size_u() const
{
	return 2;
}

int ex2::dynamics::size_x() const
{
	return 4;
}