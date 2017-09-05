#include "ex2_dynamics.hpp"

void ex2::dynamics::operator()(const double & t, const vec & x, const vec & u, vec & xdot)
{
	const double vmag = sqrt(x[2] * x[2] + x[3] * x[3]);
	const double d[2] = { x[3] / vmag, -x[2] / vmag };
	const double g = 9.81;
	const double c = 1e-4;
	const double dragv[2] = {c*vmag*x[2], c*vmag*x[3]};
	xdot[0] = x[2];
	xdot[1] = x[3];
	xdot[2] = d[0]*u[0] - dragv[0];
	xdot[3] = d[1]*u[0] - g - dragv[1];
}

void ex2::dynamics::grad_x(const double & t, const vec & x, const vec & u, int dim, vec & gradx)
{
	const double c = 1e-4;
	const double vmag = sqrt(x[2] * x[2] + x[3] * x[3]);
	const double d[2] = { x[3] / vmag, -x[2] / vmag };
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
		const double dd[2] = {-d[0]*x[2]/(vmag*vmag), -(1 + d[1]*x[2]/vmag)/vmag};
		const double dragd[2] = { c*(x[2]*x[2]/vmag + vmag), c*x[3]*x[2]/vmag};
		gradx[0] = 1.0;  gradx[1] = 0.0;
		gradx[2] = dd[0] * u[0] - dragd[0];
		gradx[3] = dd[1] * u[0] - dragd[1];
		return;
	}
	default:
	{
		const double dd[2] = { (1.0 - d[0] * x[3] / vmag)/vmag, -d[1] * x[3] / (vmag * vmag) };
		const double dragd[2] = { c*(x[2] * x[3] / vmag), c*(x[3] * x[3] / vmag + vmag) };
		gradx[1] = 1.0;  gradx[0] = 0.0;
		gradx[2] = dd[0] * u[0] - dragd[0];
		gradx[3] = dd[1] * u[0] - dragd[1];
		return;
	}
	}
}

void ex2::dynamics::grad_u(const double & t, const vec & x, const vec & u, int dim, vec & gradu)
{
	const double vmag = sqrt(x[2] * x[2] + x[3] * x[3]);
	const double d[2] = { x[3] / vmag, -x[2] / vmag };

	gradu[0] = gradu[1] = 0.0;
	gradu[2] = d[0];
	gradu[3] = d[1];

	/*switch (dim) {
	case 0:
	{
		gradu[2] = 1.0;  gradu[1] = gradu[0] = gradu[3] = 0.0; return;
	}
	case 1:
	{
		gradu[3] = 1.0;  gradu[0] = gradu[2] = gradu[1] = 0.0; return;
	}
	}*/
}

int ex2::dynamics::size_u() const
{
	return 1;
}

int ex2::dynamics::size_x() const
{
	return 4;
}