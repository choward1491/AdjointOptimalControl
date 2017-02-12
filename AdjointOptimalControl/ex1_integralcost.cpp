#include "ex1_integralcost.hpp"

double ex1::IntegralCost::operator()(const double & t, const vec & x, const vec & u)
{
	return 0.5*(x[0]*x[0] + u[0]*u[0]);
}

double ex1::IntegralCost::grad_u(const double & t, const vec & x, const vec & u, int dim)
{
	return u[0];
}

double ex1::IntegralCost::grad_x(const double & t, const vec & x, const vec & u, int dim)
{
	return x[0];
}
