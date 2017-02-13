#include "ex2_integralcost.hpp"

double ex2::IntegralCost::operator()(const double & t, const vec & x, const vec & u)
{
	return 0.5*( u[0] * u[0] + u[1]*u[1] );
}

double ex2::IntegralCost::grad_u(const double & t, const vec & x, const vec & u, int dim)
{
	switch (dim) {
	case 0: return u[0];
	case 1: return u[1];
	}
}

double ex2::IntegralCost::grad_x(const double & t, const vec & x, const vec & u, int dim)
{
	switch (dim) {
	case 0: return 0.0;
	case 1: return 0.0;
	case 2: return 0.0;
	case 3: return 0.0;
	}
}