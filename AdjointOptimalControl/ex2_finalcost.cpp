#include "ex2_finalcost.hpp"

double ex2::FinalCost::operator()(const double & t, const vec & x)
{
	return x[0]*x[0] + x[1]*x[1];
}

void ex2::FinalCost::grad_x(const double & t, const vec & x, vec & gradx)
{
	gradx[0] = 2 * x[0];
	gradx[1] = 2 * x[1];
	gradx[2] = 0.0;
	gradx[3] = 0.0;
}