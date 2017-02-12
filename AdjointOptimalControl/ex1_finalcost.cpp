#include "ex1_finalcost.hpp"

double ex1::FinalCost::operator()(const double & t, const vec & x)
{
	return 0.0;
}

void ex1::FinalCost::grad_x(const double & t, const vec & x, vec & gradx)
{
	gradx[0] = 0.0;
}
