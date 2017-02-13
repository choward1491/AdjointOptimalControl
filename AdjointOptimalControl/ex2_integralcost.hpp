#pragma once

#include <vector>

namespace ex2 {

	typedef std::vector<double> vec;

	class IntegralCost {
	public:

		IntegralCost() = default;
		~IntegralCost() = default;

		double operator()(const double & t, const vec & x, const vec & u);
		double grad_u(const double & t, const vec & x, const vec & u, int dim);
		double grad_x(const double & t, const vec & x, const vec & u, int dim);

	};

}