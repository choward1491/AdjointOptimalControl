#pragma once
#include <vector>
namespace ex1 {

	typedef std::vector<double> vec;

	class FinalCost {
	public:
		FinalCost() = default;
		~FinalCost() = default;

		double operator()(const double & t, const vec & x);
		void grad_x(const double & t, const vec & x, vec & gradx);
	};

}