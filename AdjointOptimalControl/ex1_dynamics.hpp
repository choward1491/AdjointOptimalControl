#pragma once
#include <vector>

namespace ex1 {
	typedef std::vector<double> vec;

	class dynamics {
	public:

		dynamics() = default;
		~dynamics() = default;

		void operator()(const double & t, const vec & x, const vec & u, vec & xdot);
		void grad_x(const double & t, const vec & x, const vec & u, int dim, vec & gradx);
		void grad_u(const double & t, const vec & x, const vec & u, int dim, vec & gradu);
		int size_u() const;
		int size_x() const;


	};

}