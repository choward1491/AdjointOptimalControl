#pragma once
#include <vector>

namespace ex1 {

	typedef std::vector<double> vec;

	class control {
	public:

		control();
		~control() = default;

		void init();
		void getControl(const double & t, const vec & x, vec & u);
		vec & getCoefs();
		const vec & getCoefs() const;
		int num_coefs() const;
		int size_u() const;
		void includeJacobian(const double & t, const vec & x, const vec & u, const vec & gradu, vec & gradu_coef);

	private:
		vec coefs;
		double basis(const double & t, int idx);
	};


}