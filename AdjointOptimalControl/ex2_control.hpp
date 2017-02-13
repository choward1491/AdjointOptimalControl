#pragma once
#include <vector>

namespace ex2 {

	typedef std::vector<double> vec;

	class control {
	public:

		control();
		~control() = default;

		void init();
		void setTimeFrame(const double & t0_, const double & tf_, int Nt_);
		void getControl(const double & t, const vec & x, vec & u);
		vec & getCoefs();
		const vec & getCoefs() const;
		int num_coefs() const;
		int size_u() const;
		void includeJacobian(const double & t, const vec & x, const vec & u, const vec & gradu, vec & gradu_coef);

	private:
		double t0, tf;
		vec coefs;
		double basis(const double & t, int idx);
	};


}