#pragma once
#include <vector>

namespace opt {
	namespace control {

		typedef std::vector<double> vec;

		template<class IC, class F>
		class Hamiltonian {
		public:
			Hamiltonian() = default;
			~Hamiltonian() = default;

			double operator()(const double & t, const vec & x, const vec & u, const vec & lambda );
			void grad_x(const double & t, const vec & x, const vec & u, const vec & lambda, vec & Hgrad_x);
			void grad_u(const double & t, const vec & x, const vec & u, const vec & lambda, vec & Hgrad_u);

			IC L;	// integral cost term
			F f;	// dynamics operator

		private:

			//helper functions
			void inner_prod(const vec & v1, const vec & v2, double & out) const;
		};

		template<class IC, class F>
		inline double Hamiltonian<IC, F>::operator()(const double & t, const vec & x, const vec & u, const vec & lambda)
		{
			static vec xdot(f.size_x(), 0.0);
			double dyn_term = 0.0;
			f(t, x, u, xdot);
			inner_prod(lambda, xdot, dyn_term);
			return L(t, x, u) + dyn_term;
		}

		template<class IC, class F>
		inline void Hamiltonian<IC, F>::grad_x(const double & t, const vec & x, const vec & u, const vec & lambda, vec & Hgrad_x)
		{
			static vec fgrad_x(f.size_x(), 0.0);
			double dyn_term = 0.0;
			for (int i = 0; i < f.size_x(); ++i) {
				f.grad_x(t, x, u, i, fgrad_x);
				inner_prod(lambda, fgrad_x, dyn_term);
				Hgrad_x[i] = L.grad_x(t, x, u, i) + dyn_term;
			}

		}

		template<class IC, class F>
		inline void Hamiltonian<IC, F>::grad_u(const double & t, const vec & x, const vec & u, const vec & lambda, vec & Hgrad_u)
		{
			static vec fgrad_u(f.size_x(), 0.0);
			double dyn_term = 0.0;
			for (int i = 0; i < f.size_u(); ++i) {
				f.grad_u(t, x, u, i, fgrad_u);
				inner_prod(lambda, fgrad_u, dyn_term);
				Hgrad_u[i] = L.grad_u(t, x, u, i) + dyn_term;
			}
		}
		template<class IC, class F>
		inline void Hamiltonian<IC, F>::inner_prod(const vec & v1, const vec & v2, double & out) const
		{
			out = 0.0;
			for (size_t i = 0; i < v1.size(); ++i) { out += v1[i] * v2[i]; }
		}
	}
}