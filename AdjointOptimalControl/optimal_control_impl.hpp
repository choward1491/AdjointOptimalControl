#ifndef _opt_control_impl_hpp_
#define _opt_control_impl_hpp_

#include "optimal_control.hpp"

namespace opt {
	namespace control {

		template<class Hamiltonian, class FinalCost, class Control>
		inline adjoint<Hamiltonian, FinalCost, Control>::adjoint():tmpx(H.f.size_x(),0.0),
			tmpu(H.f.size_u(),0.0),tmpl(H.f.size_x(),0.0), momentum(U.num_coefs(),0.0),
			Hgrad_u(U.num_coefs()),u(H.f.size_u(),0.0),Nt(0),t0(0),tf(0),mom_coef(0.9)
		{
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::setSimVars(double t0_, double tf_, const vec & x0, int Nt_)
		{
			// init time stuff
			t0 = t0_; tf = tf_; Nt = Nt_;
			time.resize(Nt_);
			const double dt = (tf - t0) / static_cast<double>(Nt - 1);
			for (int i = 0; i < Nt; ++i) {
				time[i] = t0 + i*dt;
			}

			// allocate storage for histories of state and adjoint vector
			adjoint_h.resize(Nt);
			state_h.resize(Nt);
			for (int i = 0; i < Nt; ++i) {
				adjoint_h[i].resize(H.f.size_x(), 0.0);
				state_h[i].resize(H.f.size_x(), 0.0);
			}

			// set init state in state history
			state_h[0] = x0;

		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::setNumIterations(int num_iters_)
		{
			num_iters = num_iters_;
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::setIterationStepsize(double alpha_)
		{
			step_size = alpha_;
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::setMomentumCoefficient(double mom_coef_)
		{
			mom_coef = mom_coef_;
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline const vec & adjoint<Hamiltonian, FinalCost, Control>::getTimeframe() const
		{
			return time;
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::solve()
		{
			U.init(); // init controller

			// do forward-backward-sweep iterative algorithm
			for (int iter = 0; iter < num_iters; ++iter) {
				computeStateHistory();		// compute x_{i} \forall i
				computeAdjointHistory();	// compute \lambda_{i} \forall i
				computeOverallGradH_u();	// compute overall Hgrad_u, taking into account control basis
				updateControlCoefs();		// update the coefficients used in control basis representation
			}

		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::evalControl(double t, const vec & state, vec & opt_control)
		{
			U.getControl(t, state, opt_control);
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::computeAdjointHistory()
		{
			const int end = Nt - 1;
			const double dt = time[1] - time[0];
			Psi.grad_x(time[end], state_h[end], adjoint_h[end]);
			for (int i = (end - 1); i >= 0; --i) {
				double & t = time[i+1];
				vec & state = state_h[i+1];
				vec & ldot = tmpl;
				U.getControl(t, state, u);
				H.grad_x(t, state, u, adjoint_h[i + 1], ldot);
				updateVec(adjoint_h[i + 1], ldot, dt, adjoint_h[i]);
			}

		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::computeStateHistory()
		{
			const double dt = time[1] - time[0];
			for (int i = 1; i < Nt; ++i) {
				double & t = time[i - 1];
				vec & state = state_h[i - 1];
				vec & xdot = tmpx;
				U.getControl(t, state, u);
				H.f(t, state, u, xdot);
				updateVec(state, xdot, dt, state_h[i]);
			}

		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::computeOverallGradH_u()
		{
			static vec gradu(H.f.size_u(), 0.0);
			static vec gradu_coef(U.num_coefs(), 0.0);
			const double invN = 1.0/static_cast<double>(Nt);
			std::fill(Hgrad_u.begin(), Hgrad_u.end(), 0.0);

			for (int i = 0; i < Nt; ++i) {
				double & t = time[i];
				vec & state = state_h[i];
				vec & lambda = adjoint_h[i];
				U.getControl(t, state, u);
				H.grad_u(t, state, u, lambda, gradu);
				U.includeJacobian(t, state, u, gradu, gradu_coef);
				updateVec(Hgrad_u, gradu_coef, invN, Hgrad_u);
			}
		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::updateControlCoefs()
		{
			vec & coefs = U.getCoefs();
			double delta = 0.0;
			for (size_t i = 0; i < coefs.size(); ++i) {
				delta = -step_size*Hgrad_u[i] + mom_coef*momentum[i];
				coefs[i] += delta;
				momentum[i] = delta;
			}

		}

		template<class Hamiltonian, class FinalCost, class Control>
		inline void adjoint<Hamiltonian, FinalCost, Control>::updateVec(const vec & v_old, const vec & dv, double multiplier, vec & v_new)
		{
			for (size_t i = 0; i < v_old.size(); ++i) {
				v_new[i] = v_old[i] + multiplier*dv[i];
			}
		}

	}
}



#endif