#pragma once

#include <vector>

namespace opt {
	namespace control {

		typedef std::vector<double> vec;


		/*
			Class to numerically solve optimal control problems
			using the adjoint method and a forward-backward-sweep
			iterative method. This templated class takes in classes
			that store the Hamiltonian, the Final Cost function, and
			the Control function.

			The Hamiltonian class will store the dynamics and will have
			the integral cost contained within it.
		*/
		template<class Hamiltonian, class FinalCost, class Control>
		class adjoint {
		public:

			// constructor
			adjoint();
			~adjoint() = default;

			// setters
			void setSimVars(double t0_, double tf_, const vec & x0, int Nt_);
			void setNumIterations(int num_iters_);
			void setIterationStepsize(double alpha_);
			void setMomentumCoefficient(double mom_coef_);

			// getters
			const vec & getTimeframe() const;

			// method to solve for optimal control 
			// using adjoint method numerically
			void solve();

			// method to get the control from the current controller
			// that exists within this class
			void evalControl(double t, const vec& state, vec & opt_control);

			// important entities for computation
			Hamiltonian H;
			FinalCost   Psi;
			Control     U;

		private:
			// internal storage for important
			// mathematical items
			std::vector<vec> adjoint_h;
			std::vector<vec> state_h;
			vec time;
			vec u, tmpx, tmpu, tmpl, Hgrad_u, momentum;
			int Nt, num_iters;
			double t0, tf, step_size, mom_coef;

			// helper methods
			void computeAdjointHistory();
			void computeStateHistory();
			void computeOverallGradH_u();
			void updateControlCoefs();
			void updateVec(const vec & v_old, const vec & dv, double multiplier, vec & v_new);

		};

	}
}

#include "optimal_control_impl.hpp"