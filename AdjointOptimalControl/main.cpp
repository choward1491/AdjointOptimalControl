

#include <stdio.h>
#include "example1.hpp"
#include "Hamiltonian.hpp"
#include "optimal_control.hpp"

typedef opt::control::Hamiltonian<ex1::IntegralCost, ex1::dynamics> Hamiltonian1;
typedef opt::control::adjoint<Hamiltonian1, ex1::FinalCost, ex1::control> opt_adjoint1;

void saveControlHistory1(ex1::control & U, const opt::control::vec & timeframe);

int main(int argc, char** argv) {

	opt::control::vec x0(1, 1);

	opt_adjoint1 oa1;
	oa1.setIterationStepsize(1e-1);
	oa1.setMomentumCoefficient(0.9);
	oa1.setNumIterations(10000);
	oa1.setSimVars(0.0, 1.0, x0, 100);
	oa1.solve();
	saveControlHistory1(oa1.U, oa1.getTimeframe());

	return 0;
}

void saveControlHistory1(ex1::control & U, const opt::control::vec & timeframe)
{
	opt::control::vec x(1, 0.0);
	opt::control::vec u(U.size_u(), 0.0);
	FILE* fptr = fopen("control_hist.csv", "w");

	if (fptr) {
		for (size_t i = 0; i < timeframe.size(); ++i) {
			const double & t = timeframe[i];
			U.getControl(t, x, u);
			fprintf(fptr, "%lf, %lf\n", t, u[0]);
		}
		fclose(fptr); fptr = nullptr;
	}

}
