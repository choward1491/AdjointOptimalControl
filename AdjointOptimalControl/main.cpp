

#include <stdio.h>
#include "example1.hpp"
#include "example2.hpp"
#include "Hamiltonian.hpp"
#include "optimal_control.hpp"

typedef opt::control::Hamiltonian<ex1::IntegralCost, ex1::dynamics> Hamiltonian1;
typedef opt::control::adjoint<Hamiltonian1, ex1::FinalCost, ex1::control> opt_adjoint1;
typedef opt::control::Hamiltonian<ex2::IntegralCost, ex2::dynamics> Hamiltonian2;
typedef opt::control::adjoint<Hamiltonian2, ex2::FinalCost, ex2::control> opt_adjoint2;

void saveControlHistory1(ex1::control & U, const opt::control::vec & timeframe);
void saveControlHistory2(ex2::control & U, const opt::control::vec & timeframe);

int main(int argc, char** argv) {

	/*
	opt::control::vec x0(1, 1);
	opt_adjoint1 oa1;
	oa1.setIterationStepsize(1e-1);
	oa1.setMomentumCoefficient(0.9);
	oa1.setNumIterations(10);
	oa1.setSimVars(0.0, 1.0, x0, 50);
	oa1.solve();
	saveControlHistory1(oa1.U, oa1.getTimeframe());
	*/

	opt::control::vec x0(4, 0.0);
	x0[0] = -10000; x0[1] = 5000;
	x0[2] = 180; x0[3] = 0.0;

	opt_adjoint2 oa2;
	oa2.setIterationStepsize(1e-7);
	oa2.setMomentumCoefficient(0.9);
	oa2.setNumIterations(50000);
	oa2.setSimVars(0.0, 40.0, x0, 1000);
	oa2.solve();
	saveControlHistory2(oa2.U, oa2.getTimeframe());

	return 0;
}

void saveControlHistory1(ex1::control & U, const opt::control::vec & timeframe)
{
	opt::control::vec x(1, 0.0);
	opt::control::vec u(U.size_u(), 0.0);
	FILE* fptr = fopen("../matlab/data/control_hist.csv", "w");

	if (fptr) {
		for (size_t i = 0; i < timeframe.size(); ++i) {
			const double & t = timeframe[i];
			U.getControl(t, x, u);
			fprintf(fptr, "%lf, %lf\n", t, u[0]);
		}
		fclose(fptr); fptr = nullptr;
	}

}

void saveControlHistory2(ex2::control & U, const opt::control::vec & timeframe)
{
	opt::control::vec x(4, 0.0);
	opt::control::vec u(U.size_u(), 0.0);
	FILE* fptr = fopen("../matlab/data/control_hist2.csv", "w");

	if (fptr) {
		for (size_t i = 0; i < timeframe.size(); ++i) {
			const double & t = timeframe[i];
			U.getControl(t, x, u);
			//fprintf(fptr, "%lf, %lf, %lf\n", t, u[0],u[1]);
			fprintf(fptr, "%lf, %lf\n", t, u[0]);
		}
		fclose(fptr); fptr = nullptr;
	}

}
