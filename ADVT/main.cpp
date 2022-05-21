#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "ADVT.hpp"
#include "ADVTOptions.hpp"

int main(int argc, char const* argv[]) {
	oppt::ProblemEnvironment p;
	p.setup<solvers::ADVT, oppt::ADVTOptions>(argc, argv);
	p.runEnvironment(argc, argv);
	return 0;
}