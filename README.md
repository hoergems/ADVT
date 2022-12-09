Adaptive Discretization using Voronoi Trees for Continuous-Action POMDPs (ADVT)
==========================================
This repository contains the  C++ source code of our online POMDP solver ADVT: 
```
@inproceedings{hoerger22:AVDT,
  title={Adaptive Discretization using Voronoi Trees for Continuous-Action POMDPs},
  author={Hoerger, Marcus and Kurniawati, Hanna and Kroese, Dirk and Ye, Nan},
  booktitle={Proc. Int. Workshop on the Algorithmic Foundations of Robotics},  
  year={2022}
}
```

# Overview
The code consists of two parts: The `ADVT` folder contains the implementation of ADVT. The `ProblemScenarios` folder contains implementations of the example problems ADVT was evaluated on in the above paper.

# Getting started
#### Dependencies
To build and run ADVT on the provided problem scenarios, the following dependencies have to be installed:

- Ubuntu 20.04
- C++ standard 11 or higher
- OPPT: https://github.com/RDLLab/oppt (Installation instructions are provided in the repository)
- CGAL version 5.4 or higher: https://www.cgal.org/. (Installation: [Linux](https://www.cgal.org/download/linux.html))

#### Compiling ADVT
To compile ADVT, navigate to the ADVT folder an execute

	mkdir build && cd build
	cmake ..
	make
After ADVT has been successfully build, the `advt` executable can be found inside the `ADVT/bin` folder.

#### Compiling the problem scenarios
To compile the problem scenarios, navigate to the `ProblemScenarios` folder and execute

	mkdir build && cd build
	cmake -DCMAKE_INSTALL_PREFIX=<problem_scenarios_install_dir> ..
	make && make install
where `<problem_scenarios_install_dir>` is a directory of your choice.

# Running ADVT on the problem scenarios
### Configuring the OPPT environment
After successfully compiling ADVT and the problem scenarios, open a terminal and run

	source <problem_scenarios_install_dir>/share/oppt/setup.sh
Note that this command must be executed whenever a new terminal window is used. Alternatively, append this command to your `~/.bashrc`file.

### Running ADVT
After configuring the OPPT environment as described above, navigate to the `ADVT/bin` folder and execute

	./advt --cfg <ADVT_INSTALL>/ProblemScenarios/cfg/<ProblemScenario>.cfg
where `<ADVT_INSTALL>` is the folder the repository was cloned into and `<ProblemScenario>` is the configuration file for a specific problem scenario (see `ProblemScenarios/cfg`).

After running ADVT on a problem scenario, a log file is generated in the `ADVT/bin/log` directory which contains a summary of the simulation run.
