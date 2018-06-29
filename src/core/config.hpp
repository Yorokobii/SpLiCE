#pragma once
#include <mecacell/mecacell.h>
#include <metaconfig/metaconfig.hpp>
#include "cell.hpp"
#include "controller.hpp"
#include "scenario.hpp"

struct BaseController;
template <typename Cell, typename Conf> class Scenario;
struct Config {
	// ---------   STATIC CONFIG   ------------
	using CtrlType = BaseController;
	using CellType = Cell<BaseController>;
	using scenario_t = Scenario<CellType, Config>;

	// ---------   DYNAMICAL CONFIG  ----------
	// -- GA
	struct GAConfig {
		DECLARE_CONFIG(GAConfig, (size_t, populationSize), (double, mutationRate),
		               (double, crossoverRate), (size_t, nbGenerations), (size_t, verbosity),
		               (size_t, nbThreads), (size_t, minSpecieSize), (bool, speciation),
		               (double, speciationThreshold_base),
		               (double, speciationThreshold_increment),
		               (double, speciationThreshold_max), (double, speciationThreshold_min))
		GAConfig() {
			populationSize = 200;
			mutationRate = 0.9;
			crossoverRate = 0.1;
			nbGenerations = 400;
			verbosity = 2;
			nbThreads = 4;
			speciation = true;
			speciationThreshold_base = 0.3;
			speciationThreshold_increment = 0.3;
			speciationThreshold_max = 0.8;
			speciationThreshold_min = 0.01;
		}
	};

	// -- Simulation
	DECLARE_CONFIG(Config, (int, seed), (double, dt), (double, maxDuration),
	               (size_t, t_contract), (double, cell_radius), (double, cell_stiffness),
	               (double, cell_adhesion), (double, fluid_density), (GAConfig, ga))

	Config() {
		seed = 0;
		dt = 0.05;
		maxDuration = 20;
		t_contract = 500;
		cell_radius = 10.0;
		cell_stiffness = 10.0;
		cell_adhesion = 5.0;
		fluid_density = 1e-4;
	}
};
