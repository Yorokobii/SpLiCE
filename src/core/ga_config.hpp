#ifndef LALACONFIG_HPP
#define LALACONFIG_HPP
#include <mecacell/mecacell.h>
#include <metaconfig/metaconfig.hpp>
#include "cell.hpp"
#include "controller.hpp"
#include "scenario.hpp"

// For jean, errors when using this config:
// ‘struct Config::GAConfig’ has no member named ‘from_json’
// template <typename T> void from_json(const nlohmann::json& j, T& e) { e.from_json(j); }

template <typename Cell, typename Conf> class Scenario;
struct Config {
	// ---------   STATIC CONFIG   ------------
	using CtrlType = BaseController;
	using CellType = Cell<BaseController, Config>;
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
	DECLARE_CONFIG(Config, (int, seed), (double, sim_dt), (double, sim_duration),
	               (size_t, t_contract), (double, cell_radius), (double, cell_stiffness),
	               (double, cell_adhesion), (double, div_radius), (double, fluid_density),
                 (double, energy_initial), (double, energy_duplicate),
                 (double, energy_rotate), (double, energy_apoptosis),
                 (double, energy_contraction), (double, energy_quiescence),
                 // (int, min_action_age))
                 (int, min_action_age), (GAConfig, ga))

	Config() {
		seed = 0;
		sim_dt = 0.05;
		sim_duration = 20;
		t_contract = 500;
		cell_radius = 10.0;
		cell_stiffness = 10.0;
		cell_adhesion = 5.0;
		fluid_density = 1e-4;
    div_radius = 20;
    energy_initial = 10.0;
    energy_duplicate = 1.0;
    energy_rotate = 0.1;
    energy_apoptosis = 0.5;
    energy_contraction = 0.1;
    energy_quiescence = 0.01;
    min_action_age = 20;
	}
};

#endif
