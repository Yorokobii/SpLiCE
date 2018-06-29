#ifndef LALACONFIG_HPP
#define LALACONFIG_HPP
#include <mecacell/mecacell.h>
#include <fstream>
#include "../external/cxxopts/cxxopts.hpp"
#include "../external/json/json.hpp"
#include "cell.hpp"
#include "scenario.hpp"
#include "controller.hpp"

#define CHKPARAM(paramName)         \
  if (it.key() == "" #paramName "") \
  paramName = it.value(),           \
  MecaCell::logger<MecaCell::INF>("Config :: \"", it.key(), "\" = ", it.value())

template <typename Cell, typename Conf> class Scenario;
struct Config {
  using json = nlohmann::json;

  // ---------   STATIC CONFIG  ----------
	using CtrlType = BaseController;
  using CellType = Cell<BaseController, Config>;
  using scenario_t = Scenario<CellType, Config>;

  // --------    DYNAMIC CONFIG  ----------
  // params and their default values
  double sim_duration = 1000.0;
  double sim_dt = 0.1;
  int t_contract = 500;
  double cell_radius = 10.0;
  double cell_mass = 0.001;
  double cell_stiffness = 10.0;
  double cell_adhesion = 5.0;
  double div_radius = 20.0;
  double fluid_density = 1e-4;
  double energy_initial = 10.0;
  double energy_duplicate = 1.0;
  double energy_rotate = 0.1;
  double energy_apoptosis = 0.5;
  double energy_contraction = 0.1;
  double energy_quiescence = 0.01;
  int seed = 0;

  Config(int argc, char** argv) {
    cxxopts::Options options("lala", "lala experiment program");
    options.add_options()("f,file", "configuration file", cxxopts::value<std::string>());
    options.parse(argc, argv);

    if (!options.count("file"))
      MecaCell::logger<MecaCell::WARN>("No configuration file specified. Using defaults");
    else
      load(options["file"].as<std::string>());
  }

  // loads a conf file
  void load(std::string file) {
    std::ifstream t(file);
    std::stringstream buffer;
    buffer << t.rdbuf();
    auto o = json::parse(buffer.str());
    for (auto it = o.begin(); it != o.end(); ++it) {
      CHKPARAM(sim_duration);
      else CHKPARAM(sim_dt);
      else CHKPARAM(t_contract);
      else CHKPARAM(cell_radius);
      else CHKPARAM(cell_mass);
      else CHKPARAM(cell_stiffness);
      else CHKPARAM(cell_adhesion);
      else CHKPARAM(div_radius);
      else CHKPARAM(fluid_density);
      else CHKPARAM(seed);
      else MecaCell::logger<MecaCell::WARN>("Config :: unknown field \"", it.key(), "\"");
    }
  }
};
#endif
