#ifndef LALACONFIG_HPP
#define LALACONFIG_HPP
#include <mecacell/mecacell.h>
#include <fstream>
#include "../external/cxxopts/cxxopts.hpp"
#include "../external/json/json.hpp"
#include "cell.hpp"
#include "scenario.hpp"
#include "controller.hpp"
#include "grncontroller.hpp"

#define CHKPARAM(paramName)         \
  if (it.key() == "" #paramName "") \
  paramName = it.value(),           \
  MecaCell::logger<MecaCell::INF>("Config :: \"", it.key(), "\" = ", it.value())

template <typename cell_t, typename ctrl_t, typename cfg_t> class Scenario;
struct Config {
  using json = nlohmann::json;

  // ---------   STATIC CONFIG  ----------
	using CtrlType = GRNController;
  using CellType = Cell<CtrlType, Config>;
  using scenario_t = Scenario<CellType, CtrlType, Config>;

  // --------    DYNAMIC CONFIG  ----------
  // params and their default values
  double simDuration = 1000.0;
  double dt = 0.01;
  double originalRadius = 30.0;
  double adhCoef = 25.0;
  double contractRatio = 0.9;
  double contractDuration = 0.4;
  double divRadius = 20.0;
  int minActionAge = 20;
  double energyDuplicate = 1.0;
  double energyRotate = 0.1;
  double energyContraction = 0.1;
  double energyQuiescence = 0.01;
  double energyInitial = 10.0;
  double devCells = 20;
  double maxCells = 30;
  double betaAge = 0.4;
  double betaPressure = 30.0;
  double fluidDensity = 1e-4;
  int controllerUpdate = 5;
  int seed = 0;

  size_t populationSize = 200;
  double mutationRate = 0.9;
  double crossoverRate = 0.1;
  size_t nbGenerations = 2;
  size_t verbosity = 2;
  size_t nbThreads = 4;
  bool speciation = true;
  size_t minSpecieSize = 10;
  double speciationThreshold_base = 0.3;
  double speciationThreshold_increment = 0.3;
  double speciationThreshold_max = 0.8;
  double speciationThreshold_min = 0.01;

  std::string grnFile = "";

  Config(int argc, char** argv) {
    cxxopts::Options options("splice", "spike-based learning in a cellular environment");
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
      CHKPARAM(simDuration);
      else CHKPARAM(dt);
      else CHKPARAM(originalRadius);
      else CHKPARAM(adhCoef);
      else CHKPARAM(contractRatio);
      else CHKPARAM(contractDuration);
      else CHKPARAM(divRadius);
      else CHKPARAM(minActionAge);
      else CHKPARAM(energyDuplicate);
      else CHKPARAM(energyRotate);
      else CHKPARAM(energyContraction);
      else CHKPARAM(energyQuiescence);
      else CHKPARAM(energyInitial);
      else CHKPARAM(devCells);
      else CHKPARAM(maxCells);
      else CHKPARAM(betaAge);
      else CHKPARAM(betaPressure);
      else CHKPARAM(fluidDensity);
      else CHKPARAM(controllerUpdate);
      else CHKPARAM(seed);
      else CHKPARAM(populationSize);
      else CHKPARAM(mutationRate);
      else CHKPARAM(crossoverRate);
      else CHKPARAM(nbGenerations);
      else CHKPARAM(verbosity);
      else CHKPARAM(nbThreads);
      else CHKPARAM(speciation);
      else CHKPARAM(minSpecieSize);
      else CHKPARAM(speciationThreshold_base);
      else CHKPARAM(speciationThreshold_increment);
      else CHKPARAM(speciationThreshold_max);
      else CHKPARAM(speciationThreshold_min);
      else CHKPARAM(grnFile);
      else MecaCell::logger<MecaCell::WARN>("Config :: unknown field \"", it.key(), "\"");
    }
  }
};
#endif
