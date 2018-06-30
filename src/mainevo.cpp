#define MECACELL_TERMINAL_COLORS
#define MECACELL_LOGGER_WARN_DISABLE
#define MECACELL_LOGGER_DBG_DISABLE
#include <mecacell/mecacell.h>
#include <gaga/gaga.hpp>
#include "core/config.hpp"

int main(int argc, char** argv) {
  Config cfg(argc, argv);

  GAGA::GA<Config::CtrlType> ga(argc, argv);

  ga.setEvaluator([cfg](auto& individual, int) {
        Config::scenario_t scenario(cfg);
        scenario.init();
        while (!scenario.finished()) scenario.loop();

        MecaCell::Vec com = scenario.com;

        individual.fitnesses["DistanceEnergy"] = (com.x() * com.x() +
                                                  com.y() * com.y() +
                                                  com.z() * com.z()); // TODO: use julia.
      },
      "DistanceEnergy");

  ga.setPopSize(cfg.populationSize);
  ga.setMutationProba(cfg.mutationRate);
  ga.setCrossoverProba(cfg.crossoverRate);
  ga.setVerbosity(cfg.verbosity);
  ga.setNbThreads(cfg.nbThreads);
  ga.setSaveFolder("evos");

  if (cfg.speciation) {
    ga.enableSpeciation();
    ga.setMinSpecieSize(cfg.minSpecieSize);
    ga.setSpeciationThreshold(cfg.speciationThreshold_base);
    ga.setSpeciationThresholdIncrement(cfg.speciationThreshold_increment);
    ga.setMaxSpeciationThreshold(cfg.speciationThreshold_max);
    ga.setMinSpeciationThreshold(cfg.speciationThreshold_min);
    ga.setIndDistanceFunction([](const auto& a, const auto& b) {
      return Config::CtrlType::getDistance(a.dna, b.dna);
    });
  }

	ga.initPopulation([]() {
                      Config::CtrlType dna;
                      return dna;
                    });
  ga.step(cfg.nbGenerations);
  return 0;
}
