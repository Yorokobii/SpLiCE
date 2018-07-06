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
        scenario.controller = Config::CtrlType(individual.dna);
        scenario.init();
        while (!scenario.finished()) scenario.loop();

        std::vector<std::vector<double>> footprints;

        //compute nconn footprint
        float nconn;
        for(auto& c : scenario.getWorld().cells)
          for(auto& conn : c->getBody().cellConnections)
            nconn += (conn->adhCoef > 0.0) ? 1.0 : 0.0;
        nconn /= scenario.getWorld().cells.size();

        footprints.push_back({nconn});

        //


        individual.footprint = footprints;

        individual.fitnesses["DistanceEnergy"] = scenario.fit;
        // individual.fitnesses["ShapeEnergy"] = scenario.shapefit;
      },
      "DistanceEnergy");

  ga.setPopSize(cfg.populationSize);
  ga.setMutationProba(cfg.mutationRate);
  ga.setCrossoverProba(cfg.crossoverRate);
  ga.setVerbosity(cfg.verbosity);
  ga.setNbThreads(cfg.nbThreads);
  ga.setSaveFolder("evos");
  ga.enableNovelty();

  // ga.setSaveParetoFront(true);

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
                      return Config::CtrlType::random();
                    });
  ga.step(cfg.nbGenerations);
  return 0;
}
