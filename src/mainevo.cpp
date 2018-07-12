#define MECACELL_TERMINAL_COLORS
#define MECACELL_LOGGER_WARN_DISABLE
#define MECACELL_LOGGER_DBG_DISABLE
#include <mecacell/mecacell.h>
#include <gaga/gaga.hpp>
#include "core/config.hpp"
// #include "core/evaluators.hpp"
#include "core/capture.hpp"

int main(int argc, char** argv) {
  Config cfg(argc, argv);

  GAGA::GA<Config::CtrlType> ga(argc, argv);

  ga.setEvaluator([cfg](auto& individual, int) {
        Config::scenario_t scenario(cfg);
        scenario.controller = Config::CtrlType(individual.dna);
        scenario.init();
        while (!scenario.finished()) scenario.loop();

        std::vector<std::vector<double>> footprints;
        footprints.push_back(std::vector<double>());

        //compute nconn footprint
        float nconn = 0.0;
        float ncontr = 0.0;
        for(auto& c : scenario.getWorld().cells){
          for(auto& conn : c->getBody().cellConnections)
            nconn += (conn->adhCoef > 0.0) ? 1.0 : 0.0;
          ncontr += c->contractionCount;
        }
        nconn /= scenario.getWorld().cells.size();
        ncontr /= scenario.getWorld().cells.size();
        // footprints[0].push_back(nconn);
        footprints[0].push_back(ncontr);
        // footprints[0].push_back(scenario.getWorld().cells.size());

        // if (scenario.getWorld().cells.size() != 0) {
        //   auto clusters =
        //       ClusterTools::getClusters(scenario.getWorld().cells);
        //   footprints[0].push_back(clusters.size());
        // }

        individual.footprint = footprints;

        individual.fitnesses["DistanceEnergy"] = scenario.getWorld().cells.size() < cfg.minCells ? 0.0 : scenario.fit;
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
  // ga.setComputeFootprintDistanceFunction([](const auto& f0, const auto& f1) {
  //   assert(f0.size() == f1.size());
	// 	double d = 0;
	// 	for (size_t i = 0; i < f0.size(); ++i) {
	// 		for (size_t j = 0; j < f0[i].size(); ++j) {
	// 			d += std::pow(f0[i][j] - f1[i][j], 2);
	// 		}
	// 	}
	// 	return sqrt(d);
  // });

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