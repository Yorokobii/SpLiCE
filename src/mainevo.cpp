#define MECACELL_TERMINAL_COLORS
#define MECACELL_LOGGER_WARN_DISABLE
#define MECACELL_LOGGER_DBG_DISABLE
#include <mecacell/mecacell.h>
#include <gaga/gaga.hpp>
#include "core/config.hpp"
// #include "core/evaluators.hpp"
#include "core/capture.hpp"
#include <cstdio>

int main(int argc, char** argv) {
  Config cfg(argc, argv);

  GAGA::GA<Config::CtrlType> ga(argc, argv);

  mkdir("pop/", 0777);

  ga.setEvaluator([cfg](auto& individual, int k) {

        // ----- SAVES ALL INDs BEFORE EVALUATION -----

        // std::stringstream fileName;
        // fileName << "pop/ind" << k << "_" << individual.fitnesses.[0].second << ".dna"; //not tested
        // std::ofstream fs(fileName.str());
        // if (!fs) {
        //   cerr << "Cannot open the output file." << endl;
        // }
        // fs << individual.dna.serialize();
        // fs.close();

        // Config::scenario_t scenario(cfg);
        // scenario.controller = Config::CtrlType(individual.dna);
        // scenario.init();
        // while (!scenario.finished()) scenario.loop();

        // std::remove(fileName.str().c_str());

        // -------------------

        std::vector<std::vector<double>> footprints;

        MecaCell::Vector3D com = scenario.com;
        MecaCell::Vector3D coc = MecaCell::Vector3D::zero();
        double totalContr = 0.0;
        for(auto& c : scenario.getWorld().cells){
          coc += c->contractionCount * c->getPosition();
          totalContr += c->contractionCount;
        }
        if(coc.length() != 0){
          coc /= totalContr;
          coc -= com;
        }
        
        footprints.push_back(std::vector<double>());
        footprints[0].push_back(coc.x());
        footprints[0].push_back(coc.y());
        footprints[0].push_back(coc.z());
        com.normalize();
        footprints[0].push_back(com.y());
        footprints[0].push_back(com.x());
        footprints[0].push_back(com.z());


        // novelty number of clusters

        // if (scenario.getWorld().cells.size() != 0) {
        //   auto clusters =
        //       ClusterTools::getClusters(scenario.getWorld().cells);
        //   footprints[0].push_back(1.0 - 1.0/(double)clusters.size());
        // }

        individual.footprint = footprints;

        individual.fitnesses["DistanceEnergy"] = max(0.0, 1.0 - 1.0/(scenario.fit+1));
      },
      "DistanceEnergy");

  ga.setPopSize(cfg.populationSize);
  ga.setMutationProba(cfg.mutationRate);
  ga.setCrossoverProba(cfg.crossoverRate);
  ga.setVerbosity(cfg.verbosity);
  ga.setNbThreads(cfg.nbThreads);
  ga.setSaveFolder("evos");
  ga.enableNovelty();
  ga.setKNN(cfg.populationSize/10);
  ga.setComputeFootprintDistanceFunction([](const auto& f0, const auto& f1) {
    assert(f0.size() == f1.size());
		double d = 0;
		for (size_t i = 0; i < f0.size(); ++i) {
			for (size_t j = 0; j < f0[i].size(); ++j) {
				d += std::pow(f0[i][j] - f1[i][j], 2);
			}
		}
		return sqrt(d);
  });

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
