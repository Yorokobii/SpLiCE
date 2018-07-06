#define MECACELL_TERMINAL_COLORS
#define MECACELL_LOGGER_WARN_DISABLE
#define MECACELL_LOGGER_DBG_DISABLE
#include <mecacell/mecacell.h>
#include <gaga/gaga.hpp>
#include "core/config.hpp"
// #include "core/evaluators.hpp"
#include "capture.hpp"

int main(int argc, char** argv) {
  Config cfg(argc, argv);

  GAGA::GA<Config::CtrlType> ga(argc, argv);

  ga.setEvaluator([cfg](auto& individual, int) {
        Config::scenario_t scenario(cfg);
        scenario.controller = Config::CtrlType(individual.dna);
        scenario.init();
        while (!scenario.finished()) scenario.loop();

        std::vector<std::vector<double>> footprints;
        std::vector<double> fp;

        //compute nconn footprint
        float nconn;
        for(auto& c : scenario.getWorld().cells)
          for(auto& conn : c->getBody().cellConnections)
            nconn += (conn->adhCoef > 0.0) ? 1.0 : 0.0;
        nconn /= scenario.getWorld().cells.size();
        fp.push_back(nconn);
        fp.push_back(scenario.getWorld().cells.size()/10.0);

        ComplexMorphologyEvaluator<Config::scenario_t> cme(argc, argv);
        // for(auto& v : cme.getFootprint(scenario))
        //   fp.push_back(v);
        footprints.push_back(cme.getFootprint(scenario));

        footprints.push_back(fp);
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


template <typename Cell> double computeSphericity(const unordered_set<Cell *> clust, float radius) {
  MecaCell::Grid<Cell *> grid(radius / 3.0);
  for (auto &c : clust) grid.insert(c);
  auto s = grid.computeSphericity();
  return s;
}

std::vector<double> getFootprint(Config::scenario_t &sc) {
  if (sc.getWorld().cells.size() == 0) {
    return {{0, 0}};
  }
  auto clusters =
      ClusterTools::getClusters(sc.getWorld().cells);
  size_t biggestClusterId = 0;
  size_t biggestClusterSize = clusters[biggestClusterId].size();
  for (size_t i = 1; i < clusters.size(); ++i) {
    if (clusters[i].size() > biggestClusterSize) {
      biggestClusterSize = clusters[i].size();
      biggestClusterId = i;
    }
  }
  return {{static_cast<double>(biggestClusterSize), 0.0}};

  if (sc.getWorld().cells.size() == 0) {
    return {{-1, -1, -1, -1}};
  }

  auto sphericity = computeSphericity(clusters[biggestClusterId], sc.config.originalRadius);
  vector<typename Config::scenario_t::cell_t *> biggestClusterVec;
  biggestClusterVec.reserve(clusters[biggestClusterId].size());
  MecaCell::Vec centroid(0, 0, 0);
  for (auto &c : clusters[biggestClusterId]) {
    biggestClusterVec.push_back(c);
    centroid += c->getPosition();
  }
  if (biggestClusterSize > 0) {
    centroid /= static_cast<double>(biggestClusterSize);
  }

  auto longestAxis = getLongestDistance(biggestClusterVec);
  double longestProj2 = 0;
  double longestProj3 = 0;
  double longestDist = longestAxis.length();
  if (longestDist > 0) {
    MecaCell::Vec axis2 = longestAxis.ortho().normalized();
    MecaCell::Vec axis3 = longestAxis.cross(axis2).normalized();
    for (size_t i = 0; i < biggestClusterSize; ++i) {
      for (size_t j = i + 1; j < biggestClusterSize; ++j) {
        auto AB =
            biggestClusterVec[i]->getPosition() - biggestClusterVec[j]->getPosition();
        auto proj2 = fabs(AB.dot(axis2));
        auto proj3 = fabs(AB.dot(axis3));
        if (longestProj2 < proj2) longestProj2 = proj2;
        if (longestProj3 < proj3) longestProj3 = proj3;
      }
    }
  }
  double normalizedNbCells = ClusterTools::sqAsympt(
      biggestClusterSize, 0.002);  // counts less after 1000 cells.
  double ratioProj2 = 1.0;
  double ratioProj3 = 1.0;
  if (longestDist > 0) {
    ratioProj2 = longestProj2 / longestDist;
    ratioProj3 = longestProj3 / longestDist;
  }
  return {{normalizedNbCells, sphericity, ratioProj2, ratioProj3}};
}

template <typename Cell> MecaCell::Vec getLongestDistance(std::vector<Cell *> cells) {
  MecaCell::Vec longest(0, 0, 0);
  double sql = 0;
  for (size_t i = 0; i < cells.size(); ++i) {
    for (size_t j = 0; j < cells.size(); ++j) {
      auto axis = cells[i]->getPosition() - cells[j]->getPosition();
      auto l = axis.sqlength();
      if (l > sql) {
        sql = l;
        longest = axis;
      }
    }
  }
  return longest;
}