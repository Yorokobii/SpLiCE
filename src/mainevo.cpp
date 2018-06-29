#include <mecacell/mecacell.h>
#include "core/config.hpp"

int main(int argc, char** argv) {
	Config cfg();

	GAGA::GA<Config::CtrlType> ga(argc, argv);

	ga.setEvaluator(
	    [cfg](auto& individual, int) {
		    Config::scenario_t scenario(cfg);
		    scenario.init();
		    while (!scenario.finished()) scenario.loop();

		    individual.fitnesses["DistanceEnergy"] = 1;  // TODO: compute fitness

	    },
	    "DistanceEnergy");

	ga.setPopSize(cfg.ga.populationSize);
	ga.setMutationProba(cfg.ga.mutationRate);
	ga.setCrossoverProba(cfg.ga.crossoverRate);
	ga.setVerbosity(cfg.ga.verbosity);
	ga.setNbThreads(cfg.ga.nbThreads);

	if (cfg.ga.speciation) {
		ga.enableSpeciation();
		ga.setMinSpecieSize(cfg.ga.minSpecieSize);
		ga.setSpeciationThreshold(cfg.ga.speciationThreshold_base);
		ga.setSpeciationThresholdIncrement(cfg.ga.speciationThreshold_increment);
		ga.setMaxSpeciationThreshold(cfg.ga.speciationThreshold_max);
		ga.setMinSpeciationThreshold(cfg.ga.speciationThreshold_min);
		ga.setIndDistanceFunction([](const auto& a, const auto& b) {
			return Config::CtrlType::getDistance(a.dna, b.dna);
		});
	}

	ga.step(cfg.ga.nbGenerations);
	return 0;
}
