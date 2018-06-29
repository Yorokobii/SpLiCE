#pragma once
#include <grgen/grn.hpp>
#include <grgen/real.hpp>

struct GRNController {  // both controller and DNA
	using GRN_t = GRN<RealCoords>;
	GRN_t grn;
	GRNController() {
		grn.randomParams();   // Random Beta & Delta
		grn.randomReguls(1);  // start with one regul

		grn.addRandomProtein(ProteinType::input, "gcomm");
		// ...
		grn.addRandomProtein(ProteinType::output, "out1");
		// ...
	}

	MyDNA(GRN_t g) : grn(g) {}

	/***** GAGA requirements: ********/
	MyDNA(const string& s) : grn(s) {}  // unserialize
	std::string serialize() const { return grn.serialize(); }

	void reset() { grn.reset(); }

	void mutate() { grn.mutate(); }
	GRNController crossover(const GRNController& other) {
		return GRNController(grn.crossover(other.grn));
	}
	inline static double getDistance(const GRNController& a, const GRNController& b) {
		return GRN_t::getDistance(a, b);
	}
};
