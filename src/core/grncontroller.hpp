#pragma once
// #include <mecacell/mecacell.h>
#include <grgen/grn.hpp>
#include <grgen/real.hpp>

struct GRNController {  // both controller and DNA
  using GRN_t = GRN<RealCoords>;
  static constexpr unsigned int nbMorphogens = 3;
  GRN_t grn;
  GRNController() {}

	static GRNController random() {
    GRN_t g;
    g.addRandomProtein(ProteinType::input, "theta");
    g.addRandomProtein(ProteinType::input, "phi");
    g.addRandomProtein(ProteinType::input, "pressure");
    g.addRandomProtein(ProteinType::input, "energy");
    g.addRandomProtein(ProteinType::input, "comdist");
    g.addRandomProtein(ProteinType::input, "contracting");
    g.addRandomProtein(ProteinType::input, "deltcom");
    g.addRandomProtein(ProteinType::input, "ncells");
    g.addRandomProtein(ProteinType::input, "isDuplicated");
    g.addRandomProtein(ProteinType::input, "surroundContract");
    for(auto i = 0u ; i < nbMorphogens; ++i)
      g.addRandomProtein(ProteinType::input, "inputMorphogen" + std::to_string(i));

    g.addRandomProtein(ProteinType::output, "duplicate");
    g.addRandomProtein(ProteinType::output, "rotate");
    g.addRandomProtein(ProteinType::output, "quiescence");
    g.addRandomProtein(ProteinType::output, "contraction");
    g.addRandomProtein(ProteinType::output, "extension");
    g.addRandomProtein(ProteinType::output, "apoptosis");
    g.addRandomProtein(ProteinType::output, "thetaPlus");
    g.addRandomProtein(ProteinType::output, "thetaMinus");
    g.addRandomProtein(ProteinType::output, "phiPlus");
    g.addRandomProtein(ProteinType::output, "phiMinus");
    for(auto i = 0u ; i < nbMorphogens; ++i)
      g.addRandomProtein(ProteinType::output, "outputMorphogen" + std::to_string(i));


    g.randomParams();   // Random Beta & Delta
    g.randomReguls(5);  // start with one regul
    return GRNController(g);
  }

  GRNController(const GRNController &c) : grn(c.grn) { reset(); }

  GRNController(GRN_t g) : grn(g) { reset(); }

  GRNController(const string& s) : grn(s) {}  // unserialize
  std::string serialize() const { return grn.serialize(); }

  void reset() { grn.reset(); }

  void mutate() { grn.mutate(); }

  void update() { grn.step(); }

  void setInput(const std::string &input, double val, const int& verb = 1) {
    if(verb >= 3) MecaCell::logger<MecaCell::DBG>("input: ", input, " ", val);
    grn.setProteinConcentration(input, ProteinType::input, val);
  }

  double getOutput(const std::string &output, const int& verb = 1) const {
    auto r = grn.getProteinConcentration(output, ProteinType::output);
    if(verb >= 3) MecaCell::logger<MecaCell::DBG>("output: ", output, " ", r);
    return r;
  }

  double getInput(const std::string &input, const int& verb = 1) const {
    auto r = grn.getProteinConcentration(input, ProteinType::input);
    if(verb >= 3) MecaCell::logger<MecaCell::DBG>("input: ", input, " ", r);
    return r;
  }

  double getDelta(const std::string &o1, const std::string &o2) {
    double plus = getOutput(o1);
    double minus = getOutput(o2);
    double div = plus + minus;
    if (div > 0.0) {
      return (plus - minus) / div;
    }
    return 0.0;
  }

  GRNController crossover(const GRNController& other) {
    return GRNController(grn.crossover(other.grn));
  }
  inline static double getDistance(const GRNController& a, const GRNController& b) {
    return GRN_t::getDistance(a.grn, b.grn);
  }
};
