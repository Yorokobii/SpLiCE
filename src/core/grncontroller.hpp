#pragma once
#include <grgen/grn.hpp>
#include <grgen/real.hpp>

struct GRNController {  // both controller and DNA
  using GRN_t = GRN<RealCoords>;
  GRN_t grn;
  GRNController() {}

	static GRNController random() {
    GRN_t g;
    g.addRandomProtein(ProteinType::input, "age");
    g.addRandomProtein(ProteinType::input, "gcomm");
    g.addRandomProtein(ProteinType::input, "lcomm");
    g.addRandomProtein(ProteinType::input, "theta");
    g.addRandomProtein(ProteinType::input, "phi");
    g.addRandomProtein(ProteinType::input, "pressure");

    g.addRandomProtein(ProteinType::output, "duplicate");
    g.addRandomProtein(ProteinType::output, "rotate");
    g.addRandomProtein(ProteinType::output, "quiescence");
    g.addRandomProtein(ProteinType::output, "contraction");
    g.addRandomProtein(ProteinType::output, "dlcommPlus");
    g.addRandomProtein(ProteinType::output, "dlcommMinus");
    g.addRandomProtein(ProteinType::output, "dgcommPlus");
    g.addRandomProtein(ProteinType::output, "dgcommMinus");
    g.addRandomProtein(ProteinType::output, "thetaPlus");
    g.addRandomProtein(ProteinType::output, "thetaMinus");
    g.addRandomProtein(ProteinType::output, "phiPlus");
    g.addRandomProtein(ProteinType::output, "phiMinus");

    g.randomParams();   // Random Beta & Delta
    g.randomReguls(1);  // start with one regul
    return GRNController(g);
  }

  GRNController(GRN_t g) : grn(g) { reset(); }

  GRNController(const string& s) : grn(s) {}  // unserialize
  std::string serialize() const { return grn.serialize(); }

  void reset() { grn.reset(); }

  void mutate() { grn.mutate(); }

  void update() { grn.step(); }

  void setInput(const std::string &input, double val) {
    grn.setProteinConcentration(input, ProteinType::input, val);
  }

  double getOutput(const std::string &output) const {
    auto r = grn.getProteinConcentration(output, ProteinType::output);
    return r;
  }

  double getDelta(const std::string &o1, const std::string &o2) {
    double plus = getOutput(o1);
    double minus = getOutput(o1);
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
