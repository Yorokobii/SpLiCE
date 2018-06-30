#pragma once
#include <grgen/grn.hpp>
#include <grgen/real.hpp>

struct GRNController {  // both controller and DNA
  using GRN_t = GRN<RealCoords>;
  GRN_t grn;
  GRNController() {
    grn.addRandomProtein(ProteinType::input, "age");
    grn.addRandomProtein(ProteinType::input, "gcomm");
    grn.addRandomProtein(ProteinType::input, "lcomm");
    grn.addRandomProtein(ProteinType::input, "theta");
    grn.addRandomProtein(ProteinType::input, "phi");
    grn.addRandomProtein(ProteinType::input, "conns");

    grn.addRandomProtein(ProteinType::output, "duplicate");
    grn.addRandomProtein(ProteinType::output, "rotate");
    grn.addRandomProtein(ProteinType::output, "quiescence");
    grn.addRandomProtein(ProteinType::output, "contraction");
    grn.addRandomProtein(ProteinType::output, "dlcommPlus");
    grn.addRandomProtein(ProteinType::output, "dlcommMinus");
    grn.addRandomProtein(ProteinType::output, "dgcommPlus");
    grn.addRandomProtein(ProteinType::output, "dgcommMinus");
    grn.addRandomProtein(ProteinType::output, "thetaPlus");
    grn.addRandomProtein(ProteinType::output, "thetaMinus");
    grn.addRandomProtein(ProteinType::output, "phiPlus");
    grn.addRandomProtein(ProteinType::output, "phiMinus");

    grn.randomParams();   // Random Beta & Delta
    grn.randomReguls(1);  // start with one regul
    reset();
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
