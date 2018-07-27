#ifndef SINCELL_CONTROLLER_HPP
#define SINCELL_CONTROLLER_HPP
#include <mecacell/mecacell.h>
#include <random>
#include <string>

struct SinController {
 public:
  double* step;
  SinController() {
    step = new double;
    *step = 0.0;
  }

  static SinController random() {
    return SinController();
  }

  void setInput(const std::string &input, double val) {
    MecaCell::logger<MecaCell::DBG>("input: ", input, " ", val);
  }

  double getOutput(const std::string &output, const int& step = 0, const int& rank = 0, const int& maxrank = 0) const {
    double r = 0.0;
    if ((sin((step/50)/*-(2*M_PI*rank/maxrank)*/) > 0.5 && output == "contraction")
        /*|| (sin(step/100) <= 0 && output == "extension")*/)
        r = 1.0;
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

  SinController(const SinController &c) {
    // MecaCell::logger<MecaCell::DBG>("cloning");
  }

  SinController(const string& s) {}  // unserialize
  std::string serialize() const { return ""; }

  void reset() {}


  void update() {}

  void mutate() {}

  SinController crossover(const SinController& other) {
    return SinController();
  }
  inline static double getDistance(const SinController& a, const SinController& b) {
    return 0.0;
  }

};

#endif
