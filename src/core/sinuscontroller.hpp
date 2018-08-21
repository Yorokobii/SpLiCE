#ifndef SINCELL_CONTROLLER_HPP
#define SINCELL_CONTROLLER_HPP
#include <mecacell/mecacell.h>
#include <random>
#include <string>

struct SinController {
 public:
  SinController() {}

  static SinController random() {
    return SinController();
  }

  double getInput(const std::string &input, const int& verb = 1) const {
    return 0.0;
  }

  void setInput(const std::string &input, double val) {
    MecaCell::logger<MecaCell::DBG>("input: ", input, " ", val);
  }

  double getOutput(const std::string &output, const int& verb = 1, const int& step = 0) const {
    double r = 0.0;
    if ((sin((step/50)) > 0.5 && output == "contract")
        /*|| (sin(step/100) <= 0 && output == "extension")*/)
        r = 1.0;
    if(verb >= 3) MecaCell::logger<MecaCell::DBG>("output: ", output, " ", r);
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
