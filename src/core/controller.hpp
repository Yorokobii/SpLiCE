#ifndef CELL_CONTROLLER_HPP
#define CELL_CONTROLLER_HPP
#include <mecacell/mecacell.h>
#include <random>
#include <string>

struct BaseController {
 public:
  BaseController() {}

  static BaseController random() {
    return BaseController();
  }

  void setInput(const std::string &input, double val, const int& verb = 1) {
    MecaCell::logger<MecaCell::DBG>("input: ", input, " ", val);
  }

  double getOutput(const std::string &output, const int& verb = 1) const {
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double r = dis(MecaCell::Config::globalRand());
    // if (output == "contraction") r *= 0.2;
    return r;
  }

  double getInput(const std::string &input, const int& verb = 1) const {
    return 0.0;
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

  BaseController(const BaseController &c) {
    // MecaCell::logger<MecaCell::DBG>("cloning");
  }

  BaseController(const string& s) {}  // unserialize
  std::string serialize() const { return ""; }

  void reset() {}

  void update() {}

  void mutate() {}

  BaseController crossover(const BaseController& other) {
    return BaseController();
  }
  inline static double getDistance(const BaseController& a, const BaseController& b) {
    return 0.0;
  }

};

#endif
