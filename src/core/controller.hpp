#ifndef CELL_CONTROLLER_HPP
#define CELL_CONTROLLER_HPP
#include <mecacell/mecacell.h>
#include <random>
#include <string>

struct BaseController {
 public:
  double state = 0.0;
	BaseController() {}

  static BaseController random() {
    return BaseController();
  }

	void setInput(const std::string &input, double val) {}

	double getOutput(const std::string &output) const {
    if (output == "contract") return state;
		std::uniform_real_distribution<> dis(0.0, 1.0);
		return dis(MecaCell::Config::globalRand());
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

  BaseController(const string& s) {}  // unserialize
  std::string serialize() const { return ""; }

  void reset() {}

  void update() {
    state += 0.1;
    if (state > 1.0) state = 0.0;
  }

  void mutate() {}

  BaseController crossover(const BaseController& other) {
    return BaseController();
  }
  inline static double getDistance(const BaseController& a, const BaseController& b) {
    return 0.0;
  }

};

#endif
