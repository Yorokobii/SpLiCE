#ifndef CELL_CONTROLLER_HPP
#define CELL_CONTROLLER_HPP
#include <mecacell/mecacell.h>
#include <random>
#include <string>

struct BaseController {
 public:
	BaseController() {}

	void setInput(const std::string &input, double val) {}

	double getOutput(const std::string &output) const {
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

  void update() {}
};

#endif
