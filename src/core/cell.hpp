#ifndef CELL_HPP
#define CELL_HPP
#include <mecacell/mecacell.h>
#include <mecacell/elasticbody.hpp>
#include <mecacell/springbody.hpp>

template <typename Controller> class Cell
  : public MecaCell::ConnectableCell<Cell<Controller>, MecaCell::SpringBody> {
  bool contracting = false;
  double contractTime = 0.0;

 public:
	using Vec = MecaCell::Vec;
  using Base = MecaCell::ConnectableCell<Cell<Controller>, MecaCell::SpringBody>;
  double originalRadius = 30.0;
  double adhCoef = 25.0;
  double contractRatio = 0.9;
  double contractDuration = 0.4;
  double theta = 0.0;
  double phi = 0.0;
  int age = 0;
  double energy = 100.0;
  Controller ctrl;

  Cell(const Vec& p): Base(p), ctrl() {}

  Cell(const Vec& p, const Controller& ct, double e): Base(p), ctrl(ct) {
    energy = e;
  }

  double getAdhesionWith(Cell*, MecaCell::Vec) { return adhCoef; }

  void startContracting() {
    contractTime = 0.0;
    if (!contracting) {
      contracting = true;
      this->body.setRadius(originalRadius * contractRatio);
    }
  }

  template <typename W> void updateBehavior(W& w) {
    double output = ctrl.getOutput("contract");
    if (output < 0.5) {
        startContracting();
    }
    if (contracting) {
      contractTime += w.getDt();
      if (contractTime > contractDuration) {
        contracting = false;
        contractTime = 0.0;
        this->body.setRadius(originalRadius);
      }
    }
    age += 1;
  }
};
#endif
