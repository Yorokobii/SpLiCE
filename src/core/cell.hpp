#ifndef CELL_HPP
#define CELL_HPP
#include <mecacell/mecacell.h>
#include <mecacell/elasticbody.hpp>
#include <mecacell/springbody.hpp>
#include <math.h>
#include <string>

template <typename Controller, typename Config> class Cell
  : public MecaCell::ConnectableCell<Cell<Controller, Config>, MecaCell::SpringBody> {
  bool contracting = false;
  double contractTime = 0.0;

 public:
	using Vec = MecaCell::Vec;
  using Base = MecaCell::ConnectableCell<Cell<Controller, Config>, MecaCell::SpringBody>;
  double originalRadius = 30.0;
  double adhCoef = 25.0;
  double contractRatio = 0.9;
  double contractDuration = 0.4;
  double theta = 0.0;
  double phi = 0.0;
  double usedEnergy = 0.0;
  int age = 0;
  Controller ctrl;
  Config& config;
  std::vector<std::string> action_outputs = {"duplicate", "rotate", "quiescence",
                                        "apoptosis", "contraction"};

  Cell(const Vec& p, Config& cfg): Base(p), ctrl(), config(cfg) {
    init();
  }

  Cell(const Vec& p, const Controller& ct, Config& cfg): Base(p), ctrl(ct), config(cfg) {
    init();
  }

  double getAdhesionWith(Cell*, MecaCell::Vec) { return adhCoef; }

  void init() {
    std::uniform_real_distribution<> dis(0.0, 2 * M_PI);
    theta = dis(MecaCell::Config::globalRand());
    phi = dis(MecaCell::Config::globalRand());
  }

  void startContracting() {
    contractTime = 0.0;
    if (!contracting) {
      contracting = true;
      this->body.setRadius(originalRadius * contractRatio);
    }
  }

  template <typename W> void updateBehavior(W& w) {
    age += 1;
    std::vector<double> actions;
    for (auto& astr : action_outputs) {
      actions.push_back(ctrl.getOutput(astr));
    }
    int actionMax = 0; double actionMaxConc = 0.0;
    for (auto i = 0; i < actions.size(); ++i) {
      if (actions[i] > actionMaxConc) {
        actionMax = i;
        actionMaxConc = actions[i];
      }
    }
    std::string action = action_outputs[actionMax];

    if (action == "duplicate") {
      // cell duplicate
      Vec dpos {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
      Vec child_pos = dpos * config.div_radius + this->getPosition();
      w.addCell(new Cell(child_pos, config));
      age = 0;
      usedEnergy = config.energy_duplicate;
    } else if (action == "rotate") {
      // cell rotate
      double dtheta = ((ctrl.getOutput("theta_plus") - ctrl.getOutput("theta_minus")) /
                       (ctrl.getOutput("theta_plus") + ctrl.getOutput("theta_minus")));
      theta = min(max(theta + dtheta, 0.0), 2 * M_PI);
      double dphi = ((ctrl.getOutput("phi_plus") - ctrl.getOutput("phi_minus")) /
                     (ctrl.getOutput("phi_plus") + ctrl.getOutput("phi_minus")));
      phi = min(max(phi + dphi, 0.0), 2 * M_PI);
      usedEnergy = config.energy_rotate;
    } else if (action == "apoptosis") {
      // cell death
      if (w.cells.size() > 1) { this->die(); }
      usedEnergy = config.energy_apoptosis;
    } else if (action == "contraction") {
      startContracting();
      usedEnergy = config.energy_contraction;
    } else if (action == "quiescence") {
      usedEnergy = config.energy_quiescence;
    }

    if (contracting) {
      contractTime += w.getDt();
      if (contractTime > contractDuration) {
        contracting = false;
        contractTime = 0.0;
        this->body.setRadius(originalRadius);
      }
    }
  }
};
#endif
