#ifndef CELL_HPP
#define CELL_HPP
#include <mecacell/mecacell.h>
#include <mecacell/elasticbody.hpp>
#include <mecacell/springbody.hpp>
#include <mecacell/contactsurfacebody.hpp>
#include <math.h>
#include <string>

template <typename Controller, typename Config> class Cell
  : public MecaCell::ConnectableCell<Cell<Controller, Config>, MecaCell::ContactSurfaceBody> {
  bool contracting = false;
  double contractTime = 0.0;

 public:
	using Vec = MecaCell::Vec;
  using Base = MecaCell::ConnectableCell<Cell<Controller, Config>, MecaCell::ContactSurfaceBody>;
  double originalRadius = 30.0;
  double adhCoef = 25.0;
  double contractRatio = 0.9;
  double contractDuration = 0.4;
  double theta = 0.0;
  double phi = 0.0;
  double usedEnergy = 0.0;
  double gcomm = 0.0;
  double dgcomm = 0.0;
  double lcomm = 0.0;
  double dlcomm = 0.0;
  double maxPressure = 0.0;
  int age = 0;
  double nage = 0.0;
  bool ctrl_update = false;
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
      this->getBody().setRadius(originalRadius * contractRatio);
    }
  }

  template <typename W> void updateInputs(W& w) {
    ctrl.setInput("age", min(exp(-age / config.betaAge), 1.0));
    ctrl.setInput("gcomm", gcomm);
    ctrl.setInput("lcomm", lcomm);
    ctrl.setInput("theta", theta / (2 * M_PI));
    ctrl.setInput("phi", phi / (2 * M_PI));
		ctrl.setInput("pressure", this->getBody().getPressure() / maxPressure);
  }

  template <typename W> void updateOuputs(W& w) {
    dlcomm = ((ctrl.getOutput("dlcomm_plus") - ctrl.getOutput("dlcomm_minus")) /
              (ctrl.getOutput("dlcomm_plus") + ctrl.getOutput("dlcomm_minus")));
    dgcomm = ((ctrl.getOutput("dgcomm_plus") - ctrl.getOutput("dgcomm_minus")) /
              (ctrl.getOutput("dgcomm_plus") + ctrl.getOutput("dgcomm_minus")));
    if (age > config.min_action_age) {
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
        if (w.cells.size() > 5) { this->die(); }
        usedEnergy = config.energy_apoptosis;
      } else if (action == "contraction") {
        // start a new contraction event
        startContracting();
        usedEnergy = config.energy_contraction;
      } else if (action == "quiescence") {
        // do nothing
        usedEnergy = config.energy_quiescence;
      }
      ctrl_update = false;
    }
  }

  template <typename W> void updateBehavior(W& w) {
    if (ctrl_update) {
      // set inputs
      updateInputs(w);
      // call the controller
      ctrl.update();
      // get outputs and control cell
      age += 1;
      usedEnergy = config.energy_quiescence;
      updateOuputs(w);
    }

    // contractions
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
