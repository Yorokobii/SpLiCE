#ifndef CELL_HPP
#define CELL_HPP
#include <mecacell/mecacell.h>
#include <mecacell/elasticbody.hpp>
#include <mecacell/springbody.hpp>
#include <mecacell/contactsurfacebody.hpp>
#include <math.h>
#include <string>

template <typename Controller, typename Config> class Cell
  : public MecaCell::ConnectableCell<Cell<Controller, Config>, MecaCell::SpringBody> {
  bool contracting = false;
  double contractTime = 0.0;
  double contractDuration = 0.0;

 public:
	using Vec = MecaCell::Vec;
  using Base = MecaCell::ConnectableCell<Cell<Controller, Config>,
                                         MecaCell::SpringBody>;

  //Graph related
  bool visited = false;
  bool root = false;

  int contractionCount = 0;
  double adhCoef = 0.0;
  double theta = 0.0;
  double phi = 0.0;
  double usedEnergy = 0.0;
  double gcomm = 0.0;
  double dgcomm = 0.0;
  double lcomm = 0.0;
  double dlcomm = 0.0;
  double pressure = 0.0;
  double energy = 0.0;
  double comdist = 0.0;
  double deltcom = 0.0;
  int nconn = 0;
  int ncells = 0;
  int age = 0;
  int worldAge = 0;
  double nage = 0.0;

  // Vec* com = NULL;
  // Vec* prevCom = NULL;
  bool duplicated = false;

  bool ctrl_update = false;
  bool isNew = true;
  bool isDuplicated = true;
  Controller ctrl;
  Config& config;
  std::vector<std::string> action_outputs = {"quiescence", "duplicate", "rotate", "contraction"};
  Cell(const Vec& p, double th, double ph, const Controller& ct, Config& cfg, bool _root = false)
    : Base(p), theta(th), phi(ph), ctrl(ct), config(cfg), root(_root) {
    this->getBody().setRadius(config.originalRadius);
    this->getBody().setStiffness(config.cellStiffness);
    this->getBody().setMass(config.cellMass);
    adhCoef = config.adhCoef;
    contractDuration = config.contractDuration;
  }

  double getAdhesionWith(Cell* c, MecaCell::Vec) { return (this->isDuplicated || c->isDuplicated) ? adhCoef : 0.0; }

  template <typename W> void updateInputs(W& w) {
    ctrl.setInput("gcomm", gcomm);
    ctrl.setInput("lcomm", lcomm);
    ctrl.setInput("theta", theta / (2 * M_PI));
    ctrl.setInput("phi", phi / (2 * M_PI));
    ctrl.setInput("pressure", exp(-pressure / config.betaPressure));
    ctrl.setInput("energy", max((energy / config.energyInitial), 0.0));
    ctrl.setInput("comdist", comdist);
    ctrl.setInput("contracting", (double)contracting);
    ctrl.setInput("deltcom", exp(deltcom / 10));
    ctrl.setInput("ncells", (double)ncells);
    ctrl.setInput("isDuplicated", (double)isDuplicated);
    ctrl.setInput("stiffness", (double)this->getBody().stiffness);
  }

  template <typename W> void updateOuputs(W& w) {
    dlcomm = ctrl.getDelta("dlcommPlus", "dlcommMinus");
    dgcomm = ctrl.getDelta("dgcommPlus", "dgcommMinus");

    if(duplicated) duplicated = false;

    //set forced dev
    if((ncells < config.minCells || isDuplicated) && config.simShape == ""){
      action_outputs = {"quiescence", "duplicate", "rotate"};
    }
    else{
      action_outputs = {"quiescence", "duplicate", "rotate", "contraction"};
    }
    //set bone-like 
    // if(nconn>7){
    //   action_outputs = {"quiescence"};
    //   this->getBody().stiffness = 200;

    // }

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
    contracting = false;

    if (action == "duplicate") {
      if (energy >= config.energyDuplicate
          && (config.maxCells != 0 ? w.cells.size() < config.maxCells : true)
          && nconn < 7
          && config.simShape == "") {
        // cell duplicate
        Vec dpos {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
        w.addCell(new Cell(this->getPosition() + dpos * config.divRadius, theta, phi, ctrl, config));

        duplicated = true;

        usedEnergy = config.energyDuplicate;
      }
    } else if (action == "rotate") {
      // cell rotate
      double dtheta = ctrl.getDelta("thetaPlus", "thetaMinus");
      theta = min(max(theta + dtheta, 0.0), 2 * M_PI);
      double dphi = ctrl.getDelta("phiPlus", "phiMinus");
      phi = min(max(phi + dphi, 0.0), 2 * M_PI);
      usedEnergy = config.energyRotate;
    } else if (action == "contraction") {

      Vec dpos {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
      contractionCount++;
      contracting = true;
      for (auto &conn : this->getBody().cellConnections) {
        if (conn->unbreakable) {
          double force = conn->direction.dot(dpos) * config.force;
          conn->cells.first->getBody().receiveForce(force, conn->direction,
                                                    config.compressForce);
          conn->cells.second->getBody().receiveForce(force, -conn->direction,
                                                    config.compressForce);
        }
      }
      //more energy used for smaller systems
      // usedEnergy = (config.energyInitial/config.energyDuplicate)/ncells * config.energyContraction;
      usedEnergy = config.energyContraction;

      // if(contracting == false){
      //   // start a new contraction event
      //   contracting = true;
      //   Vec dpos {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
      //   contractionCount++;
      //   for (auto &conn : this->getBody().cellConnections) {
      //     if (conn->unbreakable) {
      //       double force = conn->direction.dot(dpos) * config.force;
      //       conn->cells.first->getBody().receiveForce(force, conn->direction,
      //                                                 config.compressForce);
      //       conn->cells.second->getBody().receiveForce(force, -conn->direction,
      //                                                 config.compressForce);
      //     }
      //   }
      //   //more energy used for smaller systems
      //   // usedEnergy = (config.energyInitial/config.energyDuplicate)/ncells * config.energyContraction;
      //   usedEnergy = config.energyContraction;
      // }
      // else{
      //   if(contractTime >= contractDuration){
      //     contractTime = 0.0;
      //     contracting = false;
      //   }
      //   else{
      //     if(contractTime <= contractDuration/4){
      //       for (auto &conn : this->getBody().cellConnections) {
      //         if (conn->unbreakable) {
      //           conn->cells.first->getBody().receiveForce(config.force, conn->direction,
      //                                                     config.compressForce);
      //           conn->cells.second->getBody().receiveForce(config.force, -conn->direction,
      //                                                     config.compressForce);
      //         }
      //       }
      //     }
      //     contractTime++;
      //   }
      // }
    } else if (action == "quiescence") {
      // do nothing
      usedEnergy = config.energyQuiescence;
    } else if (action == "apoptosis") {
      if(!this->root)
        this->die();
    }
    ctrl_update = false;
  }

  template <typename W> void updateBehavior(W& w) {
    //passed a certain age the cell is not new anymore
    if(age < config.newAge){
      for(auto& c : this->getBody().cellConnections){
        c->unbreakable = true;
      }
    }
    else{
      isDuplicated = false;
    }
    if (!isNew && ctrl_update) {
      // set inputs
      updateInputs(w);
      // call the controller
      ctrl.update();
      // get outputs and control cell
      age += 1;
      usedEnergy = config.energyQuiescence;
      updateOuputs(w);
    }
  }
};
#endif
