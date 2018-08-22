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


 public:
	using Vec = MecaCell::Vec;
  using Base = MecaCell::ConnectableCell<Cell<Controller, Config>,
                                         MecaCell::SpringBody>;
  static const unsigned int NB_MORPHOGENS = 3;
  using morphogrid =
    std::vector<std::array<std::pair<MecaCell::Vec, double>, NB_MORPHOGENS>>;
  bool contracting = false;
  double contractTime = 0.0;
  double contractDuration = 0.0;

  //Mophogenesis related
  std::array<double, NB_MORPHOGENS> morphogensProduction{};

  //Graph related
  bool visited = false;
  bool root = false;

  double surroundContract = 0.0;
  int contractionCount = 0;
  int extensionCount = 0;
  double adhCoef = 0.0;
  double theta = 0.0;
  double phi = 0.0;
  double usedEnergy = 0.0;
  double pressure = 0.0;
  double energy = 0.0;
  double comdist = 0.0;
  double deltcom = 0.0;
  int nconn = 0;
  int ncells = 0;
  int age = 0;
  int worldAge = 0;
  double nage = 0.0;
  double contractForce = 1.0;

  // Vec* com = NULL;
  // Vec* prevCom = NULL;

  bool ctrl_update = false;
  bool isNew = true;
  bool isDuplicated = true;
  Controller ctrl;
  Config& config;
  std::vector<std::string> action_outputs = {"quiescence", "duplicate", "rotate", "contraction", "extension"};
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
    ctrl.setInput("theta", theta / (2 * M_PI));
    ctrl.setInput("phi", phi / (2 * M_PI));
    // ctrl.setInput("pressure", exp(-pressure / config.betaPressure));
    ctrl.setInput("pressure", pressure);
    ctrl.setInput("energy", max((energy / config.energyInitial), 0.0));
    ctrl.setInput("comdist", comdist);
    ctrl.setInput("contracting", (double)contracting);
    ctrl.setInput("deltcom", exp(deltcom / 10));
    ctrl.setInput("ncells", (double)ncells);
    ctrl.setInput("isDuplicated", (double)isDuplicated);
    ctrl.setInput("surroundContract", surroundContract);
    ctrl.setInput("contractForceInput", contractForce);
    ctrl.setInput("velocity", this->getBody().getVelocity().length());
    //debug
    for(auto i = 0u; i < NB_MORPHOGENS; ++i)
      ctrl.getInput("inputMorphogen" + std::to_string(i), config.verbosity);
  }

  template <typename W> void updateOuputs(W& w) {

    //update morphogens output
    for(auto i = 0u; i < NB_MORPHOGENS; ++i){
      auto o = ctrl.getOutput(std::string("outputMorphogen") + std::to_string(i), config.verbosity);
      morphogensProduction[i] = o > 0.0 ? o : 0.0;
    }

    //update contraction force
    contractForce = ctrl.getOutput(std::string("contractForceOutput"), config.verbosity);

    //set forced dev
    if(config.simShape == ""){
      if(ncells < config.minCells || isDuplicated){
        action_outputs = {"quiescence", "duplicate", "rotate"};
      }
      else{
        action_outputs = {"quiescence", "duplicate", "rotate", "contraction", "extension"};
      }
    }
    else
      action_outputs = {"quiescence", "rotate", "contraction", "extension"};
    //set bone-like 
    if(nconn>8){
      action_outputs = {"quiescence"};
    }

    std::vector<double> actions;
    for (auto& astr : action_outputs) {
      actions.push_back(ctrl.getOutput(astr, config.verbosity));
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
          double force = conn->direction.dot(dpos) * ((contractForce *
                                                    (config.maxContractForce - config.minContractForce)) +
                                                    config.minContractForce);
          conn->cells.first->getBody().receiveForce(force, conn->direction,
                                                    config.compressForce);
          conn->cells.second->getBody().receiveForce(force, -conn->direction,
                                                    config.compressForce);
        }
      }
      usedEnergy = config.energyContraction * contractForce;

    } else if (action == "extension") {

      Vec dpos {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
      for (auto &conn : this->getBody().cellConnections) {
        if (conn->unbreakable) {
          double force = conn->direction.dot(dpos) * ((contractForce *
                                                    (config.maxContractForce - config.minContractForce)) +
                                                    config.minContractForce);
          conn->cells.first->getBody().receiveForce(force, -conn->direction,
                                                    config.compressForce);
          conn->cells.second->getBody().receiveForce(force, conn->direction,
                                                    config.compressForce);
        }
      }
      extensionCount++;
      usedEnergy = config.energyContraction * contractForce;

    } else if (action == "quiescence") {
      // do nothing
      usedEnergy = config.energyQuiescence;
    } else if (action == "apoptosis") {
      if(!this->root)
        this->die();
    }
    ctrl_update = false;
  }

  template <typename W> void updateSensedMorphogens(W& w, const morphogrid& morphogens){
    for (auto i = 0u; i < NB_MORPHOGENS; ++i) {
      double sensedMorphogens = 0.0;
      //for each cells in the morphogrid
      for(const auto& c : morphogens){
        auto l = (c[i].first - this->getPosition()).length() / config.diffusionCoeff;
        sensedMorphogens += c[i].second / (l + 1.0);
      }
      ctrl.setInput(std::string("inputMorphogen") + std::to_string(i), sensedMorphogens);
    }
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
    if (/*!isNew && */ctrl_update) {
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
