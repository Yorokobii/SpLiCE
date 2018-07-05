#ifndef SCENARIO_HPP
#include <mecacell/mecacell.h>
#include <mecacell/utilities/utils.hpp>
#include <mecacell/utilities/obj3D.hpp>
#include <random>
#include <cmath>
#include <math.h>
#include "../simplifiedfluidplugin.hpp"
#include "config.hpp"

template <typename cell_t, typename ctrl_t, typename cfg_t> class Scenario {
 public:
  using world_t = MecaCell::World<cell_t>;
  cfg_t config;
  world_t world;
  std::random_device rd;
  std::mt19937 gen;
  double duration = 0.0;
  double energy = 0.0;
  double gcomm = 0.0;
  double fit = 0.0;
  double shapefit = 0.0;
  int worldAge = 0;
  bool setDevoPhase = false;
  ctrl_t controller;
  MecaCell::Vec com = MecaCell::Vec::zero();
  MecaCell::Vec prevCom = MecaCell::Vec::zero();
  MecaCell::Vec comDevo = MecaCell::Vec::zero();

 protected:
  double currentTime = 0;
  MecaCell::SimplifiedFluidPlugin<cell_t> sfp;  // fluid dynamics

 public:
  Scenario(cfg_t c) : config(c), gen(rd()) {}

  std::vector<MecaCell::Vec> readCellCoordinates(std::string path) {
    std::vector<MecaCell::Vec> res;
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("Unable to open shape file");
    std::string line;
    while (std::getline(file, line)) {
      auto vs = MecaCell::splitStr(line, ' ');
      res.push_back(MecaCell::Vec(stod(vs[0]), stod(vs[1]), stod(vs[2])));
    }
    return res;
  }

  void init() {
    gen.seed(config.seed);
    sfp.fluidDensity = config.fluidDensity;
    sfp.fluidVelocity = {0, 0, 0};
    world.registerPlugins(sfp);
    world.setDt(config.dt);
    duration = config.simDuration;
    energy = config.energyInitial;
    if (config.grnFile != "") {
      std::ifstream file(config.grnFile);
      std::stringstream buffer;
      buffer << file.rdbuf();
      controller = ctrl_t(buffer.str());
    }

    if (config.simShape != "") {
      MecaCell::logger<MecaCell::DBG>("Cell coords :: ", config.simShape);
      auto cellsCoordinates = readCellCoordinates(config.simShape);

      for (const auto& p : cellsCoordinates) {
        cell_t* c = new cell_t(p, 0.0, 0.0, controller, config);
        world.addCell(c);
      }

      MecaCell::logger<MecaCell::DBG>("Added cells");
      world.update();
      world.update();

      for (auto& c : world.cells)
        c->action_outputs = {"duplicate", "rotate", "quiescence", "contraction"};
    } else {
      world.addCell(new cell_t(MecaCell::Vec::zero(), 0.0, 0.0, controller, config));
      world.update();
    }
  }

  void controllerUpdate() {
    int nconn = 0;
    int maxConn = 0;
    size_t ncells = world.cells.size();
    for (auto& c : world.cells) {
      energy -= c->usedEnergy;
      gcomm += c->dgcomm;
      c->nage = (double) c->age / (double) worldAge;
      nconn = c->getBody().cellConnections.size();
      MecaCell::Vec pressure = MecaCell::Vec::zero();
      for (auto& con : c->getBody().cellConnections) {
        pressure += con->collision.computeForce(config.dt) * con->direction;
        con->cells.first->lcomm += con->cells.second->dlcomm;
      }
      c->pressure = exp(-pressure.length() / config.betaPressure);
      c->nconn = nconn;
      maxConn = max(nconn, maxConn);
      com += c->getPosition();
    }
    if (ncells > 0){
      com = com / ncells;
      //gain energy
      energy += (com - prevCom).length()*20;
      prevCom = com;
    }
    gcomm = min(max(gcomm, 0.0), 1.0);
    double maxComDist = 0.0;
    for (auto& c : world.cells) {
      c->comdist = (c->getPosition() - com).length();
      maxComDist = max(maxComDist, c->comdist);
    }
    std::uniform_real_distribution<> dis(-0.2, 0.2);
    for (auto& c : world.cells) {
      if ((c->nconn == 0) && world.cells.size() > 1) c->die();
      c->lcomm = min(max(c->lcomm, 0.0), 1.0);
      c->gcomm = gcomm;
      c->maxConn = maxConn;
      c->energy = energy;
      c->worldAge = worldAge;

      c->deltcom = (com - prevCom).length();

      if (ncells > 1) c->comdist = c->comdist / maxComDist;
      c->ctrl_update = true;
      if (c->isNew) {
        c->theta = min(max(c->theta + dis(gen), 0.0), 2*M_PI);
        c->phi = min(max(c->theta + dis(gen), 0.0), 2*M_PI);
        c->isNew = false;
      }
    }
    if (setDevoPhase) {
      if ((((config.devoSteps > 0) && (ncells > config.devCells))
          || (config.devoSteps == 0))) {
        MecaCell::Vec movement = comDevo - com;
        fit = movement.length() / config.originalRadius;
      }
    }
    int connections_per_cell = 0;
    for (auto& c : world.cells) {
      connections_per_cell += c->nconn;
    }
    connections_per_cell /= world.cells.size();
    shapefit = connections_per_cell*5;

    MecaCell::logger<MecaCell::DBG>(":S| ", currentTime, " ", worldAge, " ", energy, " ",
                                    world.cells.size(), " ", gcomm, " ", comDevo, " ",
                                    com, " ", fit, " ");

  }

  void loop() {
    currentTime += world.getDt();
    worldAge += 1;
    if (!setDevoPhase) {
      if(worldAge > config.devoSteps){
        setDevoPhase = true;
      }
      else{
        com = MecaCell::Vec::zero();
        for (auto& c : world.cells) {
          com += c->getPosition();
        }
        if (world.cells.size() > 0) com = com / world.cells.size();
        comDevo = com;
        prevCom = com;
      }
      for (auto& c : world.cells) {
        c->getBody().setAngularVelocity(MecaCell::Vec::zero());
        c->getBody().setTorque(MecaCell::Vec::zero());
        c->getBody().setVelocity(MecaCell::Vec::zero());
        for (auto &conn : c->getBody().cellConnections) {
          conn->unbreakable = true;
          conn->adhCoef = c->adhCoef;
        }
        if(worldAge > config.devoSteps){
          c->devoPhase = false;
          c->adhCoef = 0.0;
          c->action_outputs = {"quiescence", "contraction"};
        }
        if(c->nconn > 5)
          c->action_outputs = {"quiescence"};
      }
    }
    if (worldAge % config.controllerUpdate == 0) controllerUpdate();
    world.update();
  }

  world_t& getWorld() { return world; }
  bool finished() { return (currentTime > duration || energy <= 0.0 ||
                            world.cells.size() == 0); }
};

#endif
