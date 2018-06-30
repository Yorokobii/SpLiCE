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
  bool reachedCellSize = false;
  int worldAge = 0;
  ctrl_t controller;
  MecaCell::Vec com = MecaCell::Vec::zero();
  MecaCell::Vec comDevo = MecaCell::Vec::zero();

 protected:
  double currentTime = 0;
  MecaCell::SimplifiedFluidPlugin<cell_t> sfp;  // fluid dynamics

 public:
  Scenario(cfg_t c) : config(c), gen(rd()) {}

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

    world.addCell(new cell_t(MecaCell::Vec::zero(), 0.0, 0.0, controller, config));
    world.update();
  }

  void controllerUpdate() {
    int nconn = 0;
    int maxConn = 0;
    std::uniform_real_distribution<> dis(0.0, 2 * M_PI);
    for (auto& c : world.cells) {
      energy -= c->usedEnergy;
      gcomm += c->dgcomm;
      c->nage = (double) c->age / (double) worldAge;
      nconn = 0;
      MecaCell::Vec pressure = MecaCell::Vec::zero();
      for (auto& con : c->getBody().cellConnections) {
        nconn += 1;
        pressure += con->collision.computeForce(config.dt) * con->direction;
        con->cells.first->lcomm += con->cells.second->dlcomm;
      }
      if (nconn == 0 && world.cells.size() > 1) c->die();
      c->pressure = exp(-pressure.length() / config.betaPressure);
      c->nconn = nconn;
      maxConn = max(nconn, maxConn);
    }
    for (auto& c : world.cells) {
      com += c->getPosition();
    }
    com /= world.cells.size();
    if (!reachedCellSize && (world.cells.size() > config.devCells)) {
      reachedCellSize = true;
      comDevo = com;
    }
    gcomm = min(max(gcomm, 0.0), 1.0);
    for (auto& c : world.cells) {
      c->lcomm = min(max(c->lcomm, 0.0), 1.0);
      c->gcomm = gcomm;
      c->maxConn = maxConn;
      c->energy = energy;
      c->ctrl_update = true;
      if (c->isNew) {
        c->theta = dis(gen);
        c->phi= dis(gen);
        c->isNew = false;
      }
    }
    if (reachedCellSize) {
      MecaCell::Vec movement = comDevo - com;
      fit = movement.length() / config.originalRadius;
    }
    MecaCell::logger<MecaCell::DBG>(":S| ", currentTime, " ", worldAge, " ", energy, " ",
                                    world.cells.size(), " ", gcomm, " ", comDevo, " ",
                                    com, " ", fit, " ");

  }

  void loop() {
    currentTime += world.getDt();
    worldAge += 1;
    world.update();
    if (worldAge % config.controllerUpdate) controllerUpdate();
  }

  world_t& getWorld() { return world; }
  bool finished() { return (currentTime > duration || energy <= 0.0 ||
                            world.cells.size() == 0); }
};

#endif
