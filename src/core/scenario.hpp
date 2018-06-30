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
  cfg_t& config;
  world_t world;
  std::random_device rd;
  std::mt19937 gen;
  double duration = 0.0;
  double energy = 0.0;
  double gcomm = 0.0;
  int worldAge = 0;

 protected:
  double currentTime = 0;
  MecaCell::SimplifiedFluidPlugin<cell_t> sfp;  // fluid dynamics

 public:
  Scenario(cfg_t& c) : config(c), gen(rd()) {}

  void init() {
    gen.seed(config.seed);
    sfp.fluidDensity = config.fluidDensity;
    sfp.fluidVelocity = {0, 0, 0};
    world.registerPlugins(sfp);
    world.setDt(config.dt);
    duration = config.simDuration;
    energy = config.energyInitial;

    world.addCell(new cell_t(MecaCell::Vec::zero(), 0.0, 0.0, ctrl_t(), config));
    world.update();
    MecaCell::logger<MecaCell::DBG>("Done initializing scenario");
  }

  void controllerUpdate() {
 		// for (auto& conn : world.cellPlugin.connections) {
    //   conn.cells.first.lcomm += conn.cells.second.dlcomm;
    //   conn.cells.second.lcomm += conn.cells.first.dlcomm;
    // }
    double maxPressure = 0.0;
    int nconn = 0;
    for (auto& c : world.cells) {
      energy -= c->usedEnergy;
      gcomm += c->dgcomm;
      c->nage = c->age / worldAge;
      maxPressure = max(c->getBody().getPressure(), maxPressure);
      nconn = 0;
      for (auto& d : world.cells) {
        if (c->isConnectedTo(d)) {
          c->lcomm += d->dlcomm;
          nconn += 1;
        }
      }
      if (nconn == 0 && world.cells.size() > 1) c->die();
    }
    gcomm = min(max(gcomm, 0.0), 1.0);
    for (auto& c : world.cells) {
      c->lcomm = min(max(c->lcomm, 0.0), 1.0);
      c->gcomm = gcomm;
      c->maxPressure = maxPressure;
      c->ctrl_update = true;
    }
  }

  void loop() {
    currentTime += world.getDt();
    world.update();
    if (worldAge % config.controllerUpdate) controllerUpdate();
    worldAge += 1;
    MecaCell::logger<MecaCell::DBG>(currentTime, " ", worldAge, " ", energy, " ", gcomm);
  }

  world_t& getWorld() { return world; }
  bool finished() { return (currentTime > duration || energy <= 0.0 ||
                            world.cells.size() == 0); }
};

#endif
