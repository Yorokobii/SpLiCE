#ifndef SCENARIO_HPP
#include <mecacell/mecacell.h>
#include <mecacell/utilities/utils.hpp>
#include <mecacell/utilities/obj3D.hpp>
#include <random>
#include <cmath>
#include <math.h>
#include "../simplifiedfluidplugin.hpp"
#include "config.hpp"

template <typename cell_t, typename config_t> class Scenario;

template <typename cell_t, typename config_t> class Scenario {
 public:
  using world_t = MecaCell::World<cell_t>;
  config_t& config;
  world_t world;
  std::random_device rd;
  std::mt19937 gen;
  double duration = 0.0;
  double energy = 0.0;

 protected:
  double currentTime = 0;
  MecaCell::SimplifiedFluidPlugin<cell_t> sfp;  // fluid dynamics

 public:
  Scenario(config_t& c) : config(c), gen(rd()) {
    energy = config.energy_initial;
  }

  void init() {
    gen.seed(config.seed);
    sfp.fluidDensity = config.fluid_density;
    sfp.fluidVelocity = {0, 0, 0};
    world.registerPlugins(sfp);
    world.setDt(config.sim_dt);
    duration = config.sim_duration;

    world.addCell(new cell_t(MecaCell::Vec::zero(), config));
    world.update();
    MecaCell::logger<MecaCell::DBG>("Done initializing scenario");
  }

  void loop() {
    currentTime += world.getDt();
    world.update();
    for (auto& c : world.cells) {
      energy -= c->usedEnergy;
    }
    MecaCell::logger<MecaCell::DBG>(energy);
  }

  world_t& getWorld() { return world; }
  bool finished() { return (currentTime > duration || energy <= 0.0 ||
                            world.cells.size() == 0); }
};

#endif
