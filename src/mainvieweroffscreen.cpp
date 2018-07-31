#define MECACELL_TERMINAL_COLORS
#define MECACELL_LOGGER_WARN_DISABLE
#include <mecacell/mecacell.h>
#include <mecacellviewer/viewer.h>
#include <mecacellviewer/plugins/meshviewerplugin.hpp>
#include "core/config.hpp"
#include "viewerplugins/camera.hpp"
#include "viewerplugins/colors.hpp"
#include "viewerplugins/complugin.hpp"
#include "viewerplugins/grid.hpp"
#include "viewerplugins/screencapture.hpp"
#include "viewerplugins/autostop.hpp"
#include "viewerplugins/autostart.hpp"
#include "viewerplugins/offscreen.hpp"

int main(int argc, char** argv) {
  Config cfg(argc, argv);
  Config::scenario_t scenario(cfg);

  scenario.controller = Config::CtrlType::random();
  scenario.init();

  ColorModePlugins cmp;
  CenterOfMassPlugin comp;
  CenteredCameraPlugin ccp;
  GridViewerPlugin gvp;
  PointViewerPlugin pvp;
  AutoStart astart;
  AutoStop astop;
  OffscreenPlugin offs;
  MecacellViewer::Viewer<Config::scenario_t> v(scenario);
  v.setNbLoopsPerFrame(2);
  v.registerPlugins(cmp, gvp, ccp, comp, astart, astop, offs);
  // v.registerPlugins(cmp, gvp, ccp, comp, scp, astop);

  return v.exec();
}
