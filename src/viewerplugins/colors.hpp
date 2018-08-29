#ifndef PLUGIN_COLORS_HPP
#define PLUGIN_COLORS_HPP

#include <math.h>
#include <array>

using namespace MecacellViewer;

struct ColorModePlugins {
	struct ChangeColorPlug {  // we need a core plugin to get the correct color updates
		bool enabled = false;
		template <typename W> void postBehaviorUpdate(W* w) {
			if (enabled) {
				std::array<std::pair<double, double>, 3> MinMaxSM;
				for (auto& c : w->cells)
					for(int i = 0; i < 3;  ++i){
						MinMaxSM[i].first = min(MinMaxSM[i].first, c->sm[i]);
						MinMaxSM[i].second = max(MinMaxSM[i].second, c->sm[i]);
					}

				for (auto& c : w->cells) {
					std::array<double, 3> sm{};
					for(int i = 0; i < 3; ++i)
						sm[i] = (c->sm[i] - MinMaxSM[i].first)/(MinMaxSM[i].second - MinMaxSM[i].first);
					
					c->setColorRGB(sm[0]*200, sm[1]*200, sm[2]*200);
				}
			}
		}
	};
	ChangeColorPlug ccp;
	template <typename R> void onLoad(R* renderer) {
		renderer->getScenario().getWorld().registerPlugins(ccp);

		MenuElement<R>* nativeDisplayMenu = renderer->getDisplayMenu();
		MenuElement<R> color = {"Cell coloration",
		                        elementType::exclusiveGroup,
		                        {
		                            {"Red", false}, {"Blue", false}, {"Morphogens", true},
		                        }};

		color.onToggled = [&](auto* r, auto* me) {
			if (me->at("Red").isChecked()) {
				for (auto& c : r->getScenario().getWorld().cells) c->setColorRGB(200, 10, 80);
			} else if (me->at("Blue").isChecked()) {
				for (auto& c : r->getScenario().getWorld().cells) c->setColorRGB(10, 160, 200);
			}
			if (me->at("Morphogens").isChecked())
				ccp.enabled = true;
			else
				ccp.enabled = false;
		};
		nativeDisplayMenu->at("Cells").add(color);
	}
};

#endif
