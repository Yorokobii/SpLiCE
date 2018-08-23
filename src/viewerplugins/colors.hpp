#ifndef PLUGIN_COLORS_HPP
#define PLUGIN_COLORS_HPP

using namespace MecacellViewer;

struct ColorModePlugins {
	struct ChangeColorPlug {  // we need a core plugin to get the correct color updates
		bool enabled = false;
		template <typename W> void postBehaviorUpdate(W* w) {
			if (enabled) {
				for (auto& c : w->cells) {
					//double d = sqrt(c->sm[0]*c->sm[0] + c->sm[1]*c->sm[1] + c->sm[2]*c->sm[2]);
					c->setColorRGB(c->sm[0]*200, c->sm[1]*200, c->sm[2]*200);
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
		                            {"Red", true}, {"Blue", false}, {"Morphogens", false},
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
