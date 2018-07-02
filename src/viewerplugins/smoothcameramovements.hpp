#ifndef SMOOTHCAMERAMOVEMENTS_MECACELLVIEWERPLUGIN_HPP
#define SMOOTHCAMERAMOVEMENTS_MECACELLVIEWERPLUGIN_HPP
#include <mecacell/mecacell.h>
#include <mecacellviewer/viewer.h>

using namespace MecacellViewer;

struct SmoothMoveAroundTargetPlugin {
	double raideurContrainte = 12.0;
	double desiredDistance = 3200.0;
	QVector3D rotationForce = {1000.0, 3000.0, 0.0};
	QVector3D centerOfMass = QVector3D(0, 0, 0);

	template <typename R> void onLoad(R* r) { r->getCamera().setMode(Camera::centered); }

	template <typename R> void preLoop(R* r) {
		centerOfMass = QVector3D(0, 0, 0);
		for (auto& c : r->getScenario().getWorld().cells)
			centerOfMass += toQV3D(c->getPosition());
		centerOfMass =
		    centerOfMass / static_cast<double>(r->getScenario().getWorld().cells.size());

		r->getCamera().setTarget(centerOfMass);

		double X = desiredDistance -
		           (r->getCamera().getPosition() - r->getCamera().getTarget()).length();
		
		// MecaCell::logger<MecaCell::DBG>(std::to_string(r->getCamera().getSpeed().length()));
		if(r->getCamera().getSpeed().length() < 301)
			r->getCamera().setForce(r->getCamera().getForce() + QVector3D(0.0, 0.0, raideurContrainte * X) + rotationForce);
	}
};

#endif
