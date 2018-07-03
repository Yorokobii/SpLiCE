#ifndef AUTOSTOP_HPP
#define AUTOSTOP_HPP
#include <mecacell/mecacell.h>
#include <mecacellviewer/viewer.h>

using namespace MecacellViewer;

struct AutoStop {
	template <typename R> void preLoop(R* r) {
        if(r->getScenario().finished())
	        r->getEngine()->quit();
    }
};

#endif //AUTOSTOP_HPP