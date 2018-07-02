#ifndef AUTOSTOP_HPP
#define AUTOSTOP_HPP
#include <mecacell/mecacell.h>
#include <mecacellviewer/viewer.h>

using namespace MecacellViewer;

struct AutoStop {
	template <typename R> void onLoad(R* r) {
        if(r->getWorld()->getNbUpdates() > 1000)
	     r->getEngine()->quit();
    }
};

#endif //AUTOSTOP_HPP