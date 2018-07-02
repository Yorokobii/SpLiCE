#ifndef AUTOSTART_HPP
#define AUTOSTART_HPP
#include <mecacell/mecacell.h>
#include <mecacellviewer/viewer.h>

using namespace MecacellViewer;

struct AutoStart {
	template <typename R> void preLoad(R* r) {
        if(r->ssb)
            r->ssb->setWorldUpdate(true);
    }
};

#endif //AUTOSTART_HPP