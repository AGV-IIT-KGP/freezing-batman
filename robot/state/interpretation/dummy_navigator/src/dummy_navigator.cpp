#include "dummy_navigator/dummy_navigator.h"

namespace navigation_space {
    
    navigation::State navigation_space::DummyNavigator::getBotLocation() {
        navigation::State bot_location;
        int x,y,z;
        x = 0.5 * MAP_MAX;
        y = 0.1 * MAP_MAX;
        z = 90;
        bot_location = navigation::State(x,y,z,0);
        return bot_location;
    }

    navigation::State navigation_space::DummyNavigator::getTargetLocation(){
    	navigation::State target_location;
    	int x,y,z;
    	x=0.5 * MAP_MAX;
    	y=0.975 * MAP_MAX;
    	z=90;
    	target_location = navigation::State(x,y,z,0);
    	return target_location;
    }
}