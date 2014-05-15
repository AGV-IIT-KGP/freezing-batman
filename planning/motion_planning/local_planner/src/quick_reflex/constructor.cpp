#include "quick_reflex/quick_reflex.hpp"

#include "ros/package.h"

namespace navigation {

    quickReflex::quickReflex() : SEEDS_FILE(ros::package::getPath("local_planner")+"/seeds/seeds8.txt") {}
}
