
#include "quick_reflex/quick_reflex.hpp"

namespace navigation {
    bool quickReflex::isOnTheObstacle(const State& state){
        return fusionMap.at<uchar>(fusionMap.rows - state.y() -1, state.x()) >= 225;
    }
}
