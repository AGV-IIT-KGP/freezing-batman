const int MAP_MAX_ = 1000;

#include "quick_reflex/quick_reflex.hpp"
namespace navigation {
    
    bool quickReflex::isWalkableWithSeeds(State const& startState, State const& targetState, Seed targetSeed) {
        
        bool NoObstacle = true;

        for (std::vector<State>::iterator stateIt = targetSeed.intermediatePoints.begin(); stateIt != targetSeed.intermediatePoints.end(); ++stateIt) {

            int intermediateXcordinate = stateIt->x() + startState.x();
            int intermediateYcordinate = stateIt->y() + startState.y();
            
            if (((intermediateXcordinate >= 0) && (intermediateXcordinate < MAP_MAX_)) && (( intermediateYcordinate >= 0) && (intermediateYcordinate < MAP_MAX_))) {

                fusionMap.at<uchar>(fusionMap.rows - intermediateYcordinate -1, intermediateXcordinate) < 250 ? NoObstacle *= 1 : NoObstacle *= 0;
    
            } 
            else {
                return false;
            }
        }
        
        if(fusionMap.at<uchar>(fusionMap.rows - targetSeed.finalState.y() -1, targetSeed.finalState.x()) >= 250)
            return false;
        return NoObstacle == true;
    }
}
