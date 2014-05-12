#include "quick_reflex/quick_reflex.hpp"

namespace navigation {
    
    std::vector<Seed> quickReflex::neighborNodesWithSeeds(const State&  start,const State&  goal) {
        std::vector<Seed> neighbours;
        
        for ( int i = 0; i < givenSeeds.size(); i++) {
            
            if ( !isWalkableWithSeeds(start, goal, givenSeeds[i])) {
                continue;
            }
            neighbours.push_back(givenSeeds[i]);
        }
        return neighbours;
    }
}
