//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
namespace navigation {

    std::pair<std::vector<StateOfCar>, Seed> AStarSeed::reconstructPath(StateOfCar const& currentStateOfCar_, std::map<StateOfCar, StateOfCar, comparatorMapState>& came_from) {

        StateOfCar currentStateOfCar = currentStateOfCar_;

        std::vector<StateOfCar> path;

        path.push_back(currentStateOfCar);

        while (came_from.find(currentStateOfCar) != came_from.end()) {

            currentStateOfCar = came_from[currentStateOfCar];

            path.push_back(currentStateOfCar);

        }

        if (path.size() < 2)
            return std::make_pair(path, Seed());
        return std::make_pair(path, givenSeeds[path[path.size() - 2].seedTaken()]);

    }
}
