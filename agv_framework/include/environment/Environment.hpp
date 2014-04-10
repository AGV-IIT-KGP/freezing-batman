/* 
 * File:   Environment.hpp
 * Author: Satya Prakash
 *
 * Created on December 12, 2013, 11:03 PM
 */

#ifndef ENVIRONMENT_HPP
#define	ENVIRONMENT_HPP

#include "Model.hpp"
#include <iostream>
#include <vector>

//Singleton Class

namespace environment {

    class Environment {
        static Environment* instance;
        std::vector<Model*> models;
        // --> data structure for map

        Environment();
        Environment(const Environment& orig);
        virtual ~Environment();
    public:

        Environment* getInstance() {
            if (!instance)
                instance = new Environment();
            return instance;
        }
    };
}

#endif	/* ENVIRONMENT_HPP */

