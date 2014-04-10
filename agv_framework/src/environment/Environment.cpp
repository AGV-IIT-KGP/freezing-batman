/* 
 * File:   Environment.cpp
 * Author: Satya Prakash
 * 
 * Created on December 13, 2013, 6:27 PM
 */

#include "environment/Environment.hpp"

namespace environment {

    Environment* Environment::instance = NULL;

    Environment::Environment() {
    }

    Environment::Environment(const Environment& orig) {
    }

    Environment::~Environment() {
    }
}
