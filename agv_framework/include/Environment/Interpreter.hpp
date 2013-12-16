/* 
 * File:   Interpreter.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 12:50 AM
 */

#ifndef INTERPRETER_HPP
#define	INTERPRETER_HPP

#include "Model.hpp"

namespace environment {

    class Interpreter {
    public:
        virtual void interpret() = 0;
    };
}

#endif	/* INTERPRETER_HPP */

