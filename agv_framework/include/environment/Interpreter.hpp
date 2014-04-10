/* 
 * File:   Interpreter.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 6:33 PM
 */

#ifndef INTERPRETER_HPP
#define	INTERPRETER_HPP

#include "Model.hpp"

namespace environment {

    class Interpreter {
    public:
        virtual void interpret() = 0;

        Interpreter();
        Interpreter(const Interpreter& orig);
        virtual ~Interpreter();
    private:
    };
}
#endif	/* INTERPRETER_HPP */

