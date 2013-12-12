/* 
 * File:   Interpreter.hpp
 * Author: Satya Prakash
 *
 * Created on December 12, 2013, 11:06 PM
 */

#ifndef INTERPRETER_HPP
#define	INTERPRETER_HPP

namespace environment {

    class Interpreter {
    public:
        virtual Model* interpret();
    };
}

#endif	/* INTERPRETER_HPP */

