/* 
 * File:   Representation.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 3:25 AM
 */

#ifndef REPRESENTATION_HPP
#define	REPRESENTATION_HPP

#include <environment/Model.hpp>
#include <vector>

namespace environment {

    class Representation {
    public:
        Representation();
        Representation(const Representation& orig);
        virtual ~Representation();
    private:
        std::vector<Model> models;
    };
}


#endif	/* REPRESENTATION_HPP */

