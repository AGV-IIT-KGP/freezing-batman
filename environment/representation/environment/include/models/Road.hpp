/* 
 * File:   Road.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 2:37 AM
 */

#ifndef ROAD_HPP
#define	ROAD_HPP

#include <Environment/Model.hpp>

class Road : public environment::Model {
public:
    Road();
    Road(const Road& orig);
    virtual ~Road();
private:

};

#endif	/* ROAD_HPP */

