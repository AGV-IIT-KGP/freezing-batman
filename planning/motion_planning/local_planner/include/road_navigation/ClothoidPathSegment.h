/* 
 * File:   ClothoidPathSegment.h
 * Author: Shiwangi
 *
 * Created on 23 January, 2014, 6:25 PM
 */

#ifndef CLOTHOIDPATHSEGMENT_H
#define	CLOTHOIDPATHSEGMENT_H
#include "PathSegment.h"

class ClothoidPathSegment : public PathSegment {
public:
    double sigma;

    ClothoidPathSegment(std::vector<State>& path_, double larc_, double sigma_) : PathSegment(path_, larc_), sigma(sigma_) {
    }


};


#endif	/* CLOTHOIDPATHSEGMENT_H */

