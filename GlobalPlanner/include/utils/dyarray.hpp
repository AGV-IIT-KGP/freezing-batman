//
//  dyarray.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 19/02/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__dyarray__
#define __GlobalPlanner__dyarray__

#include <iostream>

namespace SS {
    
    template<class Type>
    class dyarray  {
        
        Type* data;
        
    public:
        inline dyarray(size_t size,const Type& value) {
            data = new Type[size]{value};
        }
        
        inline dyarray(size_t size){
            data = new Type[size];
        }
        
        inline dyarray(){
            data = nullptr;
        }
        
        inline Type& operator[](size_t index) const {
            return data[index];
        }
        
        inline ~dyarray(){
            delete data;
        }
        
        
        
    };
}

#endif /* defined(__GlobalPlanner__dyarray__) */
