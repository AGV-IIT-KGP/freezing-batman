//
//  ss_priority_queue.hpp
//  AStarSeed
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __AStarSeed__SSPriority_queue__
#define __AStarSeed__SSPriority_queue__

#include <vector>
#include <queue>
#include <iterator>

namespace SS {
    
    
    template < typename T,typename Sequence = std::vector<T> , typename CompareFunction = std::less<typename Sequence::value_type> >
    class PriorityQueue : public std::priority_queue<T,Sequence, CompareFunction>
    {
    public:
        
        // PriorityQueue() : std::priority_queue<T>()    {}
        
        //        PriorityQueue(std::initializer_list<T> t) : PriorityQueue()   {
        //
        //            this->c = {t};
        //        }
        
        inline void clear(){   this->c.clear();    }
        
        //        void updateElement(const T& oldElement,const T& newElement)
        //        {
        //            for ( auto& i : this->c ) {
        //                if (i ==oldElement) {
        //                    i(newElement);
        //                    return;
        //                }
        //            }
        //            return;
        //        }
        
        
        //        T find(const T& value)
        //        {
        //            T first = this->c.begin();
        //            T last = this->c.end();
        //            for (; first != last; ++first) {
        //                if (*first == value) {
        //                    return first;
        //                }
        //            }
        //            return last;
        //        }
        
        
    };
}


#endif /* defined(__AStarSeed__SSPriority_queue__) */
