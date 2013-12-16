/* 
 * File:   LifeCycle.hpp
 * Author: Satya Prakash
 *
 * Created on December 12, 2013, 10:53 PM
 */

#ifndef LIFECYCLE_HPP
#define	LIFECYCLE_HPP

class LifeCycle {
public:
    virtual void onCreate() = 0;
    virtual void onStart() = 0;
    virtual void onPause() = 0;
    virtual void onResume() = 0;
    virtual void onRestart() = 0;
    virtual void onStop() = 0;
    virtual void onDestroy() = 0;
};

#endif	/* LIFECYCLE_HPP */

