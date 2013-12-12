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
    virtual void onCreate();
    virtual void onStart();
    virtual void onPause();
    virtual void onResume();
    virtual void onRestart();
    virtual void onStop();
    virtual void onDestroy();
};

#endif	/* LIFECYCLE_HPP */

