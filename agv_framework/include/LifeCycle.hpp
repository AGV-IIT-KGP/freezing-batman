/* 
 * File:   LifeCycle.hpp
 * Author: Satya Prakash
 *
 * Created on December 13, 2013, 6:24 PM
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
private:

};

#endif	/* LIFECYCLE_HPP */

