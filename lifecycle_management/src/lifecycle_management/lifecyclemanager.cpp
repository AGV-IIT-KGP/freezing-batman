#include <ros/ros.h>

#include "lifecycle_management/lifecycle.hpp"
#include "lifecycle_management/lifecyclemanager.hpp"

namespace utils {
		
		/**
		 * Contructor which accepts entity for which lifecycle manager will be operating.
		 */
		LifeCycleManager::LifeCycleManager(LifeCycle *lifeCycleHolder) {
			this->lifeCycleHolder = lifeCycleHolder;
			lifeCycleState = NOT_INITIALIZED;
		}
		
		/**
		 * To be called when entity is to be created.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::create() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == NOT_INITIALIZED) {
					ROS_INFO("Creating %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onCreate();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = CREATED;
					}
				}
			}
			return activityStatus;
		}
		
		/**
		 * To be called when entity is to be started.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::start() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == CREATED) {
					ROS_INFO("Starting %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onStart();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = STARTED;
					}
				}
			}
			return activityStatus;
		}
		
		/**
		 * To be called when entity is to be resumed.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::resume() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == STARTED || lifeCycleState == PAUSED) {
					ROS_INFO("Resuming %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onResume();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = RESUMED;
					}
				}
			}
			return activityStatus;
		}
		
		/**
		 * To be called when entity is to be paused.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::pause() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == RESUMED) {
					ROS_INFO("Pausing %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onPause();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = PAUSED;
					}
				}
			}
			return activityStatus;
		}
		
		/**
		 * To be called when entity is to be stopped.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::stop() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == PAUSED) {
					ROS_INFO("Stopping %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onStop();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = STOPPED;
					}
				}
			}
			return activityStatus;
		}
		
		/**
		 * To be called when entity is to be restarted.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::restart() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == STOPPED) {
					ROS_INFO("Restarting %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onRestart();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = PAUSED;
					}
				}
			}
			return activityStatus;
		}
		
		/**
		 * To be called when entity is to be destroyed.
		 */
		LifeCycle::ActivityStatus LifeCycleManager::destroy() {
			LifeCycle::ActivityStatus activityStatus = LifeCycle::ILLEGAL_STATE;
			if (lifeCycleHolder != NULL) {
				if (lifeCycleState == STOPPED) {
					ROS_INFO("Destroying %s...", lifeCycleHolder->getName().c_str());
					activityStatus = lifeCycleHolder->onDestroy();
					if (activityStatus == LifeCycle::SUCCESS) {
						lifeCycleState = DESTROYED;
					}
				}
			}
			return activityStatus;
		}
	
}
