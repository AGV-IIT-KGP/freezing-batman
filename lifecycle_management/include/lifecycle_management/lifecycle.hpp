#ifndef __LIFECYCLE_H_
#define __LIFECYCLE_H_

#include <iostream>

using namespace std;

namespace utils {
	
	/**
	 * Class to provide functionalities and clean API for external lifecycle management.
	 * Can be applicable to any devices, processes, servers, modules or other entities
	 * which require to maintain state.
	 * LifeCycle::ActivityStatus is used by lifecycle manager as
	 * a result of any lifecycle event.
	 * @author Tanmay Patil 
	 * @see utils::LifeCycle::ActivityStatus
	 * @see utils::LifeCycleManager
	 */
	class LifeCycle {
		
	public:
		
		/**
		 * Represents status returned by each lifecycle management function.
		 * It can be used as a feedback so as to determine if a retry is needed
		 * for any lifecycle management function.
		 */
		typedef enum ActivityStatus {
			
			SUCCESS = 0,			/**< Activity was successfully executed. */
			FAILED_PERMANENT = 1,	/**< Activity failed, retry is useless. Use carefuly. */
			FAILED_PARTIAL = 2,		/**< Activity failed, a retry might be appropriate. */
			ILLEGAL_STATE = 3,		/**< Activity can not be called in this state. */
			UNSUPPORTED = 4			/**< Activity is not supported by the entity. */
			
		} ActivityStatus;
		
		/**
		 * Name of the entity to be used for logging.
		 */
		virtual std::string getName() = 0;
		
		/**
		 * Initialization of the entity. Will be called only once.
		 * Useful for memory allocation purpose.
		 * Sets state to ActivityState::CREATED.
		 */
		virtual ActivityStatus onCreate() = 0;
		
		/**
		 * Called at the start-up. Will be called only once.
		 * Useful for communications (ports, publishers, subscribers, services etc)
		 * and utilities establishments.
		 * Sets state to ActivityState::STARTED.
		 */
		virtual ActivityStatus onStart() = 0;
		
		/**
		 * Starts or resumes actual working of the entity. Can be called again if it was paused.
		 * Sets state to ActivityState::RESUMED.
		 */
		virtual ActivityStatus onResume() = 0;
		
		/**
		 * Called whenever the entity is needed to be paused. Can be called multiple times.
		 * Sets state to ActivityState::PAUSED.
		 */
		virtual ActivityStatus onPause() = 0;
		
		/**
		 * Called when complete stop is required. Can be restarted if needed.
		 * Sets state to ActivityState::Stopped
		 */
		virtual ActivityStatus onStop() = 0;
		
		/**
		 * Called when a restart is required. Can be called multiple times is it was stopped.
		 * Sets state to ActivityState::PAUSED
		 */
		virtual ActivityStatus onRestart() = 0;
		
		/**
		 * Called to destroy the entity. Can not be used anymore.
		 * Sets state to ActivityState::DESTROYED
		 */
		virtual ActivityStatus onDestroy() = 0;
		
		virtual ~LifeCycle() {
			
		}
		
	};
	
	
	
}

#endif
