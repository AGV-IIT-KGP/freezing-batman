#ifndef __LIFECYCLEMANAGER_H_
#define __LIFECYCLEMANAGER_H_

#include <ros/ros.h>

#include "lifecycle_management/lifecycle.hpp"

namespace utils {
	
	/**
	 * Class to manage and invoke lifecycle events of any lifecycle holder entity.
	 * @author Tanmay Patil
	 * @see utils::LifeCycle
	 * @see utils::LifeCycle::ActivityStatus
	 */
	class LifeCycleManager {
		
	private:
	
		/**
		 * Represents state of the entity for invokinng lifecycle events.
		 */
		enum LifeCycleState {
			
			NOT_INITIALIZED,	/**< To be created */
			CREATED,			/**< To be started */
			STARTED,			/**< To be resumed */
			RESUMED,			/**< To be paused */
			PAUSED,				/**< To be resumed or stopped */
			STOPPED,			/**< To be restarted or destroyed */
			DESTROYED			/**< Unusable */
			
		};
		
		/**
		 * Maintains state of the activity so as to determine which lifecycle event should be evoked.
		 * To be maintained by lifecycle manager after calling each method.
		 */
		LifeCycleState lifeCycleState;
		
		/**
		 * The entity for which the manager operates.
		 */
		LifeCycle *lifeCycleHolder;
		
	public:
		
		/**
		 * Contructor which accepts entity for which lifecycle manager will be operating.
		 */
		LifeCycleManager(LifeCycle *lifeCycleHolder);
		
		/**
		 * To be called when entity is to be created.
		 */
		LifeCycle::ActivityStatus create();
		
		/**
		 * To be called when entity is to be started.
		 */
		LifeCycle::ActivityStatus start();
		
		/**
		 * To be called when entity is to be resumed.
		 */
		LifeCycle::ActivityStatus resume();
		
		/**
		 * To be called when entity is to be paused.
		 */
		LifeCycle::ActivityStatus pause();
		
		/**
		 * To be called when entity is to be stopped.
		 */
		LifeCycle::ActivityStatus stop();
		
		/**
		 * To be called when entity is to be restarted.
		 */
		LifeCycle::ActivityStatus restart();
		
		/**
		 * To be called when entity is to be destroyed.
		 */
		LifeCycle::ActivityStatus destroy();
		
	};
	
}

#endif
