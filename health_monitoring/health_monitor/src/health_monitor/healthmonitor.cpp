#include <ros/ros.h>

#include <lifecycle_management/lifecycle.hpp>
#include <lifecycle_management/lifecyclemanager.hpp>

#include <iostream>

class MyClass : public utils::LifeCycle {
	
	std::string getName() {
		return "MyClassObject";
	}
		
	ActivityStatus onCreate() {
		ROS_INFO("onCreate Called");
		return utils::LifeCycle::SUCCESS;
	}
		
	ActivityStatus onStart() {
		ROS_INFO("onStart Called");
		return utils::LifeCycle::SUCCESS;
	}
		
	ActivityStatus onResume() {
		ROS_INFO("onResume Called");
		return utils::LifeCycle::SUCCESS;
	}
		
	ActivityStatus onPause() {
		ROS_INFO("onPause Called");
		return utils::LifeCycle::SUCCESS;
	}
		
	ActivityStatus onStop() {
		ROS_INFO("onStop Called");
		return utils::LifeCycle::SUCCESS;
	}
		
	ActivityStatus onRestart() {
		ROS_INFO("onRestart Called");
		return utils::LifeCycle::SUCCESS;
	}
		
	ActivityStatus onDestroy() {
		ROS_INFO("onDestroy Called");
		return utils::LifeCycle::SUCCESS;
	}
		
};
	
int main(int argc, char **argv) {
	
	ROS_INFO("Testing Life Cycle Event System");
	
	MyClass myClass;
	utils::LifeCycleManager lifeCycleManager(&myClass);
	
	lifeCycleManager.create();
	lifeCycleManager.start();
	lifeCycleManager.create();
	lifeCycleManager.resume();
	lifeCycleManager.pause();
	lifeCycleManager.resume();
	lifeCycleManager.pause();
	lifeCycleManager.stop();
	lifeCycleManager.restart();
	lifeCycleManager.stop();
	lifeCycleManager.destroy();
	
}
