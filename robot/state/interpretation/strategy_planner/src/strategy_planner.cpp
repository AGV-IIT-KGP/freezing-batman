#include "strategy_planner/strategy_planner.hpp"

void setDummyTarget (geometry::msgs proposed_dummy_target_)
{
	dummy_target_ = proposed_dummy_target_;
	return;
}

void setNoseTarget (geometry::msgs proposed_nose_target_)
{
	nose_target_ = proposed_nose_target_;
	return;
}

void setWaypointTarget (geometry::msgs proposed_waypoint_target_)
{
	waypoint_target_ = proposed_waypoint_target_;
	return;
}

void setLaneTarget (geometry::msgs proposed_lane_target_)
{
	lane_target_ = proposed_lane_target_;
	return;
}