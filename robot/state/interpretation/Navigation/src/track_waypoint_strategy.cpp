#include "Navigation/Navigation.h"

namespace navigation_space {
    int navigation_space::TrackWaypointStrategy::i=0;
    geometry_msgs::Pose navigation_space::TrackWaypointStrategy::target_pose = geometry_msgs::Pose();
    std::ifstream *navigation_space::TrackWaypointStrategy::waypoints=NULL;
    void navigation_space::TrackWaypointStrategy::publish(sensor_msgs::NavSatFix target_)
    {
        ros::NodeHandle nh;
        ros::Publisher pub_GPS_target_pose;
        pub_GPS_target_pose = nh.advertise<sensor_msgs::NavSatFix>("/GPS_waypoints/file_input",10);
        pub_GPS_target_pose.publish(target_);
    }

    void navigation_space::TrackWaypointStrategy::setTargetPose(geometry_msgs::Pose pose_from_target_)
    {
        target_pose=pose_from_target_;
    }

    bool navigation_space::TrackWaypointStrategy::readPublishTargetLocation() {

        sensor_msgs::NavSatFix target_;
        if (i==0)
        {
            waypoints->open("GPS_waypoints.txt",std::ios::in);
        }
        if (!waypoints->eof())
        {
            *waypoints>>target_.latitude;
            *waypoints>>target_.longitude;

            navigation_space::TrackWaypointStrategy::publish(target_);

            i++;
            return true;
        }

        if (waypoints->eof())
        {
            waypoints->close();
            return false;
        }
    }

    navigation::State navigation_space::TrackWaypointStrategy::getTargetLocation(double heading)
    {
        heading *= (M_PI / 180.0);  
        double alpha = -heading;
        double x2 = target_pose.position.x* cos(alpha) + target_pose.position.y* sin(alpha);
        double y2 = -target_pose.position.x* sin(alpha) + target_pose.position.y* cos(alpha);
    
        // Adjusting according to map's scale
        x2 *= 100;
        y2 *= 100;

        // Shifting to bot's center
        x2 += 500;
        y2 += 100;

        int tx, ty;
        navigation_space::truncate(x2, y2, tx, ty);

        geometry_msgs::Pose target_location;
        target_location.position.x = tx;
        target_location.position.y = ty;
        target_location.position.z = 90;//can be changed later to point to next waypoint.

        navigation::State target_state(target_location.position.x,target_location.position.y,target_location.position.z,0);
        return target_state;
    }

    navigation::State navigation_space::TrackWaypointStrategy::getBotLocation() {
        geometry_msgs::Pose bot_location;
        bot_location.position.x = 0.5 * MAP_MAX;
        bot_location.position.y = 0.1 * MAP_MAX;
        bot_location.position.z = 90;

        navigation::State bot_state(bot_location.position.x,bot_location.position.y,bot_location.position.z,0);
        return bot_state;
    }

}