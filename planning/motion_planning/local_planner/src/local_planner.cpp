#include <RoadNavigation.hpp>

using namespace message_filters;
using namespace std;
ros::Publisher bestpath_pub;
using namespace navigation;

//class Path {
//public:
//    std::vector<PathSegment*> paths;
//};

void function(const nav_msgs::Path::ConstPtr& lane_traj, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
    navigation::RoadNavigation planner;
    planner.planRoadDetection(lane_traj, pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;
//    bestpath_pub = n.advertise<local_planner::Path>("local_planner/path", 10);

    message_filters::Subscriber<nav_msgs::Path> lane_traj_sub(n, "sensor_fusion/lanes/trajectory", 10);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> localization_pose_sub(n, "localization/pose", 100);

    typedef sync_policies::ApproximateTime<nav_msgs::Path, geometry_msgs::PoseWithCovarianceStamped> ApproxTimePolicy;
    Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), lane_traj_sub, localization_pose_sub);

    sync.registerCallback(boost::bind(&function, _1, _2));

    ros::spin();

    return 0;
}
