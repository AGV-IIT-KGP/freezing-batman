#include "backup_planner.hpp"

int main(int argc,char* argv[]) {

    const std::string node_name = "backup_planner";

    ros::init(argc, argv, node_name.c_str());

    ros::NodeHandle nh;
    
    navigation::backupPlanner local_planner_seed(nh);

    //local_planner_seed.plan();

    return 0;
}
