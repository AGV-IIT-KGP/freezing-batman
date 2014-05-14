
#include "local_planner.hpp"

int main(int argc,char* argv[]) {

    const std::string node_name = "local_planner";

    ros::init(argc, argv, node_name.c_str());

    ros::NodeHandle nh;
    
    navigation::LocalPlanner local_planner_seed(nh);

    local_planner_seed.planWithQuickReflex();
    
    return 0;
}
