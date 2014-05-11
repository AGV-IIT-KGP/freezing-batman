#include "PosefromTarget.hpp"

void PosefromTarget::interpret() {
	mrpt::topography::TGeodeticCoords p0(current.latitude, current.longitude, current.altitude);
    mrpt::topography::TGeodeticCoords p1(target.latitude, target.longitude, target.altitude);
    mrpt::poses::TPoint3D p;
    /// p1 : TARGET
    /// p0 : CURRENT LOCATION OF BOT
    /// p : Location of Target w.r.t the current location of bot in ENU Coord Frame
    mrpt::topography::geodeticToENU_WGS84(p1, p, p0);
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = p.z;
    
    posestamped.pose.position.x = p.x;
    posestamped.pose.position.y = p.y;
    posestamped.pose.position.z = p.z;
    
publish_pose();
}

void PosefromTarget::publish_pose(){
	pose_pub.publish(pose);
	poseStamped_pub.publish(posestamped);
}

PosefromTarget::PosefromTarget(int argc, char* argv[]) {
	
	target_topic=std::string(argv[2]);
	gps_topic=std::string(argv[3]);
	
	pose_topic=std::string("sensors/pose/")+std::string(argv[1]);
	posestamped_topic=std::string("sensors/poseStamped/")+std::string(argv[1]);
	
	buffer=10;
	target_sub = nh.subscribe(target_topic.c_str(), buffer, &PosefromTarget::targetCallback, this);
	gps_sub = nh.subscribe(gps_topic.c_str(), buffer, &PosefromTarget::gpsback, this);
	
	pose_pub=nh.advertise<geometry_msgs::Pose>(pose_topic.c_str(),buffer);
	poseStamped_pub=nh.advertise<geometry_msgs::PoseStamped>(posestamped_topic.c_str(),buffer);
	
}	

PosefromTarget::PosefromTarget(const PosefromTarget& orig) {
}

PosefromTarget::~PosefromTarget() {
}

void PosefromTarget::targetCallback(const sensor_msgs::NavSatFix::ConstPtr msg){
	target.latitude=msg->latitude;
	target.longitude=msg->longitude;
    target.altitude=msg->altitude;
}

void PosefromTarget::gpsback(const sensor_msgs::NavSatFix::ConstPtr msg){
	current.latitude=msg->latitude;
	current.longitude=msg->longitude;
    current.altitude=msg->altitude;
}

int main(int argc, char** argv) {
	std::string node_name;
	if(argc<4){
		printf("Usage: <name> <id> <target_topic_name> <gps_topic_name>\n");
		return 1;
	}
	node_name = std::string("Position_From_Target_")+std::string(argv[1]);
	ros::init(argc, argv, node_name.c_str());
    PosefromTarget posefromtarget(argc, argv);
    int LOOP_RATE=10;
    ros::Rate loop_rate(LOOP_RATE);
    ROS_INFO("Pose Publisher Started");
    while (ros::ok()){
        ros::spinOnce();
        posefromtarget.interpret();
        loop_rate.sleep();
    }
	ROS_INFO("Pose Publisher exiting");
    return 0;
}

