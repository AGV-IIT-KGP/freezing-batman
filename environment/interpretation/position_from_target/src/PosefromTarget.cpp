#include "PosefromTarget.hpp"

typedef long double precnum_t;

void PosefromTarget::interpret() {
	/*...... Taken from the url: https://github.com/jlblancoc/mrpt/blob/master/libs/topography/src/conversions.cpp ........*/	
	
	static const precnum_t a = 6378137L;		// Semi-major axis of the Earth (meters)
	static const precnum_t b = 6356752.3142L;	// Semi-minor axis:
	static const precnum_t ae = acos(b/a);  	// eccentricity:
	static const precnum_t cos2_ae_earth =  (cos(ae))*(cos(ae)); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
	static const precnum_t sin2_ae_earth = (sin(ae))*(sin(ae));  // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;
	

	precnum_t lon  = ( precnum_t(current.longitude)*22/7/180 );
	precnum_t lat  = ( precnum_t(current.latitude)*22/7/180 );
	precnum_t N = a / std::sqrt( 1 - sin2_ae_earth*( sin(lat) )*( sin(lat) ) );
	
	pose.position.x = (N+current.altitude)*cos(lat)*cos(lon);
	pose.position.y = (N+current.altitude)*cos(lat)*sin(lon);
	pose.position.z = (cos2_ae_earth*N+current.altitude)*sin(lat);
	
	lon  = ( precnum_t(target.longitude)*22/7/180 );
	lat  = ( precnum_t(target.latitude) *22/7/180);
	N = a / std::sqrt( 1 - sin2_ae_earth*( sin(lat) )*( sin(lat) ) );
	
	temp.position.x = (N+target.altitude)*cos(lat)*cos(lon);
	temp.position.y = (N+target.altitude)*cos(lat)*sin(lon);
	temp.position.z = (cos2_ae_earth*N+target.altitude)*sin(lat);
	
	const double clat = cos((current.latitude*22/7/180)), slat = sin((current.latitude*22/7/180));
	const double clon = cos((current.longitude*22/7/180)), slon = sin((current.longitude*22/7/180));
	
	temp.position.x -= pose.position.x;
	temp.position.y -= pose.position.y;
	temp.position.z -= pose.position.z;
	
	pose.position.x = -slon*temp.position.x + clon*temp.position.y;
	pose.position.y = -clon*slat*temp.position.x -slon*slat*temp.position.y + clat*temp.position.z;
	pose.position.z = clon*clat*temp.position.x + slon*clat*temp.position.y +slat*temp.position.z;
   	
   	posestamped.pose.position.x = pose.position.x;
    posestamped.pose.position.y = pose.position.y;
    posestamped.pose.position.z = pose.position.z;
	
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
    current.altitude
    =msg->altitude;
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

