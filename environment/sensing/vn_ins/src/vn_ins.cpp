#include <vn_ins.hpp>
#include <LifeCycle.hpp>

vectorNav::vectorNav(int argc, char** argv) : Sensor(argc, argv) {
    if (argc > 1) {
        initializeParameters(argc, argv);
    } else {
        initializeParameters();
    }
    ros::init(argc, argv, node_name.c_str());
    node_handle = new ros::NodeHandle();
    setupCommunications();
}

vectorNav::~vectorNav() {
}

bool vectorNav::connect() {
    vn200_connect(&vn200, vn200_com_port.c_str(), baud_rate);
    vn100_connect(&vn100, vn100_com_port.c_str(), baud_rate);
    return true;
}

bool vectorNav::disconnect() {
    vn200_disconnect(&vn200);
    vn100_disconnect(&vn100);
    return true;
}

bool vectorNav::fetch() {
    unsigned short gpsWeek, status;
    float yaw;
    VnYpr vn100_attitude;
    VnVector3 vn100_magnetic, vn100_acceleration, vn100_angular_rate;
    vn100_getYawPitchRollMagneticAccelerationAngularRate(&vn100, &vn100_attitude, &vn100_magnetic, &vn100_acceleration, &vn100_angular_rate);
    yaw = vn100_attitude.yaw;

    unsigned char gpsFix, numberOfSatellites;
    float speedAccuracy, timeAccuracy, attitudeUncertainty, positionUncertainty, velocityUncertainty, temperature, pressure;
    double gpsTime, latitude, longitude, altitude;
    VnVector3 magnetic, acceleration, angularRate, ypr, latitudeLognitudeAltitude, nedVelocity, positionAccuracy;

    vn200_getGpsSolution(&vn200, &gpsTime, &gpsWeek, &gpsFix, &numberOfSatellites, &latitudeLognitudeAltitude, &nedVelocity, &positionAccuracy, &speedAccuracy, &timeAccuracy);
    ROS_INFO("Triangulating from %d satellites", numberOfSatellites);

    _gps.latitude = latitudeLognitudeAltitude.c0;
    _gps.longitude = latitudeLognitudeAltitude.c1;
    _gps.altitude = latitudeLognitudeAltitude.c2;

    _yaw.data = yaw;

    return true;
}

void vectorNav::publish(int frame_id) {
    fix_publisher.publish(_gps);
    yaw_publisher.publish(_yaw);
}

void vectorNav::initializeParameters() {
    baud_rate = 115200;
    message_queue_size = 10;
    node_name = std::string("vn_ins");
    fix_topic_name = node_name + std::string("/fix");
    yaw_topic_name = node_name + std::string("/yaw");
    vn200_com_port = std::string("/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTUTUVO5-if00-port0");
    vn100_com_port = std::string("/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTVJUC0O-if00-port0");
}

void vectorNav::initializeParameters(int argc, char** argv) {
    baud_rate = 115200;
    message_queue_size = 10;
    node_name = std::string("vn_ins") + std::string(argv[1]);
    fix_topic_name = node_name + std::string(argv[1]) + std::string("/fix");
    yaw_topic_name = node_name + std::string(argv[1]) + std::string("/yaw");
    vn200_com_port = std::string(argv[2]);
    vn100_com_port = std::string(argv[3]);
}

void vectorNav::setupCommunications() {
    fix_publisher = node_handle->advertise<sensor_msgs::NavSatFix>(fix_topic_name.c_str(), message_queue_size);
    yaw_publisher = node_handle->advertise<std_msgs::Float64>(yaw_topic_name.c_str(), message_queue_size);
}

int main(int argc, char** argv) {
    if (argc > 1 && argc <= 3) {
        printf("Usage : <name> <id_no> <COM_PORT>\n");
    }

    double loop_rate = 10;
    int frame_id;
    frame_id = 0;
    vectorNav *vectornav = new vectorNav(argc, argv);
    vectornav->connect();
    ROS_INFO("VectorNav successfully connected. \n");

    ros::Rate rate_enforcer(loop_rate);
    while (ros::ok()) {
        vectornav->fetch();
        vectornav->publish(frame_id);
        ros::spinOnce();
        rate_enforcer.sleep();
    }

    vectornav->disconnect();
    return 0;
}
