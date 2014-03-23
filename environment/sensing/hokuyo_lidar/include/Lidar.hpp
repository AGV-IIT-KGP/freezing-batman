/* 
 * File:   HokuyoLidar.hpp
 * Author: satya
 *
 * Created on December 12, 2013, 9:47 PM
 */

#ifndef HOKUYOLIDAR_HPP
#define	HOKUYOLIDAR_HPP

#include <environment/Sensor.hpp>


using namespace std;
extern IplImage* showImg1;
extern IplImage* showImg2;
extern IplImage* showImg3;


class HokuyoLidar : public environment::Sensor {
public:
    bool connect();
    bool disconnect();
    bool fetch();
    void publish(int frame_id);

    static void update_map(const sensor_msgs::LaserScan&);
    static void writeVal(int val);
    
    HokuyoLidar(string serial_name);
    HokuyoLidar(int argc, char** argv);
    HokuyoLidar(const HokuyoLidar& orig);
    virtual ~HokuyoLidar();

private:
	//Parameters
	int lidar_id;
    std::string node_name;
    std::string topic_name;
    int message_queue_size;


    void initializeParameters();
    void initializeParameters(int argc, char** argv);
    void setupCommunications();

	static void createCircle(int x, int y);

};

#endif	/* HOKUYOLIDAR

