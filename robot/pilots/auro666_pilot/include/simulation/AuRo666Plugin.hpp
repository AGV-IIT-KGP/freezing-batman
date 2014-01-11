/* 
 * File:   AuRo666Plugin.hpp
 * Author: samuel
 *
 * Created on 4 January, 2014, 11:47 PM
 */

#ifndef AURO666PLUGIN_HPP
#define	AURO666PLUGIN_HPP

#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include <auro666_pilot/Controls.h>
#include <auro666_pilot/State.h>

namespace gazebo {

    class AuRo666Plugin : public ModelPlugin {
    public:
        AuRo666Plugin();
        AuRo666Plugin(const AuRo666Plugin& orig);
        virtual ~AuRo666Plugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        // TODO: Borrow its value from a standard library
        static const double PI = 3.14159265359;

        // TODO: Move them to the Visualization class provided by AGV Framework
        std::string NORMAL;
        std::string BLACK;
        std::string RED;
        std::string GREEN;
        std::string BROWN;
        std::string BLUE;
        std::string MAGENTA;
        std::string CYAN;
        std::string LIGHTGRAY;
        std::string YELLOW;
        std::string WHITE;

        // TODO: Should come from the AuRo666 Class
        static const double wheel_radius = 0.3;
        static const double wheel_base = 1.2;
        static const double vehicle_length = 1.6;

        static const double coefficient_p = 50;
        static const double coefficient_i = 15;
        static const double coefficient_d = 10;
        static const double coefficient_i_max = 50;
        static const double coefficient_i_min = -50;
        static const double command_max = 30;
        static const double command_min = -30;

        static const unsigned int front_left_yaw_id = 0;
        static const unsigned int front_right_yaw_id = 1;
        static const unsigned int back_left_velocity_id = 2;
        static const unsigned int back_right_velocity_id = 3;

        bool first_iteration;
        double command_velocity;
        double command_theta;

        gazebo::common::PID joint_pids[4];
        gazebo::physics::JointPtr joints[4];
        gazebo::physics::ModelPtr model_ptr;
        gazebo::event::ConnectionPtr connection_ptr;
        gazebo::common::Time last_update_time;

        ros::Publisher pose_publisher;
        ros::Publisher state_publisher;
        ros::Subscriber controls_subscriber;

        void applyControls(const auro666_pilot::Controls::ConstPtr& controls_ptr);
        void initializePID(sdf::ElementPtr sdf_element_ptr);
        void updateState();
    };

    GZ_REGISTER_MODEL_PLUGIN(AuRo666Plugin);
}

#endif	/* AURO666PLUGIN_HPP */

