/* 
 * File:   AuRo666Plugin.cpp
 * Author: samuel
 * 
 * Created on 4 January, 2014, 11:47 PM
 */

#include <simulation/AuRo666Plugin.hpp>

namespace gazebo {

    AuRo666Plugin::AuRo666Plugin() {
        // TODO: Move them to the Visualization class provided by AGV Framework        
        NORMAL = std::string("\033[0m");
        BLACK = std::string("\033[0;30m");
        RED = std::string("\033[0;31m");
        GREEN = std::string("\033[0;32m");
        BROWN = std::string("\033[0;33m");
        BLUE = std::string("\033[0;34m");
        MAGENTA = std::string("\033[0;35m");
        CYAN = std::string("\033[0;36m");
        LIGHTGRAY = std::string("\033[0;37m");
        YELLOW = std::string("\033[0;33m");
        WHITE = std::string("\033[37;01m");

        first_iteration = true;
        command_theta = 0;
        command_velocity = 1;

        char *argv[] = {"auro666_plugin"};
        int argc = 1;
        ros::init(argc, argv, "auro666_plugin");
        ros::NodeHandle node_handle;
        
        controls_subscriber = node_handle.subscribe("controller/controls", 2, &AuRo666Plugin::applyControls, this);
        pose_publisher = node_handle.advertise<geometry_msgs::Pose>("simulator/pose", 100);
        state_publisher = node_handle.advertise<auro666_pilot::State>("simulator/state", 100);
    }

    AuRo666Plugin::AuRo666Plugin(const AuRo666Plugin& orig) {
    }

    AuRo666Plugin::~AuRo666Plugin() {
    }

    void AuRo666Plugin::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_element_ptr) {
        this->model_ptr = model_ptr;

        initializePID(sdf_element_ptr);

        /* ********************* Assigning the joint pointers *********************** */
        // TODO: Replace such strings with const vars
        joints[front_left_yaw_id] = model_ptr->GetJoint("front_left_joint");
        joints[front_right_yaw_id] = model_ptr->GetJoint("front_right_joint");
        joints[back_left_velocity_id] = model_ptr->GetJoint("back_left_joint");
        joints[back_right_velocity_id] = model_ptr->GetJoint("back_right_joint");

        last_update_time = model_ptr->GetWorld()->GetSimTime();
        connection_ptr = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&AuRo666Plugin::updateState, this));
    }

    void AuRo666Plugin::applyControls(const auro666_pilot::Controls::ConstPtr& controls_ptr) {
        command_theta = controls_ptr->steer_angle;
        command_velocity = controls_ptr->rear_wheel_speed;
        ROS_DEBUG("%sTheta: %lf...........Vel: %lf%s", BLUE.c_str(), command_theta, command_velocity, NORMAL.c_str());
    }

    void AuRo666Plugin::initializePID(sdf::ElementPtr sdf_element_ptr) {
        for (int joint_id = 0; joint_id < 2; joint_id++) {
            joint_pids[joint_id].Init(coefficient_p,
                                      coefficient_i,
                                      coefficient_d,
                                      coefficient_i_max,
                                      coefficient_i_min,
                                      command_max,
                                      command_min);
            joint_pids[joint_id].SetCmd(0);
        }

        for (int joint_id = 2; joint_id < 4; joint_id++) {
            joint_pids[joint_id].Init(sdf_element_ptr->Get<double>("PID_P"),
                                      sdf_element_ptr->Get<double>("PID_I"),
                                      sdf_element_ptr->Get<double>("PID_D"),
                                      sdf_element_ptr->Get<double>("PID_I_MAX"),
                                      sdf_element_ptr->Get<double>("PID_I_MIN"),
                                      sdf_element_ptr->Get<double>("PID_CMD_MAX"),
                                      sdf_element_ptr->Get<double>("PID_CMD_MIN"));
            joint_pids[joint_id].SetCmd(0);
        }
    }

    void AuRo666Plugin::updateState() {
        ros::spinOnce();

        double targets[4];

        targets[back_left_velocity_id] = command_velocity / wheel_radius;
        targets[back_right_velocity_id] = command_velocity / wheel_radius;

        // Something wrong here! seriously wrong!
        double tan_command_theta = tan(command_theta);
        targets[front_left_yaw_id] = atan2(2 * vehicle_length * tan_command_theta,
                                           2 * vehicle_length - wheel_base * tan_command_theta);
        targets[front_right_yaw_id] = atan2(2 * vehicle_length * tan_command_theta,
                                            2 * vehicle_length + wheel_base * tan_command_theta);
        ROS_DEBUG("(%lf , %lf)", targets[front_left_yaw_id] * 180 / PI, targets[front_right_yaw_id] * 180 / PI);

        gazebo::common::Time current_time = model_ptr->GetWorld()->GetSimTime();
        for (int joint_id = 0; joint_id < 4; joint_id++) {
            double error = 0;

            if (joint_id == 0 || joint_id == 1) {
                error = joints[joint_id]->GetAngle(0).Radian() - targets[joint_id];
            }

            if (joint_id == 2 || joint_id == 3) {
                error = joints[joint_id]->GetVelocity(0) - targets[joint_id];
            }

            joint_pids[joint_id].Update(error, current_time.Double() - last_update_time.Double());
            joints[joint_id]->SetForce(0, joint_pids[joint_id].GetCmd());
        }
        last_update_time = current_time;

        tan_command_theta = tan(joints[front_left_yaw_id]->GetAngle(0).Radian());
        double current_theta = atan(2 * vehicle_length * tan_command_theta / (2 * vehicle_length + wheel_base * tan_command_theta));
        double current_velocity = joints[back_left_velocity_id]->GetVelocity(0) * wheel_radius;

        double origin_x = 0;
        double origin_y = 0;
        double origin_z = 0;

        double bot_x = model_ptr->GetWorldPose().pos.x;
        double bot_y = model_ptr->GetWorldPose().pos.y;
        double bot_z = model_ptr->GetWorldPose().pos.z;

        if (first_iteration) {
            origin_x = bot_x;
            origin_y = bot_y;
            origin_z = bot_z;
            first_iteration = false;
        }

        geometry_msgs::Pose pose;
        pose.position.x = bot_x - origin_x;
        pose.position.y = bot_y - origin_y;
        pose.position.z = bot_z - origin_z;
        pose.orientation.x = model_ptr->GetWorldPose().rot.x;
        pose.orientation.y = model_ptr->GetWorldPose().rot.y;
        pose.orientation.z = model_ptr->GetWorldPose().rot.z;
        pose.orientation.w = model_ptr->GetWorldPose().rot.w;
        pose_publisher.publish(pose);

        auro666_pilot::State state;
        state.steer_angle = current_theta;
        state.rear_wheel_speed = current_velocity;
        state_publisher.publish(state);
    }
}

