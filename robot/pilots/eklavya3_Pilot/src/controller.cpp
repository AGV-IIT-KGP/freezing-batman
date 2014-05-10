#include "eklavya3_Pilot/controller.hpp"

void sendCommand(cons   local_planner::Seed temp) {
        geometry_msgs::Twist cmdvel;
        local_planner::Seed seed = temp;

        int left_vel = 0;
        int right_vel = 0;
        float left_velocity = seed.leftVelocity;
        float right_velocity = seed.rightVelocity;

        if ((left_velocity == 0) && (right_velocity == 0)) {
            pub_control.publish(cmdvel);
        }
        else if ((left_velocity >= 0) && (right_velocity >= 0)) {
            switch (PID_MODE) {
                case 0:
                {
                    if (left_velocity == right_velocity) {
                        double vavg = 80;
                        left_vel = right_vel = vavg;
			            printf("straight seed\n");
                    }
                    else if (seed.velocityRatio == 1.258574 || seed.velocityRatio == 0.794550) {
                        double vavg = 50;
                        double aggression = 1;
                        seed.velocityRatio = seed.velocityRatio < 1 ? seed.velocityRatio / aggression : seed.velocityRatio * aggression;
                        left_vel = (int) 2 * vavg * seed.velocityRatio / (1 + seed.velocityRatio);
                        right_vel = (int) (2 * vavg - left_vel);
			            printf("soft seed\n");
                    } 
                    else if (seed.velocityRatio == 1.352941 || seed.velocityRatio == 0.739130) {
                        double vavg = 20;
                        double aggression = 1.5;
                        seed.velocityRatio = seed.velocityRatio < 1 ? seed.velocityRatio / aggression : seed.velocityRatio * aggression;
                        left_vel = (int) 2 * vavg * seed.velocityRatio / (1 + seed.velocityRatio);
                        right_vel = (int) (2 * vavg - left_vel);
			             printf("hard seed\n");
                    }

                    break;
                }
        //         case 1:
        //         {
        //             double myTargetCurvature = 5.0 * ((double) (left_velocity - right_velocity)) / (left_velocity + right_velocity);
        //             static double myYaw = 0.5;
        //             static double previousYaw = 1;
        //             static double errorSum = 0;
        //             static double previousError;
        //             double Kp = 6.4, Kd = 0.01, Ki = 0.0001;
        //             left_vel = 0;
        //             right_vel = 0;
        //             int mode = 4;

        //             double error = (myTargetCurvature - (myYaw - previousYaw) / 0.37);
        //             errorSum += error;

        //             if (seed.velocityRatio > 1.35) {
        //                 mode = 2;
        //             } else if (seed.velocityRatio > 1.25) {
        //                 mode = 3;
        //             } else if (seed.velocityRatio > 0.99) {
        //                 mode = 4;
        //             } else if (seed.velocityRatio > 0.79) {
        //                 mode = 5;
        //             } else if (seed.velocityRatio > 0.73) {
        //                 mode = 6;
        //             } else {
        //                 mode = 4;
        //             }

        //             int hashSpeedLeft[9] = {23, 30, 35, 38, 40, 42, 45, 50, 57};
        //             int hashSpeedRight[9] = {57, 50, 45, 42, 40, 38, 35, 30, 23};

        //             previousYaw = myYaw;

        //             pthread_mutex_lock(&pose_mutex);
        //             myYaw = pose.orientation.z;
        //             pthread_mutex_unlock(&pose_mutex);

        //             mode += (int) (Kp * error + Ki * errorSum + Kd * (error - previousError));

        //             if (mode < 0) {
        //                 mode = 0;
        //             }
        //             if (mode > 8) {
        //                 mode = 8;
        //             }

        //             left_vel = hashSpeedLeft[mode];
        //             right_vel = hashSpeedRight[mode];

        //             ROS_INFO("[PID] %lf, %lf, %lf, %lf, %d, %d",
        //                     myTargetCurvature,
        //                     (myYaw - previousYaw) / 0.37,
        //                     left_velocity,
        //                     right_velocity,
        //                     left_vel,
        //                     right_vel);

        //             break;
        //         }
        //         case 2:
        //         {
        //             pthread_mutex_lock(&controllerMutex);
        //             targetCurvature = 5.0 * ((double) (seed.velocityRatio - 1.0)) / (seed.velocityRatio + 1.0);
        //             ROS_INFO("Updated : %lf, k = %lf, left = %lf, right = %lf", targetCurvature, seed.velocityRatio, s.vl, s.vr);
        //             pthread_mutex_unlock(&controllerMutex);

        //             break;
        //         }
            }
        } 
            else {
            ROS_INFO("REVERSING");
            
            left_vel = left_velocity;
            right_vel = right_velocity;
        }



        left_vel = left_vel > 80 ? 80 : left_vel;
        right_vel = right_vel > 80 ? 80 : right_vel;
        left_vel = left_vel < -80 ? -80 : left_vel;
        right_vel = right_vel < -80 ? -80 : right_vel;

        //	if(s.vl==-30&&s.vr==30)
        //	{
        //	left_vel=-30;
        //	right_vel=30;
        //	}
        //	if(s.vl==30&&s.vr==-30)
        //	{
        //	left_vel=30;
        //	right_vel=-30;
        //	}

        double scale = 100;
        double w = 0.55000000;
        cmdvel.linear.x = (left_vel + right_vel) / (2 * scale);
        cmdvel.linear.y = 0;
        cmdvel.linear.z = 0;
        cmdvel.angular.x = 0;
        cmdvel.angular.y = 0;
        cmdvel.angular.z = (left_vel - right_vel) / (w * scale);
        
        pub_control.publish(cmdvel);
//        std::cout << "linear: " << cmdvel.linear.x << " angular: " << cmdvel.angular.z << std::endl;
        ROS_INFO("[Planner] Command : (%d, %d)", left_vel, right_vel);

}


