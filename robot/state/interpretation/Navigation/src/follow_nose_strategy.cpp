#include "Navigation/Navigation.h"

namespace navigation_space {
    
    double reference_heading;

    void navigation_space::FollowNoseStrategy::calibrateReferenceHeading(double heading, int iterations) {
        if (iterations < 5) {
            reference_heading = heading;
            usleep(200 * 1000);
        } else if (iterations == 5) {
            ROS_INFO("Ref Heading: %lf", reference_heading);
        }
    }

    navigation::State navigation_space::FollowNoseStrategy::getTargetLocation(double heading) {
        
        double alpha;
        
        navigation::State target_location;
        int x,y,z;

        if (heading * reference_heading > 0) {
            alpha = reference_heading - heading;
        } else {
            if (heading - reference_heading > 180) {
                alpha = 360 + reference_heading - heading;
            } else if (reference_heading - heading > 180) {
                alpha = reference_heading - heading - 360;
            } else {
                alpha = reference_heading - heading;
            }
        }

        alpha *= M_PI / 180.0;

        double map_height = 0.875 * MAP_MAX;
        double beta = atan(0.4 * (double)MAP_MAX / (double) map_height);
        double gamma = 3.14 - atan(0.4 * (double)MAP_MAX/ (double)(MAP_MAX - map_height));

        if ((-beta <= alpha) && (alpha <= beta)) {
            x = map_height * tan(alpha) + 0.5 * MAP_MAX;
            y = map_height + 0.1 * MAP_MAX;
        } else if (alpha > beta && alpha < gamma) {
            x = 0.9 * MAP_MAX;
            y = 0.4 * MAP_MAX / tan(alpha) + 0.1 * MAP_MAX;
        } else if (alpha < -beta && alpha > -gamma) {
            x = 0.1 * MAP_MAX;
            y = 0.1 * MAP_MAX - 0.4 * MAP_MAX / tan(alpha);
        } else {
            x = 0.5 * MAP_MAX;
            y = 0.1 * MAP_MAX;
        }
        z = 0;
        
        x = x < 100 ? 100 : x;
        y = y < 100 ? 100 : y;

        target_location = navigation::State(x,y,z,0);

        return target_location;
    }

    navigation::State navigation_space::FollowNoseStrategy::getBotLocation() {
        navigation::State bot_location;
        int x,y,z;
        x = 0.5 * MAP_MAX;
        y = 0.1 * MAP_MAX;
        z = 90;
        bot_location = navigation::State(x,y,z,0);
        return bot_location;
    }
}

