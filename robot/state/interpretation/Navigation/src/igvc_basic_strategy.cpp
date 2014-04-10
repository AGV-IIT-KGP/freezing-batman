#include "Navigation/Navigation.h"

namespace navigation_space {

    Triplet navigation_space::IGVCBasicStrategy::getTargetLocation(double x1, double y1, double heading) {
        heading *= (3.142 / 180.0);

        double alpha = -heading;
        double x2 = x1 * cos(alpha) + y1 * sin(alpha);
        double y2 = -x1 * sin(alpha) + y1 * cos(alpha);

        // Adjusting according to map's scale
        x2 *= 100;
        y2 *= 100;

        // Shifting to bot's center
        x2 += 500;
        y2 += 100;

        int tx, ty;
        navigation_space::truncate(x2, y2, &tx, &ty);

        Triplet target_location;
        target_location.x = tx;
        target_location.y = ty;
        target_location.z = 90;

        return target_location;
    }

    Triplet navigation_space::IGVCBasicStrategy::getBotLocation() {
        Triplet bot_location;
        bot_location.x = 0.5 * MAP_MAX;
        bot_location.y = 0.1 * MAP_MAX;
        bot_location.z = 90;
        return bot_location;
    }
}

