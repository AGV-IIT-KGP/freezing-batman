#include "Navigation/Navigation.h"

namespace navigation_space {

    void truncate(double xt, double yt, int& xtt, int& ytt) {
        if ((yt <= 0.9 * MAP_MAX && yt >= 0.1 * MAP_MAX) && (xt <= 0.9 * MAP_MAX && xt >= 0.1 * MAP_MAX)) {
            xtt = xt;
            ytt = yt;
            return;
        }

        double xp, yp;
        int xb = 500;
        int yb = 100;

        if (yt < 0.1 * MAP_MAX) {
            ytt = 0.05 * MAP_MAX;
            xtt = (xt < xb) ? 0.25 * MAP_MAX : 0.75 * MAP_MAX;
            return;
        }

        // L1: y = 0.9A
        xp = xb + (0.9 * MAP_MAX - yb) * ((xt - xb) / (yt - yb));
        yp = 0.9 * MAP_MAX;
        if (((0.1 * MAP_MAX <= xp) && (xp <= 0.9 * MAP_MAX)) && ((yb - 0.9 * MAP_MAX) * (yt - 0.9 * MAP_MAX) <= 0) && (yt >= 0.9 * MAP_MAX)) {
            xtt = xp;
            ytt = yp;
            return;
        }

        // L2: y = 0.1A
        xp = xb + (0.1 * MAP_MAX - yb) * ((xt - xb) / (yt - yb));
        yp = 0.1 * MAP_MAX;

        if (((0.1 * MAP_MAX <= xp) && (xp <= 0.9 * MAP_MAX)) && ((yb - 0.1 * MAP_MAX) * (yt - 0.1 * MAP_MAX) <= 0) && (yt <= 0.1 * MAP_MAX)) {
            xtt = xp;
            ytt = yp;
            return;
        }

        // L3: x = 0.1A
        xp = 0.1 * MAP_MAX;
        yp = yb + (0.1 * MAP_MAX - xb) * ((yt - yb) / (xt - xb));
        if (yp > .1 * MAP_MAX && yp < .9 * MAP_MAX && (xt < 0.1 * MAP_MAX)) {
            xtt = xp;
            ytt = yp;
            return;
        }

        // L4: x = 0.9A
        xp = 0.9 * MAP_MAX;
        yp = yb + (0.9 * MAP_MAX - xb) * ((yt - yb) / (xt - xb));
        if (yp > .1 * MAP_MAX && yp < .9 * MAP_MAX && xt > 0.9 * MAP_MAX) {
            xtt = xp;
            ytt = yp;
            return;
        }
    }

}
