/* 
 * File:   Clothoid.cpp
 * Author: Shiwangi
 *
 * Created on 23 January, 2014, 6:24 PM
 */
 
#include <road_navigation/Clothoid.hpp>

double Clothoid::inRange(double theta) {
    if (theta > PI) {
        while (theta > PI) {
            theta -= 2 * PI;
        }
    }

    if (theta <= PI) {
        while (theta <= -PI) {
            theta += 2 * PI;
        }
    }

    return theta;
}

Clothoid::Clothoid() {
    kMax = 1000;
    solution = 1;
    debug = true;
}

int Clothoid::signum(double a) {
    if (a < 0)
        return -1;
    else if (a > 0)
        return 1;
    else
        return 0;
}

double Clothoid::calcD(double alpha) {
    double x, z;
    fresnel(sqrt(2 * alpha / PI), x, z);
    double d = cos(alpha) * (x)+ (sin(alpha) * z);
    return d;
}

void Clothoid::getPath(State a, State b) {
    if (debug) {
        cout << "Start :" << a.x << " " << a.y << " " << a.theta << endl;
        cout << "End" << b.x << " " << b.y << " " << b.theta << endl;
    }

    if (fabs(a.x - b.x) < 1 || fabs(a.y - b.y) < 1) {
        solution = 0;
        return;
    }
    double beta = atan2((b.y - a.y), (b.x - a.x));

    if (fabs(beta - b.theta - a.theta + beta) < 0.001 && fabs(b.theta - a.theta) < PI) {
        if (debug) {
            cout << "Symmetric:\n" << endl;
        }

        start = a;
        end = b;
        double alpha = inRange((-start.theta + end.theta)) / 2;

        double D = calcD(fabs(alpha));
        path.sigma = 4 * PI * signum(alpha) * D * D / start.getDistance(end) / start.getDistance(end);
        if (path.sigma == 0) {
            path.lengthOfPath = start.getDistance(end);
            for (double s = 0; s < path.lengthOfPath; s += path.lengthOfPath / 1000) {
                path.path.push_back(State(start.x + s * cos(start.theta), start.y + s * sin(start.theta), 0));
            }
            paths.push_back(ClothoidPathSegment(path.path, path.lengthOfPath, path.sigma));
            return;
        } else {
            path.lengthOfPath = 2 * sqrt(fabs(2 * alpha / path.sigma));
            getTrajectory();
            paths.push_back(ClothoidPathSegment(path.path, path.lengthOfPath, path.sigma));
            std::cout << std::endl << std::endl << std::endl;
        }
    } else if (a.theta == b.theta) {
        State p((a.x + b.x) / 2, (a.y + b.y) / 2, 0);
        double beta = atan2((p.y - a.y), (p.x - a.x));
        p.theta = 2 * beta - a.theta;
        getPath(a, p);
        getPath(p, b);

    } else {
        double alpha = inRange(((-a.theta + b.theta)) / 2);
        double cc;
        cc = cos(alpha) / sin(alpha);
        if (debug) {
            cout << "c is" << cc << endl;
        }
        State p(0, 0, 0);
        p.x = (a.x + b.x + cc * (a.y - b.y)) / 2;
        p.y = (a.y + b.y + cc * (b.x - a.x)) / 2;
        double r = p.getDistance(a);
        double deflection1 = (atan2((p.y - a.y), (p.x - a.x)));
        double deflection2 = (atan2((p.y - b.y), (p.x - b.x)));
        double def;
        State c(0, 0, 0);
        if (deflection2 > deflection1) {
            //swap
            double temp = deflection2;
            deflection2 = deflection1;
            deflection1 = temp;
            c.x = a.x;
            c.y = a.y;
            c.theta = a.theta;
            a.x = b.x;
            a.y = b.y;
            a.theta = b.theta;
            b.x = c.x;
            b.y = c.y;
            b.theta = c.theta;
        }
        def = ((deflection2 + deflection1)) / 2;
        alpha = (((-a.theta + b.theta)) / 2);
        if (alpha < 0) {

            def = inRange(PI + def);
        }
        State q(0, 0, 0);
        q.x = p.x + r * cos(def);
        q.y = p.y + r * sin(def);

        double the = atan2((a.y - q.y), (a.x - q.x));
        double the2 = atan2((q.y - b.y), (q.x - b.x));

        double beta1 = inRange(2 * the - a.theta);
        double beta2 = inRange(2 * the2 - b.theta);
        q.theta = beta1;
        if (debug) {
            cout << "Intermediate Point " << q.x << " " << q.y << " " << q.theta << endl;
        }
        getPath(a, q);
        getPath(q, b);
    }
}

void Clothoid::getXY(double s, double a, double b, double c, double& x, double& y) {
    double storeX, storeY;

    if (a > 0) {
        double limit_ = (b + 2 * a * s) / (sqrt(2 * fabs(a) * PI));
        fresnel(limit_, storeX, storeY);
        x = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeX)+(sin(b * b / 4 / a - c))*(storeY));
        y = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeY)-(sin(b * b / 4 / a - c))*(storeX));

        double x1, y1;
        limit_ = (b) / (sqrt(2 * fabs(a) * PI));
        fresnel(limit_, storeX, storeY);
        x1 = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeX)+(sin(b * b / 4 / a - c))*(storeY));
        y1 = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeY)-(sin(b * b / 4 / a - c))*(storeX));
        x -= x1;
        y -= y1;
    } else {
        a = -a;
        double limit_ = (2 * a * s - b) / (sqrt(2 * fabs(a) * PI));
        fresnel(limit_, storeX, storeY);
        x = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a + c))*(storeX)+(sin(b * b / 4 / a + c))*(storeY));
        y = sqrt(PI / 2 / fabs(a))*(-(cos(b * b / 4 / a + c))*(storeY)+(sin(b * b / 4 / a + c))*(storeX));

        double x1, y1;
        limit_ = (-b) / (sqrt(2 * a * PI));
        fresnel(limit_, storeX, storeY);
        x1 = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a + c))*(storeX)+(sin(b * b / 4 / a + c))*(storeY));
        y1 = sqrt(PI / 2 / fabs(a))*(-(cos(b * b / 4 / a + c))*(storeY)+(sin(b * b / 4 / a + c))*(storeX));
        x -= x1;
        y -= y1;
    }
}

void Clothoid::getTrajectory() {
    double s = 0, tempS;
    double x, y;
    path.path.clear();
    double x0, y_zero;
    double k0 = 0, theta0 = start.theta;
    getXY(path.lengthOfPath / 2, path.sigma / 2, k0, theta0, x0, y_zero);
    double k1 = path.sigma * path.lengthOfPath / 2 + k0;
    double theta1 = path.sigma / 2 * path.lengthOfPath * path.lengthOfPath / 4 + k0 * path.lengthOfPath / 2 + theta0;
    for (s = 0; s < path.lengthOfPath; s += path.lengthOfPath / 1000) {
        tempS = s;
        if (s <= path.lengthOfPath / 2) {
            if (path.sigma * s < kMax) {
                double a = path.sigma / 2, b = k0, c = theta0;
                getXY(s, a, b, c, x, y);
            } else {
                x = sin(kMax * s + theta0) / kMax - sin(theta0) / kMax;
                y = cos(theta0) / kMax - cos(kMax * s + theta0) / kMax;
            }

            path.path.push_back(State(start.x + x, start.y + y, 0));
        } else {
            s = tempS;
            tempS = s - path.lengthOfPath / 2;

            if (path.sigma * s < kMax) {
                double a = -path.sigma / 2, c = theta1;
                double b = path.sigma * path.lengthOfPath / 2 + k0;
                double X1, Y1;

                getXY(tempS, a, b, c, X1, Y1);
                //   std::cout<<"X "<<X1<<" Y "<<Y1;

                getXY(0, a, b, c, x, y);
                X1 -= x;
                Y1 -= y;
                x = x0 + X1;
                y = y_zero + Y1;

                //  std::cout<<"Hello 3 theta "<<theta1+a*tempS*tempS+b*tempS<<" s "<<s<<" k "<<sigma*(larc/2-tempS)<<" x "<<x<<" y "<<y<<std::endl;
            } else {
                x = sin(kMax * s + theta0) / kMax - sin(theta0) / kMax;
                y = cos(theta0) / kMax - cos(kMax * s + theta0) / kMax;
            }
            path.path.push_back(State(start.x + x, start.y + y, 0));
        }
    }
}

int Clothoid::fresnel(double x, double &costerm, double &sinterm) {
    double xxa;
    double f;
    double g;
    double cc;
    double ss;
    double t;
    double u;
    double x2;
    double sn;
    double sd;
    double cn;
    double cd;
    double fn;
    double fd;
    double gn;
    double gd;
    double mpi;
    double mpio2;

    mpi = 3.14159265358979323846;
    mpio2 = 1.57079632679489661923;
    xxa = x;
    x = fabs(xxa);
    x2 = x*x;
    if (x2 < 2.5625) {
        t = x2*x2;
        sn = -2.99181919401019853726E3;
        sn = sn * t + 7.08840045257738576863E5;
        sn = sn * t - 6.29741486205862506537E7;
        sn = sn * t + 2.54890880573376359104E9;
        sn = sn * t - 4.42979518059697779103E10;
        sn = sn * t + 3.18016297876567817986E11;
        sd = 1.00000000000000000000E0;
        sd = sd * t + 2.81376268889994315696E2;
        sd = sd * t + 4.55847810806532581675E4;
        sd = sd * t + 5.17343888770096400730E6;
        sd = sd * t + 4.19320245898111231129E8;
        sd = sd * t + 2.24411795645340920940E10;
        sd = sd * t + 6.07366389490084639049E11;
        cn = -4.98843114573573548651E-8;
        cn = cn * t + 9.50428062829859605134E-6;
        cn = cn * t - 6.45191435683965050962E-4;
        cn = cn * t + 1.88843319396703850064E-2;
        cn = cn * t - 2.05525900955013891793E-1;
        cn = cn * t + 9.99999999999999998822E-1;
        cd = 3.99982968972495980367E-12;
        cd = cd * t + 9.15439215774657478799E-10;
        cd = cd * t + 1.25001862479598821474E-7;
        cd = cd * t + 1.22262789024179030997E-5;
        cd = cd * t + 8.68029542941784300606E-4;
        cd = cd * t + 4.12142090722199792936E-2;
        cd = cd * t + 1.00000000000000000118E0;

        sinterm = signum(xxa) * x * x2 * sn / sd;
        costerm = signum(xxa) * x * cn / cd;
        return 0;
    }
    if (x > 36974.0) {
        costerm = signum(xxa)*0.5;
        sinterm = signum(xxa)*0.5;
        return 0;
    }
    x2 = x*x;
    t = mpi*x2;
    u = 1 / (t * t);
    t = 1 / t;
    fn = 4.21543555043677546506E-1;
    fn = fn * u + 1.43407919780758885261E-1;
    fn = fn * u + 1.15220955073585758835E-2;
    fn = fn * u + 3.45017939782574027900E-4;
    fn = fn * u + 4.63613749287867322088E-6;
    fn = fn * u + 3.05568983790257605827E-8;
    fn = fn * u + 1.02304514164907233465E-10;
    fn = fn * u + 1.72010743268161828879E-13;
    fn = fn * u + 1.34283276233062758925E-16;
    fn = fn * u + 3.76329711269987889006E-20;
    fd = 1.00000000000000000000E0;
    fd = fd * u + 7.51586398353378947175E-1;
    fd = fd * u + 1.16888925859191382142E-1;
    fd = fd * u + 6.44051526508858611005E-3;
    fd = fd * u + 1.55934409164153020873E-4;
    fd = fd * u + 1.84627567348930545870E-6;
    fd = fd * u + 1.12699224763999035261E-8;
    fd = fd * u + 3.60140029589371370404E-11;
    fd = fd * u + 5.88754533621578410010E-14;
    fd = fd * u + 4.52001434074129701496E-17;
    fd = fd * u + 1.25443237090011264384E-20;
    gn = 5.04442073643383265887E-1;
    gn = gn * u + 1.97102833525523411709E-1;
    gn = gn * u + 1.87648584092575249293E-2;
    gn = gn * u + 6.84079380915393090172E-4;
    gn = gn * u + 1.15138826111884280931E-5;
    gn = gn * u + 9.82852443688422223854E-8;
    gn = gn * u + 4.45344415861750144738E-10;
    gn = gn * u + 1.08268041139020870318E-12;
    gn = gn * u + 1.37555460633261799868E-15;
    gn = gn * u + 8.36354435630677421531E-19;
    gn = gn * u + 1.86958710162783235106E-22;
    gd = 1.00000000000000000000E0;
    gd = gd * u + 1.47495759925128324529E0;
    gd = gd * u + 3.37748989120019970451E-1;
    gd = gd * u + 2.53603741420338795122E-2;
    gd = gd * u + 8.14679107184306179049E-4;
    gd = gd * u + 1.27545075667729118702E-5;
    gd = gd * u + 1.04314589657571990585E-7;
    gd = gd * u + 4.60680728146520428211E-10;
    gd = gd * u + 1.10273215066240270757E-12;
    gd = gd * u + 1.38796531259578871258E-15;
    gd = gd * u + 8.39158816283118707363E-19;
    gd = gd * u + 1.86958710162783236342E-22;
    f = 1 - u * fn / fd;
    g = t * gn / gd;
    t = mpio2*x2;
    cc = cos(t);
    ss = sin(t);
    t = mpi*x;
    costerm = 0.5 + (f * ss - g * cc) / t;
    sinterm = 0.5 - (f * cc + g * ss) / t;
    costerm = costerm * signum(xxa);
    sinterm = sinterm * signum(xxa);
    return 0;
}
