/* 
 * File:   Clothoid.cpp
 * Author: satya
 * 
 * Created on December 13, 2013, 6:03 PM
 */

#include "Clothoid.hpp"

namespace navigation {


    Clothoid::Clothoid(const Clothoid& orig) {
    }

    Clothoid::~Clothoid() {
    }

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
        img = cv::Mat(cvSize(HEIGHT, WIDTH), CV_8UC3, cvScalarAll(0));
        cv::namedWindow("[road_navigation] : clothoid", 0);
    }

    int Clothoid::signum(double a) {
        if (a < 0)
            return -1;
        else if (a > 0)
            return 1;
        else
            return 0;
    }

    double Clothoid::calc_d(double alpha) {
        double x, z;

        fresnel(sqrt(2 * alpha / PI), x, z);

        double d = cos(alpha) * (x)+ (sin(alpha) * z);
        return d;
    }

    double Clothoid::actualAtan(double theta, State a, State b) {
        if ((b.x - a.x) < 0) {
            if ((b.y - a.y) < 0) {
                theta = PI + theta;
            } else {
                theta += PI;
            }
        } else if (theta < 0) {
            theta += 2 * PI;
        }

        return theta;
    }

    /*void Clothoid::print_circle(State a,State b,State c,State d,int circle)
    {
	           
        cv::circle(img, cvPoint(a.x, HEIGHT- a.y), 5, cvScalar(0,0,255));
        cv::circle(img, cvPoint(b.x, HEIGHT- b.y), 5, cvScalar(0,255,255));
        cv::circle(img, cvPoint(c.x, HEIGHT- c.y), 5, cvScalar(255,255,0));
        cv::circle(img, cvPoint(d.x, HEIGHT- d.y), 5, cvScalar(255,255,255));
            if(circle==1)
        cv::circle(img, cvPoint(c.x, HEIGHT- c.y), sqrt(c.distance(a)), cvScalar(255,255,255));

        cv::imshow("hello", img);
        cvWaitKey(0);
    }*/
    void Clothoid::getControls(State a, State b) {
        a.theta = inRange(a.theta);
        b.theta = inRange(b.theta);

        start = a;
        end = b;
        double alpha = inRange((-start.theta + end.theta)) / 2;

        double D = calc_d(fabs(alpha));
        sigma = 4 * PI * signum(alpha) * D * D / start.distance(end);
        larc = 2 * sqrt(fabs(2 * alpha / sigma));
    }

    std::vector<PathSegment*> Clothoid::getPath(State a, State b) {

        getControls(a, b);
        getTrajectory();
        PathSegment* clothoidPath = new ClothoidPath(path, sigma, larc);
        std::vector<PathSegment*> pathVector;
        pathVector.push_back(clothoidPath);
        return pathVector;
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
            // std::cout<<"Limit "<<limit_<<std::endl;

        } else {
            a = -a;
            double limit_ = (2 * a * s - b) / (sqrt(2 * fabs(a) * PI));
            fresnel(limit_, storeX, storeY);
            // std::cout<<"Limit "<<limit_<<std::endl;


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
        k0 = 0, theta0 = start.theta;
        path.clear();
        getXY(larc / 2, sigma / 2, k0, theta0, x0, y0);
        double k1 = sigma * larc / 2 + k0;
        double theta1 = sigma / 2 * larc * larc / 4 + k0 * larc / 2 + theta0;

        for (s = 0; s < larc; s += larc / 1000) {
            tempS = s;
            if (s <= larc / 2) {
                if (sigma * s < kMax) {
                    double a = sigma / 2, b = k0, c = theta0;
                    getXY(s, a, b, c, x, y);
                }
                else {
                    x = sin(kMax * s + theta0) / kMax - sin(theta0) / kMax;
                    y = cos(theta0) / kMax - cos(kMax * s + theta0) / kMax;
                }
                path.push_back(State(start.x + x, start.y + y, 0));
            } else {
                s = tempS;
                tempS = s - larc / 2;
                if (sigma * s < kMax) {
                    double a = -sigma / 2, c = theta1;
                    double b = sigma * larc / 2 + k0;
                    double X1, Y1;
                    getXY(tempS, a, b, c, X1, Y1);
                    getXY(0, a, b, c, x, y);
                    X1 -= x;
                    Y1 -= y;
                    x = x0 + X1;
                    y = y0 + Y1;
                } else {
                    x = sin(kMax * s + theta0) / kMax - sin(theta0) / kMax;
                    y = cos(theta0) / kMax - cos(kMax * s + theta0) / kMax;
                }
                path.push_back(State(start.x + x, start.y + y, 0));
            }
        }
    }

    std::vector<PathSegment*> Clothoid::drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose) {
        State a(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        State b(target_pose.position.x, target_pose.position.y, target_pose.position.z);
        std::vector<State> path;
        Clothoid curve;
        nav_msgs::Path paths;
        double beta = atan((b.y - a.y) / (b.x - a.x));
        std::vector<PathSegment*> output;

        //printing the parameters.
        ROS_DEBUG("start point : (%lf, %lf, %lf)", a.x, a.y, a.theta);
        //std::cout << "end point : (" << b.x << "," << b.y << "," << b.theta << ")" << std::endl << std::endl;
        //std::cout << "Angle between the start and end points : " << beta << std::endl << std::endl;

        //if the symmetry condition is specified we get an elementary Euler curve--- Ref: A star Spatial
        if (fabs(beta - b.theta - a.theta + beta) < 0.001) {
            curve.getControls(a, b);
            curve.getTrajectory();
            path = curve.plotPath();

            PathSegment* pSegment1 = new ClothoidPath(path, sigma, larc);

            output.push_back(pSegment1);
            //std::cout << std::endl << std::endl << std::endl;
        } else if (a.theta == b.theta) { //when the start and end points have the same direction vector.
            //2 euler curves via point p.
            State p((a.x + b.x) / 2, (a.y + b.y) / 2, 0);
            double beta = atan((p.y - a.y) / (p.x - a.x));
            p.theta = 2 * beta - a.theta;
            //cout << p.theta << endl;

            //Plotting Euler curves between a&p && p&b.
            curve.getControls(a, p);
            curve.getTrajectory();
            path = curve.plotPath();
            PathSegment* pSegment1 = new ClothoidPath(path, sigma, larc);

            curve.getControls(p, b);
            curve.getTrajectory();

            std::vector<State> another_path = curve.plotPath();
            PathSegment* pSegment2 = new ClothoidPath(another_path, sigma, larc);

            output.push_back(pSegment1);
            output.push_back(pSegment2);
        } else { //between any 2 arbitrary points and direction.
            double alpha = ((-a.theta + b.theta)) / 2;
            double cot_alpha;
            cot_alpha = cos(alpha) / sin(alpha);
            State center(0, 0, 0); //center of the circle--which is the locus of the point q between which 2 eulers are drawn.

            //cout << "alpha is " << alpha << endl;
            center.x = (a.x + b.x + cot_alpha * (a.y - b.y)) / 2;
            center.y = (a.y + b.y + cot_alpha * (b.x - a.x)) / 2;
            double r = sqrt(center.distance(a)); //radius of the circle.
            //cout << "centre " << center.x << " " << center.y << endl;
            double deflection1 = actualAtan(atan((center.y - a.y) / (center.x - a.x)), center, a);
            double deflection2 = actualAtan(atan((center.y - b.y) / (center.x - b.x)), center, b);

            //cout << "deflection between a and center" << deflection1 << "deflection between b and center" << deflection2 << endl;
            if (deflection2 < deflection1) {
                //swap
                double temp = deflection2;
                deflection2 = deflection1;
                deflection1 = temp;
            }
            double def = ((deflection2 - deflection1) / 2) + deflection1;

            State q(0, 0, 0);
            q.x = center.x + r * cos(def);
            q.y = center.y + r * sin(def);

            //cout << endl << endl << "q has diff in distance : " << (center.distance(a) - center.distance(q)) << endl;
            double theta1 = atan((q.y - a.y) / (q.x - a.x));
            double theta2 = atan((q.y - b.y) / (q.x - b.x));
            double beta1 = 2 * theta1 - a.theta;
            double beta2 = 2 * theta2 - b.theta;
            //cout << "path should be smooth enough " << beta1 - beta2 << endl;
            q.theta = beta1;
            //cout << "q is " << q.x << "," << q.y << ", " << q.theta << endl;
            curve.getControls(a, q);

            curve.getTrajectory();
            path = curve.plotPath();

            curve.getControls(q, b);

            curve.getTrajectory();
            PathSegment* pSegment1 = new ClothoidPath(path, sigma, larc);
            std::vector<State> another_path = curve.plotPath();
            PathSegment* pSegment2 = new ClothoidPath(another_path, sigma, larc);

            output.push_back(pSegment1);
            output.push_back(pSegment2);
        }


        return output;
    }

    std::vector<State> Clothoid::plotPath() {

        cv::circle(img, cvPoint(start.x, HEIGHT - start.y), 5, cvScalarAll(255));
        cv::circle(img, cvPoint(end.x, HEIGHT - end.y), 5, cvScalarAll(255));
        cv::line(img, cvPoint(start.x, HEIGHT - start.y), cvPoint(end.x, HEIGHT - end.y), cvScalar(255));
        for (int i = 0; i < path.size() - 1; i++) {
            cv::line(img, cvPoint(path[i].x, HEIGHT - path[i].y), cvPoint(path[i + 1].x, HEIGHT - path[i + 1].y), cvScalar(255, 255, 255));
        }

        cv::imshow("[road_navigation] : clothoid", img);
        cvWaitKey(0);
        return path;

    }

    /* @Brief : Calculates the fresnel integral of x using trapezoidal
     * 			method of numerical integration
     * 			Cos term = integration of cos(s^2) with 0 < s < x
     * 			Sin term = integration of sin(s^2) with 0 < s < x
     *
     * @params  [in] x       : The variable whose fresnel integral is to be found
     * @params [out] costerm : The value of the cos term of the integration
     * @params [out] sinterm : The value of the sin term of the integration
     */

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

}