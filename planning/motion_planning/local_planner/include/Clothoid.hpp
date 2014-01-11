/* 
 * File:   Clothoid.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 6:03 PM
 */

#ifndef CLOTHOID_HPP
#define	CLOTHOID_HPP

#include <cmath>
static const double PI = M_PI;
static const int HEIGHT = 800, WIDTH = 800;
using namespace std;

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <State.hpp>
#include <PathSegment.hpp>
#include <Trajectory.hpp>

namespace navigation {
    inline int signum(int x) { return (x>0)-(x<0);}
    int fresnel(double x, double &costerm, double &sinterm);

    enum Orientation    {
        arbit=-1,symmetric=0,parallel=1
    };

    class ClothoidPath : public PathSegment {
    public:
        double sigma;

        ClothoidPath(std::vector<State>& sites_, double sigma_, double larc_) : PathSegment(sites_, larc_),sigma(sigma_) {}
    };

    class ClothoidState : public State  {
    public:
        ClothoidState(double x_, double y_, double theta_, double curvature_) : State(x_, y_, theta_, curvature_)  {}
        ClothoidState(double x_, double y_, double theta_) : State(x_, y_, theta_)  {}

        inline  double beta(const ClothoidState& b)   const  {  return atan((b.y - y) / (b.x - x)); }
        static  double inRange( double theta)   {
            if (theta > PI)  while (theta > PI)     theta -= 2 * PI;
            else             while (theta <= -PI)   theta += 2 * PI;
            return theta;
        }
        inline double alpha(const ClothoidState& b) const   {  std::cout<<"a.theta "<<theta<<"inrange(theta) "<<inRange(theta)<<"b.theta "<<b.theta<<"inRange(b.theta "<<inRange(b.theta)<<std::endl;
            return  inRange((-inRange(theta) + inRange(b.theta))) / 2; }
        Orientation orientationTo(const ClothoidState& b) const {  if(theta==b.theta) return Orientation(parallel);
            else if( (2*(this->beta(b))-theta-b.theta) < 0.001 ) return Orientation(symmetric) ;
            else return Orientation(arbit) ;     
        } 
      
        double getD(const ClothoidState& b) const  {
            double alpha=fabs(this->alpha(b));
            double x, y;
            fresnel(sqrt(2 * alpha / PI), x, y);
            double D = cos(alpha) * (x)+ (sin(alpha) * y);
            return D;
        }
        double getD(const double alpha)  const {
            double x, y;
            fresnel(sqrt(2 * fabs(alpha) / PI), x, y);
            std::cout<<"X "<<x<<"Y "<<y<<std::endl;
            double D = cos(alpha) * (x)+ (sin(alpha) * y);
            return D;
        }

        double getSigma(const ClothoidState& b)  const  {

            double alpha=this->alpha(b);
            double D = this->getD(alpha);
            std::cout<<"D "<<D<<std::endl;
            return (4 * PI * signum(alpha) * D * D / this->distanceSq(b));
        }


        inline double getArcLength(double alpha, double sigma) const { return   2 * sqrt(fabs(2 * alpha / sigma)); }

    };

    class Clothoid : public Trajectory  {
    public:
        double x0, y0;
        double kMax, theta, theta_, k0, theta0;
        cv::Mat img;
        Clothoid();
        void getXY(double s, double a, double b, double c, double& x, double& y);
        std::vector<PathSegment*> drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose);
        void plotClothoid(State a, State b);
        vector<State>& getTrajectory(ClothoidState start,ClothoidState end,double sigma,double larc);
        PathSegment* getPathSegment(ClothoidState a, ClothoidState b);
        void plotPath(ClothoidState a, ClothoidState b,std::vector<State>& path);
    };
}

#endif	/* CLOTHOID_HPP */

