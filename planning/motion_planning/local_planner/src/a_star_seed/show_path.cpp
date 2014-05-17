
#include "a_star_seed/a_star_seed.hpp"

namespace navigation {

    void quickReflex::showPath(std::vector<State>& path,const State&  startState,const State&  targetState) {
        
        cv::circle(fusionMap, cvPoint(targetState.x(),fusionMap.rows-1-targetState.y()), 5, cvScalar(128),-1);
        cv::line(fusionMap, cvPoint(targetState.x(),fusionMap.rows-1-targetState.y()), cvPoint(targetState.x()+15*cos((targetState.theta()*M_PI)/180),fusionMap.rows-1-targetState.y()-15*sin((targetState.theta()*M_PI)/180)),cvScalar(128),1,8,0);
        cv::circle(fusionMap, cvPoint(startState.x(),fusionMap.rows-1-startState.y()), 5, cvScalar(128),-1);
        cv::line(fusionMap, cvPoint(startState.x(),fusionMap.rows-1-startState.y()), cvPoint(startState.x()+15*cos((startState.theta()*M_PI)/180),fusionMap.rows-1-startState.y()-15*sin((startState.theta()*M_PI)/180)),cvScalar(128),1,8,0);
        std::cout<<"Showing A Path\n";

        std::cout<<"Path size in showPath is : " <<(int)path.size()<<" \n";

        for (std::vector<State>::iterator stateIt = path.begin(); stateIt != path.end() ; ++stateIt) {
            
            int Xcordinate = stateIt->x() + startState.x();
            int Ycordinate = stateIt->y() + startState.y();
            
            cv::circle(fusionMap, cv::Point(Xcordinate, fusionMap.rows- Ycordinate - 1), 3, cv::Scalar(255), -1);
        }

        cv::imshow("[PLANNER] Map", fusionMap);
        cvWaitKey(0);
    }
}
