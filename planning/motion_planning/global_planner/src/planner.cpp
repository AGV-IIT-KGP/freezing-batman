#include "planner.h"
#include "plannerMethods.h"

using namespace cv;
namespace planner_space {

    /*geometry_msgs::Twist kTurn() {
        seed reverse;
        double max = 30;
        switch (last_cmd) {
            case RIGHT_CMD:
                // Take reverse right turn
                reverse.vl = -max;
                reverse.vr = 0;
                break;
            case LEFT_CMD:
                // Take reverse left turn
                reverse.vl = 0;
                reverse.vr = -max;
                break;
        }

        return sendCommand(reverse);
    }
    */
    void Planner::loadPlanner() {
        loadSeeds();
        ROS_INFO("[PLANNER] Seeds Loaded");

        initBot();
        ROS_INFO("[PLANNER] Vehicle Initiated");
    }

    std::vector<Triplet> Planner::findPath(Triplet bot, Triplet target, Mat data_img) {
        state start, goal;

        start.pose = bot;
        start.seed_id = -1;
        start.g_dist = 0;
        start.h_dist = distance(bot, target);
        start.g_obs = 0;
        start.h_obs = 0;
        start.depth = 0;

        goal.pose = target;
        goal.g_dist = 0;
        goal.h_dist = 0;
        goal.seed_id = 0;
        goal.g_obs = 0;
        goal.h_obs = 0;

        vector<state> open_list;
        open_list.insert(open_list.begin(), start);
        map<Triplet, open_map_element, PoseCompare> open_map;
        open_map[start.pose].membership = OPEN;
        open_map[start.pose].cost = start.g_dist;
        std::vector<Triplet> cmdvel;
        map<Triplet, state, PoseCompare> came_from;

        brake.vl = brake.vr = 0;
        //	leftZeroTurn.vl=-15;leftZeroTurn.vr=15;
        //	rightZeroTurn.vl=15;rightZeroTurn.vr=-15;
        //
        //	if(target.y<100)
        //	{
        //		if(target.x<500)
        //		{
        //		precmdvel=sendCommand(leftZeroTurn);
        //		return precmdvel;
        //		}
        //		if(target.x>=500)
        //		{
        //		precmdvel=sendCommand(rightZeroTurn);
        //		return precmdvel;
        //		}
        //	}
        if (isEqual(start, goal)) {
            ROS_INFO("[PLANNER] Target Reached");
            Planner::finBot();
            return cmdvel;
        }

        int iterations = 0;
        while (!open_list.empty()) {
            //TODO: This condition needs to be handled in the strategy module.
            if (local_map[start.pose.x][start.pose.y] > 0) {
                ROS_WARN("[PLANNER] Robot is in Obstacles");
                Planner::finBot();
                return cmdvel;
            }

            state current = open_list.front();

#ifdef DEBUG
            cout << "==> CURRENT: ";
            print(current);

            plotPoint(data_img, current.pose);
            cv::imshow("[PLANNER] Map", data_img);

            cvWaitKey(0);
#endif

            if ((open_map.find(current.pose) != open_map.end()) &&
                    (open_map[current.pose].membership == CLOSED)) {
                pop_heap(open_list.begin(), open_list.end(), StateCompare());
                open_list.pop_back();
                continue;
            }

            if (isEqual(current, goal)) {
                cmdvel = reconstructPath(came_from, current, data_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");

                cv::imshow("[PLANNER] Map", data_img);

                cvWaitKey(0);
#endif
                //                Mat hoho;
                //                resize(data_img, hoho, cvSize(400, 400));
#ifdef SHOW_PATH

                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(WAIT_TIME);
#endif
                closePlanner();
                //		precmdvel=cmdvel;
                return cmdvel;
            }

            if (onTarget(current, goal)) {
                cmdvel = reconstructPath(came_from, current, data_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");
                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(0);
#endif

#ifdef SHOW_PATH

                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(WAIT_TIME);
#endif
                closePlanner();
                //		precmdvel=cmdvel;
                return cmdvel;
            }

            pop_heap(open_list.begin(), open_list.end(), StateCompare());
            open_list.pop_back();
            open_map[current.pose].membership = UNASSIGNED;
            open_map[current.pose].cost = -1;

            open_map[current.pose].membership = CLOSED;

            vector<state> neighbors = neighborNodes(current);

            for (unsigned int i = 0; i < neighbors.size(); i++) {
                state neighbor = neighbors[i];

#ifdef DEBUG
                //plotPoint(grayImg,neighbor.pose);
#endif

                if (!(((neighbor.pose.x >= 0) && (neighbor.pose.x < MAP_MAX)) &&
                        ((neighbor.pose.y >= 0) && (neighbor.pose.y < MAP_MAX)))) {
                    continue;
                }

                if (!isWalkable(current, neighbor)) {
                    continue;
                }

                double tentative_g_score = neighbor.g_dist + current.g_dist;
                double admissible = distance(neighbor.pose, goal.pose);
                double consistent = admissible;

                // If already in open_list update cost and came_from if cost is less
                // This update is implicit since we allow duplicates in open list
                if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                        (open_map[neighbor.pose].membership == OPEN))) {
                    came_from[neighbor. pose] = current;
                    neighbor.g_dist = tentative_g_score;
                    neighbor.h_dist = consistent;

                    //next condition is always true
                    if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                            (open_map[neighbor.pose].membership == OPEN))) {
                        open_list.push_back(neighbor);
                        push_heap(open_list.begin(), open_list.end(), StateCompare());
                        open_map[neighbor.pose].membership = OPEN;
                        open_map[neighbor.pose].cost = neighbor.g_dist;
                    }
                }
            }

            iterations++;
            if (iterations > MAX_ITER) {
                ROS_WARN("[PLANNER] Open List Overflow");
                Planner::finBot();
                return cmdvel;
            }
        }

        ROS_ERROR("[PLANNER] No Path Found");
        closePlanner();
        //return  array of pose3d
    }

   

    void Planner::finBot() {
        // sendCommand(brake);
    }
}
