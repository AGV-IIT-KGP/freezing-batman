//heuristic merger that normalises, checks if admissible and consistent

//Think of functions and add to this
//class. We can change their implementation later.
#ifndef __Lattice_Heuristic__ 
#define __Lattice_Heuristic__

#include "LState.hpp"

namespace navigation {
	class Heuristic {
		LState startState_, endState_;
		double h_cost_dt_,h_cost_pl_,h_cost_ta_;
		double hCostNormalised;
	public:
		double hCostDT(const LState& startState_,const LState& endState_);
		double hCostPathLength(const LState& startState_,const LState& endState_);//depends only on x and y coordinates
		double hCostTurnAngle(const LState& startState_,const LState& endState_);//depends on theta, and indirectly takes curvature also into account.
		double heuristicNormalise(double, double, double);//takes all the heuristics and normalises them.
	};
}

#endif //__Lattice_Heuristic__