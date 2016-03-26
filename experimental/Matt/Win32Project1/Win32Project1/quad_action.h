#pragma once
#include "vec2d.h"
#include "roomba_action.h"
class quad_action : public robot_state {
public:
	double dx;
	double dy;
	double dh;
	double startheight; //because why not
	double start_time;
	double end_time;
	double orientation; //unused currently
	actionval goal_action;
	vec2d startpos;
	quad_action * parent;
	quad_action * child;
	quad_action(double end, vec2d velocity, quad_action * prior, roomba_action * track, state_type t);
	quad_action();
	roomba_action * tracking;
	//void append();
	state_type mytype;
	bool executeAction(actionval & a, double until);
	bool flyTo(int roomba, double until);
	double simpleTimeEst(actionval & a, double time);
	double quadParamLineIntersect(vec2d roombaDisplacement, vec2d roombaVelocity, double quadVelocity);
	vec2d getPosition(double at_time);
	double getAngle(double at_time);
	double getHeight(double at_time);
	//void spawn_state(double at_time, quad_state_type st);
	~quad_action();
};
