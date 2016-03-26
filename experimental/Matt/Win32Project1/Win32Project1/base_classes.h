#pragma once
#include <string>
#include "vec2d.h"
#include "constants.h"
#include <list>
#include "dependency.h"
#include <queue>

using namespace simvals;
const enum state_type {/*for roomba*/NOSTATE, REVERSE, RUN, RANDROT, TOPTOUCH, STOPPED, OBSTACLERUN, OBSTACLESTOPPED, OUTOFPLAY,
					   /*for quad*/ BUMP, FLY, TAP, ASCEND, HOVER, FLYTHROUGH, ABORT};
const enum notification { /*for roomba*/ TOUCHED, REVERSAL, NOISE, MAGNET, ENDED, TOUCHENDED, EXITED,
						  /*for quad*/ ROOMBASPOTTED,
						  /*for management*/ INVALIDATED};

class collision_detection;
class pqueue_index;
class roomba_action;

struct rgb {
	double r = 0;
	double g = 0;
	double b = 0;
};

struct notifiable {
	virtual state_type notify(double at_time, pqueue_index info) = 0;
	virtual void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) = 0;
	virtual bool isQuad() = 0;
};

class robot_state : public notifiable, public linkable { //I've been writing too much java can someone please send a search and rescue party
protected:
	std::list<pqueue_index*> events;
public:
	double start_time;
	double end_time;
	virtual double getHeight(double at_time) = 0;
	virtual vec2d getPosition(double at_time) = 0;
	virtual double getAngle(double at_time) = 0;
};


class robot : public notifiable, public linkable {
protected:
	
	robot_state * current_state = nullptr;
	robot_state * start_state = nullptr;
	std::set<robot_state*> dependents;
	
public:
	virtual double getHeight(double at_time) = 0;
	virtual vec2d getPosition(double at_time) = 0;
	virtual double getAngle(double at_time) = 0;
	robot_state * getCurrentState() { return current_state; }
	robot_state * getStartState() { return start_state; }
	robot_state * getStateAt(double time);
	rgb color;
	virtual ~robot() { delete start_state; }
};

class quad : public robot {
public:
	state_type notify(double at_time, pqueue_index info);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	double getHeight(double at_time);
	vec2d getPosition(double at_time);
	double getAngle(double at_time);
	rgb color;
};

class roomba : public robot {
protected:
	robot_state * saved_state = nullptr;
	collision_detection * collider;
public:
	robot_state * getState();
	void setState(robot_state * a);
	state_type notify(double at_time, pqueue_index info);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	double getHeight(double at_time); 
	vec2d getPosition(double at_time);
	double getAngle(double at_time);
	bool isQuad() { return false; }
	int ID;
	roomba(state_type my_state, double s_time, vec2d spos, double init_an,int i);
	void spawn_state(double at_time, state_type st, roomba_action * parent,roomba_action * trigger);
};
