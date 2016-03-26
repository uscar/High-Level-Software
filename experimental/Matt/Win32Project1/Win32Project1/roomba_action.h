#pragma once
#include <queue>
#include <list>
#include <set>
#include "ancillary_classes.h"

class roomba_action : public robot_state {
public:
	virtual state_type notify(double at_time, pqueue_index type) = 0;
	roomba_action * parent = nullptr;
	roomba_action * child = nullptr;

	roomba * container;

	state_type mytype;

	double next_noise;
	double next_reversal;
	virtual bool isObstacle() { return false; }
	double rw_v = 0;
	double lw_v = 0;
	
	vec2d startpos;
	//vec2d endpos; //to ease calculation
	double initial_angle;

	std::set<roomba_action*> dependents;
	std::list<pqueue_index*> events;
	roomba_action * dependency = nullptr;

	//double final_angle;
	
	struct savedValues {
		double R;
		double wd_init;
		double wd;
		double sinv, cosv;
		double & rwv, &lwv;
		vec2d vec1;
		vec2d & start;
		double & init_ang;
		bool condition;
		savedValues(roomba_action * act);
		void init();
		vec2d doCalculation(double dt);
	};
	savedValues * saved;
	double getHeight(double at_time);
	vec2d getPosition(double at_time);
	double getAngle(double at_time);
	bool isQuad();
	void resetHere();
	bool is_touching(roomba_action * r,double at_time); //are we touching them at this time
	double does_collide(roomba_action * r, double epoch = 5.0, double t0 = 0, double res = .01); //are we going to collide with them at that time
	virtual void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) = 0;
	roomba_action(double at_time, roomba_action * par, state_type type);
	roomba_action(double s_time, vec2d spos, double init_an, roomba * contain);
	virtual ~roomba_action();
};

