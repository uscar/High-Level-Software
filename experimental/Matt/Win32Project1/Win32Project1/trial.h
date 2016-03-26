#pragma once
#include "renderer.h"
#include "roomba_action.h"
#include "collision_detection.h"
#include <queue>
#include <vector>
#include "evaluation_metric.h"

class trial {
public:
	std::vector<actionval> actions;
	void evaluatetime2(double until);
	void show(double until);
	void showBestmove();
	bool simulating = false;
	std::priority_queue<pqueue_index> organizer;
	collision_detector collider;
	trial(evaluation_metric * myMetric);
	~trial();
	roomba * bots[14];
	evaluation_metric * metric;
	//roomba_action * currbots[14];
	//void addBot(roomba_action * bot, int i);
	//void addQuad(quad_action * bot);
	bool clist[14][14];
	void detect_collisions(roomba_action * state);
	void collision_handle();
	double currtime = 0;
	double currquadtime = 0;
	void evaluatenext();
	void evaluatetime(double until);

	void reverttime(double until);
	std::vector<actionval> * bestMove(evaluation_metric & a, double until, actionval * lastMove);
	std::vector<actionval> * getActions(double time);
	quad * copter; 
	//quad_action * startquad; //this trial's quad actions
	//quad_action * currquad; //this trial's quad action

};