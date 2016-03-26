#pragma once
#include "roomba_action.h"
#include <queue>

class roomba_reverse : public roomba_action {
public:
	roomba_reverse(double at_time, roomba_action * par, state_type type, robot_state * trigger);
	state_type notify(double at_time, pqueue_index info);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
};

class roomba_run : public roomba_action {
public:
	roomba_run(double at_time, roomba_action * par, state_type type);
	roomba_run(double s_time, vec2d spos, double init_an);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	state_type notify(double at_time, pqueue_index info);
};

/* TODO: this
class roomba_outofplay : public roomba_action {
public:
	//roomba_outofplay(double at_time, roomba_action * par, state_type type);
	//void state_register();
	void add_notifications(std::priority_queue<pqueue_index> * organizer);
	state_type notify(double at_time, pqueue_index info);
};
*/

class roomba_toptouch : public roomba_action {
public:
	roomba_toptouch(double at_time, roomba_action * par, state_type type);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	state_type notify(double at_time, pqueue_index info);
};

class roomba_obstaclerun : public roomba_action {
public:
	bool isObstacle() { return true; }
	roomba_obstaclerun(double at_time, roomba_action * par, state_type type);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	roomba_obstaclerun(double s_time, vec2d spos, double init_an);
	state_type notify(double at_time, pqueue_index info);
};

class roomba_obstaclestopped : public roomba_action {
public:
	bool isObstacle() { return true; }
	roomba_obstaclestopped(double at_time, roomba_action * par, state_type type);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	state_type notify(double at_time, pqueue_index info);
};

class roomba_stopped : public roomba_action {
public:
	roomba_stopped(double at_time, roomba_action * par, state_type type);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	state_type notify(double at_time, pqueue_index info);
};

class roomba_randrot : public roomba_action {
public:
	roomba_randrot(double at_time, roomba_action * par, state_type type);
	void add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener);
	state_type notify(double at_time, pqueue_index info);
};

