#pragma once
#include "base_classes.h"

struct pqueue_index { //a notification
private:
	char deps; //char is a convienent way to get a byte-sized number
public:
	robot_state * caller; //who we'll notify
	robot_state * linked; //the possible bot that
	notification type;
	notifiable * to_notify;
	double at_time;
	pqueue_index(robot_state * call, robot_state * link, notification ty, double t, notifiable * to_notify) : caller(call), linked(link), type(ty), at_time(t) {
		//quad_notify = false;
		deps = (call != nullptr) + (link != nullptr) + 1; //evaluate dependencies
	}
	bool operator<(const pqueue_index & a) {
		return at_time > a.at_time;
	}
	void removeDep() {
		--deps;
		if (deps == 0) {
			delete this; //this is a legit thing that you can do I promise
		}
	}
};

struct actionval {
	int roomba = 0;
	state_type action = HOVER;
	double score = 0;
	//int special = 0;
	bool operator<(actionval & other) {
		return score < other.score;
	}
	actionval(double a, state_type b) : roomba(a), action(b) {}
	actionval() {
		action = HOVER;
		score = 0;
		roomba = 0;
	}
};

