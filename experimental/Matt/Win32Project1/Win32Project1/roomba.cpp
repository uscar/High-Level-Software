#pragma once
#include "base_classes.h"
#include "roomba_subclass_defs.h"

void roomba::spawn_state(double at_time, state_type st, roomba_action * parent, roomba_action * trigger) {
	switch (st) {
	case RUN:
		current_state = new roomba_run(at_time, parent, st);
		break;
	case REVERSE:
		current_state = new roomba_reverse(at_time, parent, st, trigger);
		break;
	case RANDROT:
		current_state = new roomba_randrot(at_time, parent, st);
		break;
	case OBSTACLERUN:
		current_state = new roomba_obstaclerun(at_time, parent, st);
		break;
	case TOPTOUCH:
		current_state = new roomba_reverse(at_time, parent, st, trigger);
		break;
	case OBSTACLESTOPPED:
		current_state = new roomba_obstaclestopped(at_time, parent, st);
		break;
	default:
		break;
	}
}

void roomba::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	if (current_state != nullptr) {
		current_state->add_notifications(organizer, this);
	}
}

robot_state * roomba::getState() {
	saved_state = current_state;
}

void roomba::setState(robot_state * a) {
	current_state = a;
	if (((roomba_action *)a)->child != nullptr) {
		delete ((roomba_action *)a)->child;
		((roomba_action *)a)->child = nullptr;
	}
}

roomba::roomba(state_type my_state, double s_time, vec2d spos, double init_an,int i) {
	ID = i;
	if (my_state == RUN) {
		current_state = new roomba_run(s_time,spos,init_an);
	}
	if (my_state == OBSTACLERUN) {
		current_state = new roomba_obstaclerun(s_time, spos, init_an);
	}
	start_state = current_state;
}

state_type roomba::notify(double at_time, pqueue_index info) {
	roomba_action * callstate = (roomba_action*)info.caller;
	state_type generated = callstate->notify(at_time, info);
	spawn_state(at_time, generated, (roomba_action *)info.caller, (roomba_action *)info.linked);
	return generated;
}
