#include "roomba_subclass_defs.h"

roomba_obstaclerun::roomba_obstaclerun(double s_time, vec2d spos, double init_an) : roomba_action(s_time, spos, init_an, curr_test, i) {
	mytype = OBSTACLERUN;
	next_noise = runendtime;
	next_reversal = runendtime;
	rw_v = robotSpeed - (double)9 / (double)1000;
	lw_v = robotSpeed + (double)9 / (double)1000;
	saved->init();
}

roomba_obstaclerun::roomba_obstaclerun(double at_time, roomba_action * par, state_type type) : roomba_action(at_time, par, type) {
	mytype = OBSTACLERUN;
	rw_v = robotSpeed - (double)9 / (double)1000;
	lw_v = robotSpeed + (double)9 / (double)1000;
	saved->init();
}

void roomba_obstaclerun::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	end_time = start_time + 5.0;
	organizer->push(pqueue_index(this, nullptr, ENDED, end_time, listener));
}


state_type roomba_obstaclerun::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
		case TOUCHED:
			if ((info.linked != nullptr) && (info.linked->end_time >= at_time)) {
				return OBSTACLESTOPPED;
			}
			break;
		case ENDED:
			return OBSTACLERUN;
			break;
		}
	}
	return NOSTATE;
}
