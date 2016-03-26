#include "roomba_subclass_defs.h"

roomba_toptouch::roomba_toptouch(double at_time, roomba_action * par, state_type type) : roomba_action(at_time, par, type) {
	mytype = TOPTOUCH;
	rw_v = -robotSpeed / 2;
	lw_v = robotSpeed / 2;
	saved->init();
}

void roomba_toptouch::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	end_time = start_time + reverseLength / 4;
	organizer->push(pqueue_index(this, nullptr, ENDED, end_time, listener));
};

state_type roomba_toptouch::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
		case TOUCHENDED:
			if (mytype == OBSTACLESTOPPED) {
				return OBSTACLERUN;
			}
			break;
		case ENDED:
			if ((mytype == RANDROT) || (mytype == REVERSE) || (mytype == TOPTOUCH)) {
				return RUN;
			}
			break;
		}
	}
	return NOSTATE;
}
