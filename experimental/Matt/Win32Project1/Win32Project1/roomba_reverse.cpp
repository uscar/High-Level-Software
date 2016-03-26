#include "roomba_subclass_defs.h"

roomba_reverse::roomba_reverse(double at_time, roomba_action * par, state_type type, robot_state * trigger) : roomba_action(at_time, par, type) {
	if (trigger != nullptr) {
		((roomba_action*)trigger)->addDependentState(this);

		next_reversal = at_time + reverseInterval;
	}
	mytype = REVERSE;
	rw_v = -robotSpeed / 2;
	lw_v = robotSpeed / 2;
	saved->init();
}

state_type roomba_reverse::notify(double at_time, pqueue_index info) { //huh. you can interrupt a reverse with a toptouch. I forgot that was even possible
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
		case MAGNET:
			return TOPTOUCH; 
			break;
		case ENDED:
			return RUN;
			break;
		}
	}
	return NOSTATE;
}

void roomba_reverse::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	end_time = start_time + reverseLength;
	organizer->push(pqueue_index(this, nullptr, ENDED, end_time, listener));
}
