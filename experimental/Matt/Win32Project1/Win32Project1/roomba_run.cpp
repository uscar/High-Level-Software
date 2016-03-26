#include "roomba_subclass_defs.h"
#include <queue>

roomba_run::roomba_run(double s_time, vec2d spos, double init_an) : roomba_action(s_time, spos, init_an) {
	mytype = RUN;
	rw_v = robotSpeed;
	lw_v = robotSpeed;
	next_noise = noiseInterval;
	next_reversal = reverseInterval;
	saved->init();
}

roomba_run::roomba_run(double at_time, roomba_action * par, state_type type) : roomba_action(at_time, par, type) {
	mytype = RUN;
	rw_v = robotSpeed;
	lw_v = robotSpeed;
	saved->init();
}

void roomba_run::add_notifications(std::priority_queue<pqueue_index> * organizer,notifiable * listener) {
	if ((next_reversal <= next_noise) || (next_reversal <= start_time)) { //reversal takes precedent
		end_time = std::max(next_reversal, start_time); //put reversal either now or at the next specified reversal time
		organizer->push(pqueue_index(this, nullptr, REVERSAL, end_time, listener));
	}
	else {
		end_time = std::max(next_noise, start_time);
		organizer->push(pqueue_index(this, nullptr, NOISE, end_time, listener));
	}
}


state_type roomba_run::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
		case TOUCHED:
			if ((info.linked != nullptr) && (info.linked->end_time >= at_time)) {
				return REVERSE;
			}
			break;
		case REVERSAL:
			return REVERSE;
			break;
		case NOISE:
			return RANDROT;
			break;
		case MAGNET:
			return TOPTOUCH;
			break;
		}
	}
	return NOSTATE
}

