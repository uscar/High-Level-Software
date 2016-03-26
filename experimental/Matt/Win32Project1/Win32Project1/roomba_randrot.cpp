#include "roomba_subclass_defs.h"

roomba_randrot::roomba_randrot(double at_time, roomba_action * par, state_type type) : roomba_action(at_time, par, type) {
	next_noise = at_time + reverseInterval;
	mytype = RANDROT;
	double randv = rand() % randomMax;
	randv = randv - randomMax / 2;
	randv /= (double)1000.00;
	rw_v = robotSpeed - randv;
	lw_v = robotSpeed + randv;
	saved->init();
}

state_type roomba_randrot::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
		case TOUCHED:
			if ((info.linked != nullptr) && (info.linked->end_time >= at_time)) {
				return REVERSE;
			}
			break;
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

void roomba_randrot::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	end_time = start_time + noiseLength;
	organizer->push(pqueue_index(this, nullptr, ENDED, end_time, listener));
}