#include "roomba_subclass_defs.h"

roomba_obstaclestopped::roomba_obstaclestopped(double at_time, roomba_action * par, state_type type) : roomba_action(at_time, par, type) {
	mytype = STOPPED;
	rw_v = 0;
	lw_v = 0;
	saved->init();
}

void roomba_obstaclestopped::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	end_time = runendtime;
}

state_type roomba_obstaclestopped::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
			case TOUCHENDED:
				if ((info.linked != nullptr) && (info.linked->end_time >= at_time)) {
					return OBSTACLERUN;
				}
				break;
		}
	}
	return NOSTATE;
}

