#include "roomba_subclass_defs.h"

roomba_stopped::roomba_stopped(double at_time, roomba_action * par, state_type type) : roomba_action(at_time, par, type) {
	mytype = STOPPED;
	rw_v = 0;
	lw_v = 0;
	saved->init();
}

state_type roomba_stopped::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time)) {
		switch (info.type) {
			//wheeeee
		}
	}
	return NOSTATE;
}

void roomba_stopped::add_notifications(std::priority_queue<pqueue_index> * organizer, notifiable * listener) {
	end_time = runendtime;
}
