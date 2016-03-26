#pragma once
#include "base_classes.h"
#include "ancillary_classes.h"
#include <queue>

class collision_detector {

public:
	collision_detector(roomba *(&roombas)[14], std::priority_queue<pqueue_index> & schedule) : bots(roombas), organizer(schedule) {}
	void generate(roomba * toUpdate);
	

protected:
	roomba *(&bots)[14]; //a reference to an array of roomba pointers
	std::priority_queue<pqueue_index> & organizer;
	
};
