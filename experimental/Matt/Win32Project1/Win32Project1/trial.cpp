#include "trial.h"
#include "constants.h"
using namespace std;
using namespace simvals;

trial::~trial() {
	delete copter;
	for (int i = 0; i < 14; ++i) {
		delete bots[i];
	}
}

void trial::evaluatenext() {
	if (!organizer.empty()) {
		pqueue_index first = organizer.top();
		organizer.pop();
		//cout << first.caller->ID << endl;
		//if (first.caller->child == nullptr) {
		state_type result = first.to_notify->notify(first.at_time, first); //send notification
		if (result != NOSTATE) {
			first.to_notify->add_notifications(&organizer, nullptr); //senders are also receivers
				
		}
		else {

		}
		/*}
			else while ((currbots[first.caller->ID]->child != nullptr)&&(currbots[first.caller->ID]->child->start_time <= currtime)) { //make it move through known motions
			currbots[first.caller->ID] = currbots[first.caller->ID]->child; //move down the line
			}*/
	}

}

void trial::evaluatetime(double until) { //actual evaluatetime method
										  //cout << "Blah" << endl;
										  //if (!organizer.empty()) {	//int a = 0;
	//double interval = ((double)15 / (double)1000);
	while (!organizer.empty()) {;
		pqueue_index first = organizer.top();
		if (first.at_time > until) {
			return;
		}
		else {
			currtime = first.at_time;
		}
		evaluatenext();
	}
}


void trial::showBestmove() {
	bestMove((*metric), predictUntil, nullptr);
}




trial::trial(evaluation_metric * myMetric = new evaluation_metric()) : collider(bots,organizer) {
	metric = myMetric;
	currtime = 0;
	//startquad = new quad_action(this, sims);
	//addQuad(startquad);
	//currquadtime = currquad->end_time;
	
	for (int i = 0; i < 10; ++i) {
		actions.push_back(actionval(i, BUMP));
		actions.push_back(actionval(i, TAP));
	}
	for (int i = 0; i < 10; i++) {
		double ang = fix_angle((2 * pi / 10)*i/* + c_sim->roomba_offset*/);
		bots[i] = new roomba(RUN,0.0, vec2d(10.0 + cos(ang), 10.0 + sin(ang)), ang, i);
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
	for (int i = 0; i < 4; i++) {
		double ang = fix_angle((2 * pi / 4)*i/* + c_sim->whack_offset*/);
		bots[10 + i] = new roomba(OBSTACLERUN,0.0, vec2d(10.0 + 5.0*cos(ang), 10.0 + 5.0*sin(ang)), fix_angle(ang - pi / 2.0), i + 10);
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
}

//TODO: whenever a state is responsible for spawning another state, make that first state link to it so when it dies the dependent states die too in the destructor, thereby giving nice behavior


vector<actionval> * trial::bestMove(double until, actionval * lastMove) {
	vector<actionval> * act = getActions(currquadtime); //quadtime is needed to sort
	bool ascent;
	double time_before, time_before_init, quad_time_before, quad_time_init = currquadtime;
	quad_action * quad_state_before, *quad_state_init = currquad;

	if (lastMove != nullptr) { //if we just made a move right before this
							   /*if (it.roomba == lastMove->roomba) { // !!!!!!!!!!!!!!!!!!
							   //do something efficient
							   }
							   else {
							   worked = currquad->executeAction(temp,until);
							   }*/
		ascent = currquad->executeAction(actionval(0, ASCEND), until); //fly back to standard level
	}
	else {
		ascent = true;
	}
	roomba_action * priorstates[14]; //it is apparent now why the "not having to redo everything" optimization is not doing anything for us
	time_before = currtime;
	for (int i = 0; i < 14; ++i) {
		priorstates[i] = (roomba_action*)bots[i]->getState();
	}
	for (auto && it : (*act)) {
		quad_time_before = currquadtime;
		quad_state_before = currquad;
		bool worked = ascent;
		if (worked) {
			worked = currquad->flyTo(it.roomba, until); //similar to evaluatetime except keeps going till quad gets to the end?
		}
		//time_before = currtime; //time before aerial robot actually makes any changes to the model
		//for (int i = 0; i < 14; ++i) {
		//	priorstates[i] = currbots[i]; for when we get event based code
		//}
		//time_before = currtime; //time before aerial robot actually makes any changes to the model
		//for (int i = 0; i < 14; ++i) {
		//priorstates[i] = currbots[i];
		//}
		if (worked) { //if we were able to fly to it
			worked = currquad->executeAction(it, until);
		}
		if (worked) { //pick best member
			vector<actionval> * ret = bestMove(a, until, &it);
			it.score = max_element(ret->begin(), ret->end())->score;
			delete ret;
		}
		else {
			//evaluatetime(currtime+5); //temporary. do something like this where you look at the result your move will have
			it.score = 0;
			for (int i = 0; i < 14; ++i) {
				it.score += (*metric)(bots[i], currtime + 5.0); //make a "stupid" prediction of the 
			}
		}
		//if (state_before->child) {
		//delete state_before->child; //eliminate any trace of the action happening
		//currbots[it.roomba] = state_before;
		//}
		if (quad_state_before->child) {
			delete quad_state_before->child; //eliminate any trace of the action happening
			addQuad(quad_state_before);
			currquad->child = nullptr;
		}
		currquadtime = quad_time_before;
		//reverttime(time_before);
		organizer = priority_queue<pqueue_index>();
		currtime = time_before; //taking a different approach to reverts
		for (int i = 0; i < 14; ++i) {
			addBot(priorstates[i], i);
			if (currbots[i]->child != nullptr) {
				delete currbots[i]->child;
				currbots[i]->child = nullptr;
			}
			currbots[i]->state_register();
		}
		collision_handle();
	}
	if (quad_state_init->child) {
		delete quad_state_init->child; //eliminate any trace of the action happening
		addQuad(quad_state_init);
		currquad->child = nullptr;
	}
	currquadtime = quad_time_init;
	return act;
}

void trial::detect_collisions(roomba_action * state) {//project into past capacity pls
	// returns the first collision or -1 if no collisions
	double tmp;
	vec2d p1, p2, diff;
	for (int i = 0; i < 15; i++) {
		if (i != state->ID) {
			tmp = does_collide(currbots[state->ID], currbots[i]);
			if (tmp != -1) {
				p1 = currbots[state->ID]->getPosition(tmp);
				p2 = currbots[i]->getPosition(tmp);
				diff = p2 - p1;
				if (fix_angle(atan2(diff.second, diff.first) - currbots[i]->getAngle(currtime) + pi / 2) <= pi) { //if 2 is in front of 1

				}
				diff = p1 - p2;
				if (fix_angle(atan2(diff.second, diff.first) - currbots[i]->getAngle(currtime) + pi / 2) <= pi) { //if 1 is in front of 2

				}
			}
		}
	}
}