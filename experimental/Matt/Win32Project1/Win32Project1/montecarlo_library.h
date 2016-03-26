//#define _USE_MATH_DEFINES
#pragma once
#include <utility>
#include <map>
#include <ctime>
#include <string>
#include <cstdlib>
//#include <math.h>
#include <iostream>
#include <queue>       
#include <vector>
#include <list>
//#include "stdafx.h"
#include "vec2d.h"
//#include "Win32Project1.h"
#include <set>
#include "helper.h"
#include <mkl.h>
//#include <WinBase.h>
#include "renderer.h"
#include "constants.h"
#include "base_classes.h"

//NOT COMPLETE, PREDICTION IS NOT FULLY FUNCTIONAL.
//I just need to get the aerial robot behavior implemented and drawn


//TO RE-ADD DRAWING: change evaluatetime implementation to evaluate at intervals of like 15 msec and then print out every time you do that

//need to pass out collision certificates to aerial bot too
using namespace std;
using namespace simvals;

//const enum notification { TOUCHED, REVERSAL, NOISE, MAGNET, ENDED,TOUCHENDED,EXITED};
//const enum state_type { REVERSE, RUN, RANDROT, TOPTOUCH, STOPPED, OBSTACLERUN, OBSTACLESTOPPED, OUTOFPLAY};
//const enum quad_state_type {BUMP, FLY, TAP, ASCEND, HOVER,FLYTHROUGH,ABORT};
//const enum quad_notification {ROOMBASPOTTED};


struct trial;
class roomba_action;
struct pqueue_index;
class quad_action;
struct evaluationFunctor;
class simulation;



//--------------------------------------------------------------------------------STATE CLASS DECLARATIONS--------------------------------------------------------------------------------


/*
void trial::addBot(roomba_action * bot,int i) {
	if (!simulating) {
		//if (startbots[i] != bot) {
			//render->removeObject((robot_state*)currbots[i]);
		//}
		render->addObject((robot_state*)bot);
	}
	currbots[i] = bot;
}

void trial::addQuad(quad_action * bot) {
	if (!simulating) {
		//if (startquad != bot) {
			//render->removeObject((robot_state*)currquad);
		//}
		render->addObject((robot_state*)bot);
	}
	currquad = bot;
}
*/

double roomba_action::getHeight(double at_time) {
	return roombaTapHeight / 2.0;
}

string roomba_action::getName() {
	return "Roomba";
}

state_type roomba_obstaclestopped::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time) && (c_test->currbots[ID] == this)) {
		switch (info.type) {
			case TOUCHENDED:
				if (((info.linked != nullptr) && (info.linked->end_time >= at_time)) || info.quadhit) {
					spawn_state(at_time, OBSTACLERUN);
				}
				break;
		}
	}
}

void roomba_outofplay::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time) && (c_test->currbots[ID] == this)) {
		switch (info.type) {
			//wheeeee
		}
	}
}

double quad_action::getAngle(double at_time) {
	return 0;
}

quad_action::quad_action(double end, vec2d toward, quad_action * prior, roomba_action * track, quad_state_type st) { //this is NOT consistent with the spawn_state shenanigans I do with roombas. Will rectify.
	startheight = prior->getHeight(prior->end_time);
	switch (st) {
		case ASCEND:
			dh = quadvSpeedEst;
			break;
		case TAP:
			dh = quadvSpeedEstInteracting;
			break;
		case BUMP:
			dh = quadvSpeedEstInteracting;
			break;
		default:
			dh = min(abs((cruiseHeight - startheight) / (end - prior->end_time)), quadvSpeedEst)*(((cruiseHeight - startheight) > 0) ? 1 : -1); //makes it tend toward the cruise height
			break;
	}
	if (prior->child != nullptr) {
		if (prior->child != this) {
			delete prior->child;
		}
	}
	ID = 18;
	goal_action = prior->goal_action;
	start_time = prior->end_time; //pretty self explanatory
	end_time = end;
	prior->child = this;  //we assume prior exists. if it doesn't, we have a problem.
	parent = prior;
	startpos = prior->getPosition(prior->end_time);
	//startheight = prior->getheight(prior->end_time);
	double magnitude = toward.getMagnitude();
	if (magnitude > 0.0001) {
		dx = toward.first / magnitude;
		dy = toward.second / magnitude;
	}
	else {
		dx = 0;
		dy = 0;
	}
	child = nullptr;
	c_test = prior->c_test;
	tracking = track;
	mytype = st;
	c_test->currquadtime = prior->end_time;
	//c_test->addQuad(this);
}

quad_action::quad_action(trial * tr, simulation * sim) { //initial constructor
	start_time = 0; //pretty self explanatory
	end_time = cruiseHeight / quadvSpeedEst;
	dx = 0;
	ID = 18;
	dy = 0;
	dh = quadvSpeedEst; //going up
	child = nullptr;
	parent = nullptr;
	c_test = tr;
	tracking = nullptr;
	mytype = ASCEND;
	//c_test->addQuad(this);
	startpos = vec2d((double)10.0, (double)10.0);
}

roomba_action * fetchInterceptState(roomba_action * targ,quad_action * copter,double cap) { //can quad reach this roomba state within time bounds? If not, is there too much time or too little.
	double dist_to, coverable;
	do {
		dist_to = targ->startpos.dist_to(copter->startpos);
		coverable = quadSpeedEst * (cap - copter->start_time);
		if (dist_to >= coverable) {
			return (targ->child); //return state we're going to run into. could be nullptr
		}
		cap = targ->start_time;
		targ = targ->parent;
	} while (dist_to < coverable);
	return nullptr;
}

bool quad_action::executeAction(actionval & a, double until) {
	double endt;
	switch (a.action) {
		case TAP:
			endt = c_test->currquadtime + abs((roombaTapHeight - getHeight(c_test->currquadtime)) / quadvSpeedEstInteracting);
			if (endt >= until) {
				return false;
			}
			end_time = c_test->currquadtime;
			child = new quad_action(endt, vec2d(0, 0), this, c_test->currbots[a.roomba],TAP);
			c_test->currquadtime = endt;
			if (c_test->currtime < endt) {
				c_test->evaluatetime(endt); //we wanna know what state we'll be interacting with
			}
			c_test->currbots[a.roomba]->addquadevent(endt, MAGNET);
			break;
		case BUMP:
			endt = c_test->currquadtime + abs((roombaBumpHeight - getHeight(c_test->currquadtime)) / quadvSpeedEstInteracting);
			if (endt >= until) {
				return false;
			}
			end_time = c_test->currquadtime;
			child = new quad_action(endt, vec2d(0, 0), this, c_test->currbots[a.roomba], BUMP);
			c_test->currquadtime = endt;
			if (c_test->currtime < endt) {
				c_test->evaluatetime(endt); //we wanna know what state we'll be interacting with
			}
			c_test->currbots[a.roomba]->addquadevent(endt, TOUCHED);
			break;
		case ASCEND:
			endt = c_test->currquadtime + abs((cruiseHeight - getHeight(c_test->currquadtime)) / quadvSpeedEst);
			if (endt >= until) {
				return false;
			}
			end_time = c_test->currquadtime;
			child = new quad_action(endt, vec2d(0,0), this, nullptr, ASCEND);
			c_test->currquadtime = endt;
			break;
		default:
			break;
	}
	return true;
}

//have to revert to get proper state??

bool quad_action::flyTo(int roomba, double until) { //cut time before calculation done
	double beginTime = c_test->currquadtime;
	double workingTime = beginTime;
	roomba_action * istate = c_test->currbots[roomba];
	double cap = min(c_test->currtime, until);//upper time bound
	roomba_action * cstate = fetchInterceptState(istate, this, cap);
	vec2d qcpos = getPosition(beginTime);
	while (cstate == nullptr) {
		if (c_test->currtime >= until) { //if we've exceeded the max allotted time
			return false; //cannot be flown to
		}
		else {
			c_test->evaluatetime(min(istate->end_time+0.1, until)); 
			cap = min(c_test->currtime, until);
		}
		cstate = fetchInterceptState(istate, this, cap);
	}
	double errorterm, dist_to, coverable;
	if (abs(cstate->rw_v - cstate->lw_v) < 1.0e-6) {
		vec2d velocityvec = vec2d(robotSpeed * cos(cstate->initial_angle), robotSpeed * sin(cstate->initial_angle));
		workingTime = c_test->currquadtime + quadParamLineIntersect(cstate->startpos + ((c_test->currquadtime-cstate->start_time)*velocityvec)-qcpos, velocityvec,quadSpeedEst); //cray math mon
		dist_to = cstate->getPosition(workingTime).dist_to(qcpos);
		coverable = quadSpeedEst * (workingTime - c_test->currquadtime);
		errorterm = (abs(dist_to - coverable));
	}
	else if (abs(cstate->rw_v + cstate->lw_v) < 1.0e-6) { //spinning in place
		workingTime = c_test->currquadtime + cstate->startpos.dist_to(qcpos) / quadSpeedEst;
	}
	else{
		vec2d epos = cstate->getPosition(cstate->end_time);
		vec2d spos = cstate->startpos;
		vec2d velocityvec = (epos, spos); //get displacement
		velocityvec = (robotSpeed/(velocityvec.getMagnitude())) * velocityvec; //multiply by ratio of velocity to roomba speed
		workingTime = c_test->currquadtime + quadParamLineIntersect(((spos+((beginTime-cstate->start_time)*velocityvec))-qcpos),velocityvec, quadSpeedEst); //take approximation from end to end and extend it out to the quadcurrtime, so as to not lose accuracy
		dist_to = cstate->getPosition(workingTime).dist_to(qcpos);
		coverable = quadSpeedEst * (workingTime - beginTime);
		errorterm = (abs(dist_to - coverable));
		int i = 0; 
		//newton's method calculation below, use when not accurate enough
		while (((abs(dist_to - coverable)) > newton_accuracy) && (i < newton_iterations)) { //todo: make readable with simple methods created earlier in code
			errorterm = (abs(dist_to - coverable));
			dist_to = cstate->getPosition(workingTime).dist_to(qcpos);
			coverable = quadSpeedEst * (workingTime - beginTime);
			double ang = (cstate->rw_v - cstate->lw_v)*(workingTime - beginTime) / wheelbase_width + cstate->initial_angle;
			double a = wheelbase_width*(cstate->rw_v + cstate->lw_v) / (2*(cstate->rw_v - cstate->lw_v));
			vec2d p_t = cstate->startpos + a*vec2d(sin(ang) - sin(cstate->initial_angle), cos(ang) - cos(cstate->initial_angle));
			vec2d prime = ((cstate->rw_v + cstate->lw_v) / 2) * vec2d(ang);
			double func = (qcpos-p_t).square_components() - square(quadSpeedEst*(workingTime-beginTime));
			double prime_term = -2 * ((qcpos - p_t).dot(prime)+ quadSpeedEst*quadSpeedEst*(workingTime-beginTime));
			workingTime = workingTime - func / prime_term;
			++i;
		}
		errorterm = (abs(dist_to - coverable));
	}
	if (workingTime >= until) {
		return false;
	}
	vec2d endpos = cstate->getPosition(workingTime);
	end_time = beginTime; //spooky
	child = new quad_action(workingTime,endpos-qcpos,this,nullptr,FLY);
	c_test->currquadtime = workingTime; //just skip to end
	//errorterm = (abs(dist(vec_subtract(child->getPosition(workingTime), cstate->getPosition(workingTime)),vec2d(0.0,0.0)))); //true error term
	return true;
}

double quad_action::simpleTimeEst(actionval & a, double time) {
	return c_test->currbots[a.roomba]->getPosition(time).dist_to(getPosition(time));
}

vec2d quad_action::getPosition(double at_time) {
	if (start_time > at_time) { //make it work universally
		return parent->getPosition(at_time);
	}
	if ((child != nullptr) && (child->start_time < at_time)) {
		return child->getPosition(at_time);
	}
	//simplistic code for rn
	if (tracking != nullptr) { //this assumes we can perfectly follow a roomba, which isn't too much of a stretch
		vec2d translate = (tracking->getPosition(at_time) - tracking->getPosition(start_time));
		return startpos,translate; //add displacement of roomba over interval to position at start
	}
	else {
		vec2d translate = (quadSpeedEst*(at_time - start_time) * vec2d(dx, dy));
		return startpos+translate; //add displacement due to velocity to position at start
	}
}

double quad_action::getHeight(double at_time) {
	return startheight + dh*(at_time - start_time);
}


class simulation { //super master class thing monster creature
public:
	list<trial> tests;
	int testn = 0;
	double roomba_offset;
	double whack_offset;
	//vector<roomba*> robots;
	//simulation(double offset);
	actionval bestMove(evaluationFunctor & a,double until);

	simulation(int trials);
	//vector<actionval> actions;
	//actionval simulation::branch();
};


vector<actionval> * trial::getActions(double time) {
	return actions;
}

//recursive call. use distance to estimate actual time required. sort actionvals by this


vector<actionval> * trial::bestMove(evaluationFunctor & a,double until,actionval * lastMove) {
	vector<actionval> * act = getActions(currquadtime); //quadtime is needed to sort
	bool ascent;
	double time_before,time_before_init,quad_time_before, quad_time_init = currquadtime;
	quad_action * quad_state_before, * quad_state_init = currquad;

	if (lastMove != nullptr) { //if we just made a move right before this
		/*if (it.roomba == lastMove->roomba) {  !!!!!!!!!!!!!!!!!!
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
		priorstates[i] = currbots[i];
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
			vector<actionval> * ret = bestMove(a, until,&it);
			it.score = max_element(ret->begin(),ret->end())->score;
			delete ret;
		}
		else {
			//evaluatetime(currtime+5); //temporary. do something like this where you look at the result your move will have
			it.score = a(this, currbots[it.roomba]->end_time); //make a "stupid" prediction of the future
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
			addBot(priorstates[i],i);
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

actionval simulation::bestMove(evaluationFunctor & a, double until) {
	auto getMedian = [](actionval * arrin, int arrsize) {
		return (arrsize % 2) ? arrin[arrsize / 2].score : ((arrin[arrsize / 2].score += arrin[arrsize / 2 - 1].score)/2);
	};
	actionval ** values = new actionval *[actions.size()]; //first index is for each action, second index is for each trial
	for (unsigned int i = 0; i < tests.size(); ++i) {
		values[i] = new actionval[tests.size()];
	}
	{
		int i = 0;
		for (auto && it : tests) {
			vector<actionval> * retv = it.bestMove(a, until,nullptr); //get best moves for the trial
			for (unsigned int j = 0; j < actions.size(); ++j) {
				values[j][i] = (*retv)[j]; //enter it in a useful way
			}
			++i;
		}
	}
	//double * results = new double[tests.front.actions.size()];
	int bestaction = 0;
	sort(values[0], values[0] + tests.size());
	int bestvalue = getMedian(values[0], tests.size());
	for (unsigned int i = 1; i < actions.size(); ++i) {
		sort(values[i], values[i] + tests.size());
		int temp = getMedian(values[i], tests.size());
		if (temp > bestvalue) {
			bestvalue = temp;
			bestaction = i;
		}
	}
	values[bestaction][0].score = bestvalue;
	return values[bestaction][0];
}

simulation::simulation(int trials) {
	for (int i = 0; i < 10; ++i) {
		actions.push_back(actionval(i, BUMP));
		actions.push_back(actionval(i, TAP));
	}
}

void trial::reverttime(double until) {
	while (!organizer.empty()) {
		organizer.pop();
	}
	if (currtime > until) { //going backwards
		for (int i = 0; i < 14; ++i) {
			while (currbots[i]->start_time > until) {
				addBot(currbots[i]->parent,i);
				delete currbots[i]->child;
			}
			currbots[i]->state_register(); //renotify
		}
		currtime = until;
		evaluatetime(currtime); //why not
		collision_handle();
	}
}


void trial::evaluatetime(double until) { //use this method to show things on the sketchy renderer.
	if (!bestmove_test) {
		return evaluatetime2(until);
	}
	double interval = (double)15.0 / (double)1000.0;
	while (currtime < until) {
		evaluatetime2(currtime + interval);
		render->draw(currtime);
	}
}

void trial::show(double until) { //use this method to show things on the sketchy renderer.
	double interval = (double)15.0 / (double)1000.0;
	while (currtime < until) {
		evaluatetime(currtime+interval);
		//setLatestTime(currtime)i;
		//cout << "drawcall!" << endl;
		render->draw(currtime);
		/*if ((currtime + interval) >= currquad->end_time) { //this block of code is very temporary and incredibly jury-rigged.
			if (currquad->mytype == ASCEND) {
				evaluationFunctor a;
				roomba_action * priorstates[14]; //it is apparent now why the not having to redo everything optimization is not doing anything for us
				double priortime = currtime;
				for (int i = 0; i < 14; ++i) {
					priorstates[i] = currbots[i];
				}
				//vector<actionval> * ret = bestMove(a, currtime+predictUntil, nullptr);
				for (int i = 0; i < 14; ++i) {
					currbots[i] = priorstates[i];
					if (currbots[i]->child) {
						delete currbots[i]->child;
					}
				}
				currquad->goal_action = *(max_element(ret->begin(), ret->end()));
				delete ret;
				currquad->flyTo(currquad->goal_action.roomba, runendtime);
			}
			else if ((currquad->mytype == TAP) || (currquad->mytype == BUMP)) {
				currquad->executeAction(actionval(0, ASCEND), runendtime);
			}
			else if (currquad->mytype == FLY){
				currquad->executeAction(currquad->goal_action, runendtime);
			}
		}*/
	}
}

/*

IN FINAL VERSION OF COLLISION HANDLER:
ONLY HAVE IT OPERATE ON ONE ROOMBA ACTION!
PUT EVENT NOTIFICATIONS FOR IT AND ANYTHING IT MIGHT TOUCH
ALSO
HAVE IT CHECK IF ANYTHING IS ALREADY TOUCHING IT!!!!!!!!!!!!!!!!!!!! (and pass out appropriate notifications)

*/

/*
//semi-defunct code
void trial::collision_handle() {
	for (int i = 0; i < 13; i++) {
		for (int j = i + 1; j < 14; j++) {
			vec2d a = currbots[i]->getPosition(currtime);
			vec2d b = currbots[j]->getPosition(currtime);
			if (square(roombaDiameter)>=(a-b).square_components()) {
				if (fix_angle(atan2(b.second - a.second, b.first - a.first) - currbots[i]->getAngle(currtime) + pi / 2) <= pi) { //if other is in front
					if ((!clist[i][j]) || (currbots[i]->mytype == RUN) || (currbots[i]->mytype == RANDROT) || (currbots[i]->mytype == TOPTOUCH) || (currbots[i]->mytype == OBSTACLERUN)) {
						clist[i][j] = true;
						organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHED, currtime,false));
					}
				}
				else {
					//put some debug stuff here!
					if (clist[i][j]) {
						organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHENDED, currtime,false));
						clist[i][j] = false;
					}
				}
				if (fix_angle(atan2(a.second - b.second, a.first - b.first) - currbots[j]->getAngle(currtime) + pi / 2) <= pi) { //if other is in front
					if ((!clist[j][i]) || (currbots[j]->mytype == RUN) || (currbots[j]->mytype == RANDROT) || (currbots[j]->mytype == TOPTOUCH) || (currbots[i]->mytype == OBSTACLERUN)) {
						clist[j][i] = true;
						organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHED, currtime,false));
					}
				}
				else {
					if (clist[j][i]) {
						organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHENDED, currtime,false));
						clist[j][i] = false;
					}				
				}
			}
			else {
				if (clist[i][j]) {
					organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHENDED, currtime,false));
					clist[i][j] = false;
				}
				if (clist[j][i]) {
					organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHENDED, currtime,false));
					clist[j][i] = false;
				}
			}
		}
	}
}
*/
