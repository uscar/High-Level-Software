//#define _USE_MATH_DEFINES
#include <utility>
#include <map>
#include <ctime>
#include <string>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <queue>       
#include <vector>
#include <list>
#include "stdafx.h"
#include "Win32Project1.h"
#include <set>
#include <WinBase.h>


//NOT COMPLETE, PREDICTION IS NOT FULLY FUNCTIONAL.
//I just need to get the aerial robot behavior implemented and drawn


//TO RE-ADD DRAWING: change evaluatetime implementation to evaluate at intervals of like 15 msec and then print out every time you do that

//need to pass out collision certificates to aerial bot too

using namespace std;

class simulation;
class quad_action;
class roomba_action;
const int numroombas = 10;
long double pi = atan2(0, -1);

//naive handling of trials, just for testing
int num_sims = 10;//we'll find a better way to manage the simulations later
double sim_duration = 20; //in seconds

// vals from arduino
double robotSpeed = (double)330/(double)1000; // m/s
int randomMax = 64;
// Time between trajectory noise injections
double noiseInterval = (double)5000/(double)1000;
double lastNoise = 0;
// Time between auto-reverse
double reverseInterval = (double)20000/(double)1000;
double lastReverse = 0;
// Time needed to affect trajectory
double noiseLength = (double)850/(double)1000;
double beginNoise = 0;
// Time needed to reverse trajectory
double reverseLength = (double)2456/(double)1000; // .33/2 m/s * pi * wheelbase / 2
double beginReverse = 0;
// Time needed to spin 45 degrees
double topTouchTime = reverseLength / 4;
double beginTopTouch = 0;

HDC hd;
RECT * re; //sketchy testing nonsense

double quadSpeedEst = 3.0; //m/s
double quadvSpeedEst = 1;
double quadvSpeedEstInteracting = -quadSpeedEst / 3;

double roombaTapHeight = 0.05;
double roombaBumpHeight = 0.0;
double cruiseHeight = 1.0;

double wheelbase_width = 0.25798379655423864; //worked backwards to this

double fix_angle(double in_afngle); //all in milliseconds
double roombaDiameter = 0.34;

double runendtime = 600; //default state end time

double newton_accuracy = 0.1; //within 10 cm for now
int newton_iterations = 7;

double topTouchTimereq = 100;
double bumperTimereq = 100;

double FOV; //we need this value to find the vield of view as a function of height

int branching = 5;
double predictUntil = 8.0; //10s predictive capacity


enum notification { TOUCHED, REVERSAL, NOISE, MAGNET, ENDED,TOUCHENDED,EXITED};
enum state_type { REVERSE, RUN, RANDROT, TOPTOUCH, STOPPED,OBSTACLERUN,OBSTACLESTOPPED,TOTALSTOP,OUTOFPLAY};
enum quad_state_type {BUMP, FLY, TAP, ASCEND, HOVER,FLYTHROUGH,ABORT};
enum quad_notification { ROOMBASPOTTED};


struct trial;
class roomba_action;
struct pqueue_index;
struct evaluationFunctor;
class simulation;

double fix_angle(double in_angle) {
	while (in_angle >= 2 * pi) {
		in_angle -= 2 * pi;
	}
	while (in_angle < 0) {
		in_angle += 2 * pi;
	}
	return in_angle;
}

double square(double a) { //fairly certain pow has a special case for this but if not, doesn't hurt to write something like this
	return a*a;
}

struct vec2d { //phase out pair<double,double>s in favor of this
	double first;
	double second;
	pair<double, double> getPaired();
	vec2d(double, double);
	vec2d(pair<double, double>);
	void swap();
	double getMagnitude();
};

void vec2d::swap() {
	double t = first;
	first = second;
	second = t;
}

double vec2d::getMagnitude() {
	return sqrt(first*first + second*second);
}

pair<double,double> vec2d::getPaired() {
	return make_pair(first, second);
}

vec2d::vec2d(double a, double b) {
	first = a;
	second = b;
}

vec2d::vec2d(pair<double,double> a) {
	first = a.first;
	second = a.second;
}

vec2d operator* (double a, const vec2d& b) {
	return vec2d(a*b.first,a*b.second);
}

vec2d operator* (const vec2d& a, double b) {
	return vec2d(a.first*b, a.second*b);
}

vec2d operator+(const vec2d & a,const vec2d & b) {
	return vec2d(a.first + b.first, a.second + b.second);
}
vec2d operator-(const vec2d & a,const vec2d & b) {
	return vec2d(a.first - b.first, a.second - b.second);
}

double dist(pair<double, double> a, pair<double, double> b) {
	return sqrt(square(a.first - b.first) + square(a.second - b.second));
}

pair<double, double> vec_subtract(pair<double, double> a, pair<double, double> b) {
	return make_pair(a.first - b.first, a.second - b.second);
}

pair<double, double> vec_scalarmult(double a, pair<double, double> b) {
	return make_pair(a*b.first, a*b.second);
}

pair<double, double> vec_add(pair<double, double> a, pair<double, double> b) {
	return make_pair(a.first + b.first, a.second + b.second);
}

struct pqueue_index { //a notification
	roomba_action * caller; //who we'll notify
	roomba_action * linked; //the possible roomba that 
	notification type;
	double at_time;
	//bool quad_notify;
	pqueue_index(roomba_action * call, roomba_action * link, notification ty, double t) : caller(call), linked(link), type(ty), at_time(t) {
		//quad_notify = false;
	}
	/*bool operator<(const pqueue_index & a) {
		return at_time < a.at_time;
	}*/
};

bool operator<(const pqueue_index&a, const pqueue_index & b) {
	return a.at_time > b.at_time;
}

struct actionval {
	int roomba = 0;
	quad_state_type action = HOVER;
	int score = 0;
	//int special = 0;
	bool operator<(actionval & other) {
		return score < other.score;
	}
	actionval(int, quad_state_type);
	actionval();
};
actionval::actionval(int a, quad_state_type b) : roomba(a),action(b)  {}
actionval::actionval() {
	action = HOVER;
	score = 0;
	roomba = 0;
}

double quadParamLineIntersect(pair<double, double> roombaDisplacement, pair<double, double> roombaVelocity, double quadVelocity) {
	double a = square(roombaVelocity.first) + square(roombaVelocity.second) - square(quadVelocity);
	double b = 2 * (roombaVelocity.first*roombaDisplacement.first + roombaVelocity.second*roombaDisplacement.second);
	double c = square(roombaDisplacement.first) + square(roombaDisplacement.second);
	return (-b - sqrt(square(b) - (4 * a*c))) / (2 * a); //I feel like this math should not come as a shock to anyone
}

struct trial {
	void evaluatetime2(double until);
	void show(double until, HDC hdc, RECT * prc);
	void showBestmove(HDC hdc, RECT*prc);
	priority_queue<pqueue_index> organizer;
	trial(simulation * sims);
	simulation * c_sim;
	roomba_action * startbots[14];
	roomba_action * currbots[14];
	bool clist[14][14];
	void collision_handle();
	double currtime = 0;
	double currquadtime = 0;
	void evaluatetime(double until);
	void reverttime(double until);
	vector<actionval> * bestMove(evaluationFunctor & a, double until, actionval * lastMove);
	vector<actionval> * getActions(double time);
	quad_action * startquad; //this trial's quad actions
	quad_action * currquad; //this trial's quad action
};

class roomba_action {
public:
	void notify(double at_time, pqueue_index type);
	roomba_action * parent = nullptr;
	roomba_action * child = nullptr;


	state_type mytype;

	double next_noise;
	double next_reversal;
	bool is_obstacle = false;

	double start_time;
	double end_time; 
	double end_signal;
	trial * c_test;
	simulation * c_sim;

	double rw_v = 0;
	double lw_v = 0;

	pair<double, double> startpos;
	double initial_angle;
	int ID;

	void addevent(double at_time, notification type);
	roomba_action(double at_time, roomba_action * par, state_type type);
	roomba_action(double s_time, pair<double, double> spos, double init_an, trial * curr_test, int i,bool killbot);
	~roomba_action();

	void spawn_state(double at_time, state_type st);
	pair<double, double> getposition(double at_time);
	double getrotation(double at_time);
	void initNotify();
};

class quad_action {
public:
	double dx;
	double dy;
	double dh;
	double start_time;
	double startheight; //because why not
	double end_time;
	double orientation; //unused currently
	pair<double, double> startpos;
	quad_action * parent;
	quad_action * child;
	quad_action::quad_action(double end, pair<double, double> velocity, quad_action * prior, roomba_action * track, quad_state_type t);
	quad_action::quad_action(trial * tr, simulation * sim);
	trial * c_test;
	simulation * c_sim;
	roomba_action * tracking;
	//void append();
	quad_state_type mytype;
	bool executeAction(actionval & a,double until);
	bool flyTo(int roomba,double until);
	double simpleTimeEst(actionval & a,double time);
	pair<double, double> getposition(double at_time);
	double getheight(double at_time);
	void spawn_state(double at_time, quad_state_type st);
	~quad_action();
};

struct evaluationFunctor {
	virtual double operator()(trial * a, double time) {
		double total = 0;
		for (int i = 0; i < 10; ++i) {
			total += a->currbots[i]->getposition(a->currtime).first;
		}
		return total;
	}
};

quad_action::quad_action(double end, pair<double, double> toward, quad_action * prior, roomba_action * track, quad_state_type st) { //this is NOT consistent with the spawn_state shenanigans I do with roombas. Will rectify.
	startheight = prior->getheight(prior->end_time);
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
	start_time = prior->end_time; //pretty self explanatory
	end_time = end;
	prior->child = this;  //we assume prior exists. if it doesn't, we have a problem.
	parent = prior;
	startpos = prior->getposition(prior->end_time);
	//startheight = prior->getheight(prior->end_time);
	double magnitude = sqrt(toward.first*toward.first + toward.second*toward.second);
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
	c_sim = prior->c_sim;
	tracking = track;
	mytype = st;
	c_test->currquadtime = prior->end_time;
	c_test->currquad = this;
}

quad_action::quad_action(trial * tr, simulation * sim) { //initial constructor
	start_time = 0; //pretty self explanatory
	end_time = cruiseHeight / quadvSpeedEst;
	dx = 0;
	dy = 0;
	dh = quadvSpeedEst; //going up
	child = nullptr;
	parent = nullptr;
	c_test = tr;
	c_sim = sim;
	tracking = nullptr;
	mytype = ASCEND;
	c_test->currquad = this;
	startpos = make_pair((double)10.0, (double)10.0);
}

roomba_action * fetchInterceptState(roomba_action * targ,quad_action * copter,double cap) { //can quad reach this roomba state within time bounds? If not, is there too much time or too little.
	double dist_to, coverable;
	do {
		dist_to = dist(targ->startpos, copter->startpos);
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
			endt = c_test->currquadtime + abs((roombaTapHeight - getheight(c_test->currquadtime)) / quadvSpeedEstInteracting);
			if (endt > until) {
				return false;
			}
			end_time = c_test->currquadtime;
			child = new quad_action(endt, make_pair(0, 0), this, c_test->currbots[a.roomba],TAP);
			c_test->currquadtime = endt;
			if (c_test->currtime < endt) {
				c_test->evaluatetime(endt); //we wanna know what state we'll be interacting with
			}
			c_test->currbots[a.roomba]->addevent(endt, MAGNET);
			break;
		case BUMP:
			endt = c_test->currquadtime + abs((roombaBumpHeight - getheight(c_test->currquadtime)) / quadvSpeedEstInteracting);
			if (endt > until) {
				return false;
			}
			end_time = c_test->currquadtime;
			child = new quad_action(endt, make_pair(0, 0), this, c_test->currbots[a.roomba], BUMP);
			c_test->currquadtime = endt;
			if (c_test->currtime < endt) {
				c_test->evaluatetime(endt); //we wanna know what state we'll be interacting with
			}
			c_test->currbots[a.roomba]->addevent(endt, TOUCHED);
			break;
		case ASCEND:
			endt = c_test->currquadtime + abs((cruiseHeight - getheight(c_test->currquadtime)) / quadvSpeedEst);
			if (endt > until) {
				return false;
			}
			end_time = c_test->currquadtime;
			child = new quad_action(endt, make_pair(0,0), this, nullptr, ASCEND);
			c_test->currquadtime = endt;
			break;
		default:
			break;
	}
	return true;
}

bool quad_action::flyTo(int roomba, double until) { //cut time before calculation done
	double beginTime = c_test->currquadtime;
	double workingTime = beginTime;
	roomba_action * istate = c_test->currbots[roomba];
	double cap = min(c_test->currtime, until);//upper time bound
	roomba_action * cstate = fetchInterceptState(istate, this, cap);
	while (cstate == nullptr) {
		if (c_test->currtime >= until) { //if we've exceeded the max allotted time
			return false; //cannot be flown to
		}
		else {
			c_test->evaluatetime(min(istate->end_time+0.1, until)); 
			cap = min(c_test->currtime, until);
		}
		if (c_test->currtime < until) {
			cstate = fetchInterceptState(istate, this, cap);
		}
	}
	double errorterm, dist_to, coverable;
	if (abs(cstate->rw_v - cstate->lw_v) < 1.0e-6) {
		pair<double, double> velocityvec = make_pair(robotSpeed * cos(cstate->initial_angle), robotSpeed * sin(cstate->initial_angle));
		workingTime = c_test->currquadtime + quadParamLineIntersect(vec_subtract(vec_add(cstate->startpos, vec_scalarmult(c_test->currquadtime-cstate->start_time,velocityvec)),startpos), velocityvec,quadSpeedEst); //cray math mon
		dist_to = dist(cstate->getposition(workingTime), startpos);
		coverable = quadSpeedEst * (workingTime - c_test->currquadtime);
		errorterm = (abs(dist_to - coverable));
	}
	else if (abs(cstate->rw_v + cstate->lw_v) < 1.0e-6) { //spinning in place
		workingTime = c_test->currquadtime + dist(cstate->startpos, startpos) / quadSpeedEst;
	}
	else{
		//workingTime = (cstate->end_time - cstate->start_time) / 2;
		pair<double, double> epos = cstate->getposition(cstate->end_time);
		pair<double, double> spos = cstate->startpos;
		pair<double, double> velocityvec = vec_subtract(epos, spos); //get displacement
		velocityvec = vec_scalarmult(robotSpeed/sqrt(square(velocityvec.first)+square(velocityvec.second)),velocityvec); //convert to velocity
		workingTime = c_test->currquadtime + quadParamLineIntersect(vec_subtract(vec_add(spos,vec_scalarmult(c_test->currquadtime-cstate->start_time,velocityvec)),startpos),velocityvec, quadSpeedEst); //take approximation from end to end and extend it out to the quadcurrtime, so as to not lose accuracy
		dist_to = dist(cstate->getposition(workingTime), startpos);
		coverable = quadSpeedEst * (workingTime - beginTime);
		errorterm = (abs(dist_to - coverable));
		int i = 0;
		while (((abs(dist_to - coverable)) > newton_accuracy) && (i < newton_iterations)) { //todo: make readable with simple methods created earlier in code
			errorterm = (abs(dist_to - coverable));
			dist_to = dist(cstate->getposition(workingTime), startpos);
			coverable = quadSpeedEst * (workingTime - beginTime);
			double ang = (cstate->rw_v - cstate->lw_v)*(workingTime - beginTime) / wheelbase_width + cstate->initial_angle;
			double a = wheelbase_width*(cstate->rw_v + cstate->lw_v) / (2*(cstate->rw_v - cstate->lw_v));
			double x_t = cstate->startpos.first + a*(sin(ang) - sin(cstate->initial_angle));
			double y_t = cstate->startpos.second - a*(cos(ang) - cos(cstate->initial_angle));
			double x_prime = ((cstate->rw_v + cstate->lw_v) / 2)*cos(ang);
			double y_prime = ((cstate->rw_v + cstate->lw_v) / 2)*sin(ang);
			double func = (startpos.first - x_t)*(startpos.first - x_t) + (startpos.second - y_t) * (startpos.second - y_t) - (quadSpeedEst*(workingTime-beginTime))*(quadSpeedEst*(workingTime-beginTime));
			double prime = -2 * (startpos.first - x_t)*x_prime - 2 * (startpos.second - y_t)*y_prime - 2 * quadSpeedEst*quadSpeedEst*(workingTime-beginTime);
			workingTime = workingTime - func / prime;
			++i;
		}
		errorterm = (abs(dist_to - coverable));
		if (workingTime > until) {
			return false;
		}
	}
	pair<double, double> endpos = cstate->getposition(workingTime);
	end_time = beginTime; //spooky
	child = new quad_action(workingTime,vec_subtract(endpos,startpos),this,nullptr,FLY);
	c_test->currquadtime = workingTime; //just skip to end
	return true;
}

double quad_action::simpleTimeEst(actionval & a, double time) {
	return dist(c_test->currbots[a.roomba]->getposition(time), getposition(time));
}

pair<double, double> quad_action::getposition(double at_time) {
	if (start_time > at_time) { //make it work universally
		return parent->getposition(at_time);;
	}
	if ((child != nullptr) && (child->start_time < at_time)) {
		return child->getposition(at_time);
	}
	//simplistic code for rn
	if (tracking != nullptr) { //this assumes we can perfectly follow a roomba, which isn't too much of a stretch
		pair<double, double> translate = vec_subtract(tracking->getposition(at_time), tracking->getposition(start_time));
		return vec_add(startpos,translate); //add displacement of roomba over interval to position at start
	}
	else {
		pair<double,double> translate = vec_scalarmult(quadSpeedEst*(at_time - start_time), make_pair(dx, dy));
		return vec_add(startpos,translate); //add displacement due to velocity to position at start
	}
}

double quad_action::getheight(double at_time) {
	return startheight + dh*(at_time - start_time);
}


class simulation { //super master class thing monster creature
public:
	list<trial> tests;
	int testn = 0;
	double roomba_offset;
	double whack_offset;
	//vector<roomba*> robots;
	simulation(double offset);
	actionval bestMove(evaluationFunctor & a,double until);
	simulation::simulation(int trials);
	vector<actionval> actions;
	//actionval simulation::branch();
};


vector<actionval> * trial::getActions(double time) {
	vector<actionval> * a = new vector<actionval>(c_sim->actions);
	auto comp = [&](actionval & a, actionval & b) {
		return (currquad->simpleTimeEst(a,time) < currquad->simpleTimeEst(b,time));
	};
	sort(a->begin(), a->end(),comp);
	return a;
}

//recursive call. use distance to estimate actual time required. sort actionvals by this


vector<actionval> * trial::bestMove(evaluationFunctor & a,double until,actionval * lastMove) {
	vector<actionval> * act = getActions(currquadtime); //quadtime is needed to sort
	bool ascent;
	quad_action * quad_state_before = currquad;
	if (lastMove != nullptr) {
		/*if (it.roomba == lastMove->roomba) {            !!!!!!!!!!!!!!!!!!
		//do something efficient
		}
		else {
		worked = currquad->executeAction(temp,until);
		}*/
		ascent = currquad->executeAction(actionval(0, ASCEND), until);
	}
	else {
		ascent = true;
	}
	for (auto && it : (*act)) {
		double quad_time_before = currquadtime;
		bool worked = ascent;
		if (worked) {
			worked = currquad->flyTo(it.roomba, until); //similair to evaluatetime except keeps going till quad gets to the end
		}
		double time_before = currtime; //time before aerial robot actually makes any changes to the model
		roomba_action * state_before = currbots[it.roomba]; //save current state of roomba we're about to mess with
		if (worked) { //if we were able to fly to it
			worked = currquad->executeAction(it, until);

		}
		if (worked) { //pick best member
			vector<actionval> * ret = bestMove(a, until,&it);
			it.score = max_element(ret->begin(),ret->end())->score;
			delete ret;
		}
		else {
			it.score = a(this, until);
		}
		if (state_before->child) {
			delete state_before->child; //eliminate any trace of the action happening
			currbots[it.roomba] = state_before;
		}
		if (quad_state_before->child) {
			delete quad_state_before->child; //eliminate any trace of the action happening
			currquad = quad_state_before;
		}
		currquadtime = quad_time_before;
		reverttime(time_before);
	}
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
/*
simulation::simulation(double offset) {
	roomba_offset = offset;
	whack_offset = 0;

	//tests.push_back(trial(this));
}*/

void HSVtoRGB(double *r, double *g, double *b, double h, double s, double v) {
	int i;
	double f, p, q, t;
	if (s == 0) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;			// sector 0 to 5
	i = (int)floor(h);
	f = h - i;			// factorial part of h
	p = v * (1 - s);
	q = v * (1 - s * f);
	t = v * (1 - s * (1 - f));
	switch (i) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
		default:		// case 5:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

/*void draw(pair<double, double> in, HDC hdc, RECT* prc) {
	HDC hdcBuffer = CreateCompatibleDC(hdc);
	HBITMAP hbmBuffer = CreateCompatibleBitmap(hdc, prc->right, prc->bottom);
	HBITMAP hbmOldBuffer = (HBITMAP)SelectObject(hdcBuffer, hbmBuffer);

	HDC hdcMem = CreateCompatibleDC(hdc);
	HBITMAP hbmOld = (HBITMAP)SelectObject(hdcMem, g_hbmMask);

	FillRect(hdcBuffer, prc, (HBRUSH)GetStockObject(WHITE_BRUSH));

	//BitBlt(hdcBuffer, g_ballInfo.x, g_ballInfo.y, g_ballInfo.width, g_ballInfo.height, hdcMem, 0, 0, SRCAND);
	//Ellipse(hdcBuffer, (int)40 * (in.first - roombaDiameter / 2), (int)40 * (in.second - roombaDiameter / 2), (int)40 * (in.first + roombaDiameter / 2), (int)40 * in.second + roombaDiameter / 2);
	//SelectObject(hdcMem, g_hbmBall);
	//BitBlEt(hdcBuffer, g_ballInfo.x, g_ballInfo.y, g_ballInfo.width, g_ballInfo.height, hdcMem, 0, 0, SRCPAINT);

	//BitBlt(hdc, 0, 0, prc->right, prc->bottom, hdcBuffer, 0, 0, SRCCOPY); //writes

	//SelectObject(hdcMem, hbmOld);
	//DeleteDC(hdcMem);

	//SelectObject(hdcBuffer, hbmOldBuffer);
	//DeleteDC(hdcBuffer);
	//DeleteObject(hbmBuffer);
}*/

void trial::reverttime(double until) {
	while (!organizer.empty()) {
		organizer.pop();
	}
	if (currtime > until) { //going backwards
		for (int i = 0; i < 14; ++i) {
			while (currbots[i]->start_time > until) {
				currbots[i] = currbots[i]->parent;
				delete currbots[i]->child;
			}
			currbots[i]->initNotify(); //renotify
		}
		currtime = until;
		collision_handle();
	}
}

void trial::evaluatetime2(double until) {
	//cout << "Blah" << endl;
	//if (!organizer.empty()) {	//int a = 0;
	double interval = ((double)15 / (double)1000);
	while (currtime < until) {
		if ((currtime + interval) < until) {
			currtime += interval;
		}
		else {
			currtime = until;
		}

		collision_handle();
		while (!organizer.empty()) {
			pqueue_index first = organizer.top();
			if (first.at_time <= currtime) {
				organizer.pop();
				//cout << first.caller->ID << endl;
				//if (first.caller->child == nullptr) {
					first.caller->notify(first.at_time, first); //send notification
				/*}
				else while ((currbots[first.caller->ID]->child != nullptr)&&(currbots[first.caller->ID]->child->start_time <= currtime)) { //make it move through known motions
					currbots[first.caller->ID] = currbots[first.caller->ID]->child; //move down the line
				}*/
			}
			else {
				break;
			}
		}
	}
}



void trial::evaluatetime(double until) { //use this method to show things on the sketchy renderer.
	double interval = (double)15.0 / (double)1000.0;
	while (currtime < until) {
		evaluatetime2(currtime + interval);
		HDC hdcBuffer = CreateCompatibleDC(hd);
		HBITMAP hbmBuffer = CreateCompatibleBitmap(hd, re->right, re->bottom);
		FillRect(hdcBuffer, re, (HBRUSH)GetStockObject(WHITE_BRUSH));
		int savedDC = SaveDC(hdcBuffer);
		SelectObject(hdcBuffer, hbmBuffer);
		//aerial robot render code
		SelectObject(hdcBuffer, GetStockObject(DC_BRUSH));
		SetDCBrushColor(hdcBuffer, RGB(255, 255, 255));
		SelectObject(hdcBuffer, GetStockObject(DC_PEN));
		SetDCPenColor(hdcBuffer, RGB(255, 255, 255));
		//pair<double,double> st = currbots[i]->getposition(currtime-interval);
		pair<double, double> ed = currquad->getposition(currtime);
		SetDCBrushColor(hdcBuffer, RGB(255, 255, 255));
		//MoveToEx(hdc, (int)40*st.first, (int)40*st.second, NULL);
		//LineTo(hdc, (int)40*ed.first, (int)40*ed.second);
		Ellipse(hdcBuffer, (int)40 * (ed.first - roombaDiameter / 4), (int)40 * (ed.second - roombaDiameter / 4), (int)40 * (ed.first + roombaDiameter / 4), (int)40 * (ed.second + roombaDiameter / 4));

		for (int i = 0; i < 14; i++) {
			SelectObject(hdcBuffer, GetStockObject(DC_BRUSH));
			SetDCBrushColor(hdcBuffer, RGB(255, 0, 0));
			SelectObject(hdcBuffer, GetStockObject(DC_PEN));
			SetDCPenColor(hdcBuffer, RGB(0, 0, 0));
			//pair<double,double> st = currbots[i]->getposition(currtime-interval);
			pair<double, double> ed = currbots[i]->getposition(currtime);
			double red = 0, green = 0, blue = 0;
			double aaa = (until - currtime) / until;
			HSVtoRGB(&red, &green, &blue, (double)300 * (double)i / (double)13, 1, 0.8);
			SetDCBrushColor(hdcBuffer, RGB(red*(double)255, green*(double)255, blue*(double)255));
			//MoveToEx(hdc, (int)40*st.first, (int)40*st.second, NULL);
			//LineTo(hdc, (int)40*ed.first, (int)40*ed.second);

			Ellipse(hdcBuffer, (int)40 * (ed.first - roombaDiameter / 2), (int)40 * (ed.second - roombaDiameter / 2), (int)40 * (ed.first + roombaDiameter / 2), (int)40 * (ed.second + roombaDiameter / 2));

		}
		BitBlt(hd, 0, 0, re->right, re->bottom, hdcBuffer, 0, 0, SRCCOPY); //writes
		RestoreDC(hdcBuffer, savedDC);
		DeleteDC(hdcBuffer);
		DeleteObject(hbmBuffer);
	}
}

void trial::show(double until,HDC hdc, RECT * prc) { //use this method to show things on the sketchy renderer.
	double interval = (double)15.0 / (double)1000.0;
	while (currtime < until) {
		evaluatetime(currtime + interval);
		HDC hdcBuffer = CreateCompatibleDC(hdc);
		HBITMAP hbmBuffer = CreateCompatibleBitmap(hdc, prc->right, prc->bottom);
		FillRect(hdcBuffer, prc, (HBRUSH)GetStockObject(WHITE_BRUSH));
		int savedDC = SaveDC(hdcBuffer);
		SelectObject(hdcBuffer, hbmBuffer);
		for (int i = 0; i < 14; i++) {
			SelectObject(hdcBuffer, GetStockObject(DC_BRUSH));
			SetDCBrushColor(hdcBuffer, RGB(255, 0, 0));
			SelectObject(hdcBuffer, GetStockObject(DC_PEN));
			SetDCPenColor(hdcBuffer, RGB(0, 0, 0));
			//pair<double,double> st = currbots[i]->getposition(currtime-interval);
			pair<double, double> ed = currbots[i]->getposition(currtime);
			double red = 0, green = 0, blue = 0;
			double aaa = (until - currtime) / until;
			HSVtoRGB(&red, &green, &blue, (double)300 * (double)i / (double)13, 1, 0.8);
			SetDCBrushColor(hdcBuffer, RGB(red*(double)255, green*(double)255, blue*(double)255));
			//MoveToEx(hdc, (int)40*st.first, (int)40*st.second, NULL);
			//LineTo(hdc, (int)40*ed.first, (int)40*ed.second);

			Ellipse(hdcBuffer, (int)40 * (ed.first - roombaDiameter / 2), (int)40 * (ed.second - roombaDiameter / 2), (int)40 * (ed.first + roombaDiameter / 2), (int)40 * (ed.second + roombaDiameter / 2));

		}
		BitBlt(hdc, 0, 0, prc->right, prc->bottom, hdcBuffer, 0, 0, SRCCOPY); //writes
		RestoreDC(hdcBuffer, savedDC);
		DeleteDC(hdcBuffer);
		DeleteObject(hbmBuffer);
	}
}

void trial::showBestmove(HDC hdc, RECT * prc) {
	hd = hdc;
	re = prc;
	evaluationFunctor a;
	bestMove(a, predictUntil, nullptr);
}

double simp_dform(pair<double, double> a, pair<double, double> b) { //simple distance
	return square(a.first - b.first) + square(a.second - b.second);
}


/*

IN FINAL VERSION OF COLLISION HANDLER:
ONLY HAVE IT OPERATE ON ONE ROOMBA ACTION!
PUT EVENT NOTIFICATIONS FOR IT AND ANYTHING IT MIGHT TOUCH
ALSO
HAVE IT CHECK IF ANYTHING IS ALREADY TOUCHING IT!!!!!!!!!!!!!!!!!!!! (and pass out appropriate notifications)

*/


//make states have properties instead of being enums???


void trial::collision_handle() {
	for (int i = 0; i < 13; i++) {
		for (int j = i + 1; j < 14; j++) {
			pair<double, double> a = currbots[i]->getposition(currtime);
			pair<double, double> b = currbots[j]->getposition(currtime);
			if ((pow(roombaDiameter,2))>=simp_dform(a,b)) {
				if (fix_angle(atan2(b.second - a.second, b.first - a.first) - currbots[i]->getrotation(currtime) + pi / 2) <= pi) { //if other is in front
					if ((!clist[i][j]) || (currbots[i]->mytype == RUN) || (currbots[i]->mytype == RANDROT) || (currbots[i]->mytype == TOPTOUCH) || (currbots[i]->mytype == OBSTACLERUN)) {
						clist[i][j] = true;
						organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHED, currtime));
					}
				}
				else {
					//put some debug stuff here!
					if (clist[i][j]) {
						organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHENDED, currtime));
						clist[i][j] = false;
					}
				}
				if (fix_angle(atan2(a.second - b.second, a.first - b.first) - currbots[j]->getrotation(currtime) + pi / 2) <= pi) { //if other is in front
					if ((!clist[j][i]) || (currbots[j]->mytype == RUN) || (currbots[j]->mytype == RANDROT) || (currbots[j]->mytype == TOPTOUCH) || (currbots[i]->mytype == OBSTACLERUN)) {
						clist[j][i] = true;
						organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHED, currtime));
					}
				}
				else {
					if (clist[j][i]) {
						organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHENDED, currtime));
						clist[j][i] = false;
					}				
				}
			}
			else {
				if (clist[i][j]) {
					organizer.push(pqueue_index(currbots[i], currbots[j], TOUCHENDED, currtime));
					clist[i][j] = false;
				}
				if (clist[j][i]) {
					organizer.push(pqueue_index(currbots[j], currbots[i], TOUCHENDED, currtime));
					clist[j][i] = false;
				}
			}
		}
	}
}

void roomba_action::addevent(double at_time, notification type) {
	c_test->organizer.push(pqueue_index(this, nullptr, type, at_time));
}


void roomba_action::initNotify() {
	switch (mytype){
	case RUN:
		if ((next_reversal <= next_noise) || (next_reversal <= start_time)) { //reversal takes precedent
			end_time = max(next_reversal, start_time); //put reversal either now or at the next specified reversal time
			addevent(end_time, REVERSAL);
		}
		else {
			end_time = max(next_noise, start_time);
			addevent(end_time, NOISE);
		}
		break;
	case REVERSE:
		end_time = start_time+reverseLength;
		addevent(end_time,ENDED);
		break;
	case RANDROT: {
		end_time = start_time+noiseLength;
		addevent(end_time,ENDED);
		break;
	}
	case TOPTOUCH:
		end_time = start_time+reverseLength/4;
		addevent(end_time, ENDED);
		break;
	case OBSTACLERUN:
		end_time = runendtime;
		break;
	case OBSTACLESTOPPED:
		end_time = runendtime;
		break;
	default:
		break;
	}
}

double roomba_action::getrotation(double at_time) {
	double dt = at_time - start_time;
	if (abs(rw_v - lw_v) < 1.0e-6) {
		return initial_angle;
	}
	else {
		double wd = (rw_v * dt - lw_v * dt) / wheelbase_width;
		return initial_angle + wd;
	}
}

pair<double, double> roomba_action::getposition(double at_time) {
	if (start_time > at_time) { //make it work universally
		return parent->getposition(at_time);;
	}
	if ((child != nullptr) && (child->start_time < at_time)) {
		return child->getposition(at_time);
	}
	double dt = at_time - start_time;
	if (abs(rw_v - lw_v) < 1.0e-6) {
		return pair<double, double>(startpos.first + rw_v * dt * cos(initial_angle), startpos.second + rw_v * dt * sin(initial_angle));
	}
	else {
		double R = wheelbase_width * (rw_v * dt + lw_v * dt) / (2 * (rw_v * dt - lw_v * dt)),
			wd = (rw_v * dt - lw_v * dt) / wheelbase_width;
		return pair<double, double>(startpos.first + R * sin(wd + initial_angle) - R * sin(initial_angle), startpos.second - R * cos(wd + initial_angle) + R * cos(initial_angle));
	}
}

roomba_action::~roomba_action() {
	//roomba_action * temp = this;
	if (parent != nullptr) {
		parent->child = nullptr;
	}
	if (child != nullptr) {
		child->parent = nullptr;
		delete child;
	}
}


quad_action::~quad_action() {
	//roomba_action * temp = this;
	if (parent != nullptr) {
		parent->child = nullptr;
	}
	if (child != nullptr) {
		child->parent = nullptr;
		delete child;
	}
}

roomba_action::roomba_action(double s_time, pair<double, double> spos, double init_an,trial * curr_test,int i,bool killbot) { //init constructor
	if (!killbot) {
		//mytype = RUN;
		next_noise = noiseInterval;
		next_reversal = reverseInterval;
	}
	else {
		//mytype = OBSTACLERUN;
		next_noise = runendtime;
		next_reversal = runendtime;
	}
	mytype = STOPPED;
	is_obstacle = killbot;
	c_test = curr_test;
	c_sim = curr_test->c_sim;
	parent = nullptr;
	child = nullptr;
	startpos = spos;
	initial_angle = fix_angle(init_an);
	start_time = s_time;
	//cout << i;
	ID = i;
}

roomba_action::roomba_action(double at_time,roomba_action * par,state_type type) { //standard constructor
	mytype = type;
	c_test = par->c_test;
	c_sim = c_test->c_sim;
	parent = par;
	child = nullptr;
	next_noise = par->next_noise;
	next_reversal = par->next_reversal;
	startpos = par->getposition(at_time);
	initial_angle = fix_angle(par->getrotation(at_time));
	start_time = at_time;
	end_signal = -1;
	ID = par->ID;
}

void roomba_action::spawn_state(double at_time,state_type st) {
	//allows us to do weird stuff starting with a state change back in time and progressing forward, updating the sim as we go. I believe
	roomba_action * temp;
	if (child != nullptr) { 
		delete child; //important
	}
	end_time = at_time;
	temp = new roomba_action(at_time, this, st);
	child = temp;
	child->parent = this;
	switch (st){
		case RUN:
			child->rw_v = robotSpeed;
			child->lw_v = robotSpeed;
			break;
		case REVERSE:
			child->rw_v = -robotSpeed / 2;
			child->lw_v = robotSpeed / 2;
			break;
		case RANDROT: {
			double randv = rand() % randomMax;
			randv = randv - randomMax / 2;
			randv /= (double)1000.00;
			child->rw_v = robotSpeed - randv;
			child->lw_v = robotSpeed + randv;
			break;
		}
		case TOPTOUCH:
			child->rw_v = -robotSpeed / 2;
			child->lw_v = robotSpeed / 2;
			break;
		case OBSTACLERUN:
			child->rw_v = robotSpeed - (double)9 / (double)1000;
			child->lw_v = robotSpeed + (double)9 / (double)1000;
			break;
		case OBSTACLESTOPPED:
			child->rw_v = 0;
			child->lw_v = 0;
			break;
		default:
			break;
	}
	child->initNotify();
	c_test->currbots[ID] = temp;
}

trial::trial(simulation * sims) {
	c_sim = sims;
	currtime = 0;
	currquad = new quad_action(this,sims);
	currquadtime = currquad->end_time;
	for (int i = 0; i < 10; i++) {
		double ang = fix_angle((2 * pi / 10)*i/* + c_sim->roomba_offset*/);
		startbots[i] = new roomba_action(0.0, make_pair(10.0 + cos(ang), 10.0 + sin(ang)), ang, this, i, false);
		currbots[i] = startbots[i];
		currbots[i]->spawn_state(0, RUN);
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
	for (int i = 0; i < 4; i++) {
		double ang = fix_angle((2 * pi / 4)*i/* + c_sim->whack_offset*/);
		startbots[10+i] = new roomba_action(0.0, make_pair(10.0 + 5.0*cos(ang), 10.0 + 5.0*sin(ang)), fix_angle(ang-pi/2.0), this, i+10.0, true);
		currbots[10+i] = startbots[i+10];
		currbots[10+i]->spawn_state(0, OBSTACLERUN);
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
}

void roomba_action::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time) &&(c_test->currbots[ID] == this)) {
		switch (info.type) {
		case TOUCHED:
			if (((mytype == RUN) || (mytype == RANDROT) || (mytype == TOPTOUCH))) { //check states to ensure they're legit
				if ((info.linked == nullptr) || (info.linked->end_time >= at_time)) {
					spawn_state(at_time,REVERSE);
				}
			}
			else if ((mytype == OBSTACLERUN) && (info.linked != nullptr)) {
				if (info.linked->end_time >= at_time) {
					spawn_state(at_time, OBSTACLESTOPPED);
				}
			}
			break;
		case REVERSAL:
			if (mytype == RUN) {
				spawn_state(at_time, REVERSE);
				child->next_reversal = at_time + noiseInterval;  //tricksy
			}
			break;
		case NOISE:
			if (mytype == RUN) {
				spawn_state(at_time, RANDROT);
				child->next_noise = at_time + noiseInterval;
			}
			break;
		case MAGNET:
			if ((mytype == RANDROT) || (mytype == REVERSE) || (mytype == RUN)) { ///
				spawn_state(at_time, TOPTOUCH);
			}
			break;
		case ENDED:
			if ((mytype == RANDROT) || (mytype == REVERSE) || (mytype == TOPTOUCH)) {
				spawn_state(at_time, RUN);
			}
			break;
		case TOUCHENDED:
			if (mytype == OBSTACLESTOPPED) {
				spawn_state(at_time, OBSTACLERUN);
			}
			break;
		default:
			break;
		}
	}
}