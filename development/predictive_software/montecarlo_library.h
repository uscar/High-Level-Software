#define _USE_MATH_DEFINES
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




//NOT COMPLETE, PREDICTION IS NOT FULLY FUNCTIONAL.
//I just need to get the aerial robot behavior implemented and drawn


//TO RE-ADD DRAWING: change evaluatetime implementation to evaluate at intervals of like 15 msec and then print out every time you do that

//need to pass out collision certificates to aerial bot too

using namespace std;

class simulation;
class quad_action;
class roomba_action;
const int numroombas = 10;


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

double quadSpeedEst = 3.0; //m/s

double wheelbase_width = 0.25798379655423864; //worked backwards to this

double fix_angle(double in_afngle); //all in milliseconds
double roombaDiameter = 0.34;

double runendtime = 600; //default state end time


double topTouchTimereq = 100;
double bumperTimereq = 100;



enum notification { TOUCHED, REVERSAL, NOISE, MAGNET, ENDED,TOUCHENDED};
enum state_type { REVERSE, RUN, RANDROT, TOPTOUCH, STOPPED,OBSTACLERUN,OBSTACLESTOPPED,TOTALSTOP};
enum quad_state_type {BUMP, FLY, TAP, ASCEND, STOPPED,FLYTHROUGH};

struct trial;
class roomba_action;
struct pqueue_index;
class simulation;

double fix_angle(double in_angle) {
	while (in_angle >= 2 * M_PI) {
		in_angle -= 2 * M_PI;
	}
	while (in_angle < 0) {
		in_angle += 2 * M_PI;
	}
	return in_angle;
}

double distance(pair<double, double> & a, pair<double, double> & b) {
	return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second)*(a.second - b.second));
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
	int action = 0;
	int score = 0;
	int special = 0;
	bool operator<(actionval & other) {
		return score < other.score;
	}
};

struct evaluationFunctor {
	virtual int operator()(trial * a, double time) = 0 {

	}
};

struct trial {
	void show(double until, HDC hdc, RECT * prc);
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
	vector<actionval> * bestMove(evaluationFunctor & a,double until);
	vector<actionval> * getActions(double time);
	static vector<actionval> actions;
	quad_action * quad; //this trial's quad action
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
	double dx = 0;
	double dy = 0;
	double orientation; //unused currently
	quad_action * parent;
	quad_action * child;
	trial * c_test;
	simulation * c_sim;
	roomba_action * tracking;
	//void append();
	quad_state_type mytype;
	bool executeAction(actionval & a,double time,double until);
	bool flyTo(int roomba,double time,double until);
	double simpleTimeEst(actionval & a,double time);
	pair<double, double> getposition(double at_time);
	~quad_action();
};

roomba_action fetchInterceptState(roomba_action * targ,quad_action * copter,double cap) { //can quad reach this roomba state within time bounds? If not, is there too much time or too little.
	double dist_to, coverable;
	do {
		dist_to = distance(cstate->startpos, copter->startpos);
		coverable = quadSpeedEst * (cap - copter->beginTime);
		if (dist_to >= coverable) {
			return targ->child; //return state we're going to run into. could be nullptr
		}
		cap = targ->start_time;
		targ = targ->parent;
	} while (dist_to < coverable)
}

bool quad_action::flyTo(int roomba, double until) { //cut time before calculation done
	double beginTime = c_test->currquadtime;
	double workingTime = beginTime;
	roomba_action * cstate = c_test->currbots[roomba];
	double cap = min(c_test->currtime, until);//upper time bound
	cstate = fetchIntceptState(cstate, this, cap);
	while (cstate == nullptr) {
		if (c_test->currtime >= until) { //if we've exceeded the max allotted time
			return false; //cannot be flown to
		}
		else {
			c_test->evaluatetime(min(c_test->currtime+1, until)); //extends prediction up to 1 second into the future, replace this with something more elegant later
		}
	}
	double dist_to, coverable;
	dist_to = distance(cstate->startpos, copter->startpos);
	coverable = quadSpeedEst * (cap - copter->beginTime);
	workingTime = (cstate->start_time - cstate->end_time) / 2;
	while () {
		
	}
	//figure out intercept time
	//while (workingTime < cap) { //once we've gotten all that sorted out and figured out what our appropriate state is
	//}
}

double quad_action::simpleTimeEst(actionval & a, double time) {
	return distance(c_test->currbots[a.roomba]->getposition(time), getposition(time));
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
	//actionval simulation::branch();
};


vector<actionval> * trial::getActions(double time) {
	vector<actionval> * a = new vector<actionval>(actions);
	auto comp = [&](actionval & a, actionval & b) {
		return (quad->simpleTimeEst(a,time) < quad->simpleTimeEst(b,time));
	};
	sort(a->begin(), a->end(),comp);
	return a;
}

//recursive call. use distance to estimate actual time required. sort actionvals by this


vector<actionval> * trial::bestMove(evaluationFunctor & a,double until) {
	vector<actionval> * act = getActions(currtime); //currtime is needed to sort
	for (auto && it : (*act)) {
		double quad_time_before = currquadtime; 
		quad_action * quad_state_before = quad;
		bool flew = quad->flyTo(it.roomba, currquadtime, until); //similair to evaluatetime except keeps going till quad gets to the roomba
		double time_before = currtime;
		roomba_action * state_before = currbots[it.roomba]; //save current state of roomba we're about to mess with
		bool interacted = quad->executeAction(it, currquadtime,until);
		if (flew && interacted) { //pick best member
			vector<actionval> * ret = bestMove(a, until);
			it.score = max_element(ret->begin(),ret->end())->score;
			delete ret;
		}
		else {
			it.score = a(this, until);
		}
		if (state_before->child) {
			delete state_before->child; //eliminate any trace of the action happening
			currbots[state_before->ID] = state_before;
		}
		if (quad_state_before->child) {
			delete quad_state_before->child; //eliminate any trace of the action happening
			quad = quad_state_before;
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
	actionval ** values = new actionval *[tests.front().actions.size()]; //first index is for each action, second index is for each trial
	for (int i = 0; i < tests.size(); ++i) {
		values[i] = new actionval[tests.size()];
	}
	{
		int i = 0;
		for (auto && it : tests) {
			vector<actionval> * retv = it.bestMove(a, until); //get best moves for the trial
			for (int j = 0; j < it.actions.size(); ++j) {
				values[j][i] = (*retv)[j]; //enter it in a useful way
			}
			++i;
		}
	}
	//double * results = new double[tests.front.actions.size()];
	int bestaction = 0;
	sort(values[0], values[0] + tests.size());
	int bestvalue = getMedian(values[0], tests.size());
	for (int i = 1; i < tests.front().actions.size(); ++i) {
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
	i = floor(h);
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
	//HDC hdcBuffer = CreateCompatibleDC(hdc);
	//HBITMAP hbmBuffer = CreateCompatibleBitmap(hdc, prc->right, prc->bottom);
	//HBITMAP hbmOldBuffer = (HBITMAP)SelectObject(hdcBuffer, hbmBuffer);

	//HDC hdcMem = CreateCompatibleDC(hdc);
	//HBITMAP hbmOld = (HBITMAP)SelectObject(hdcMem, g_hbmMask);

	//FillRect(hdcBuffer, prc, (HBRUSH)GetStockObject(WHITE_BRUSH));

	//BitBlt(hdcBuffer, g_ballInfo.x, g_ballInfo.y, g_ballInfo.width, g_ballInfo.height, hdcMem, 0, 0, SRCAND);
	Ellipse(hdcBuffer, (int)40 * (in.first - roombaDiameter / 2), (int)40 * (in.second - roombaDiameter / 2), (int)40 * (in.first + roombaDiameter / 2), (int)40 * in.second + roombaDiameter / 2);
	//SelectObject(hdcMem, g_hbmBall);
	//BitBlt(hdcBuffer, g_ballInfo.x, g_ballInfo.y, g_ballInfo.width, g_ballInfo.height, hdcMem, 0, 0, SRCPAINT);

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

void trial::evaluatetime(double until) {
	//if (!organizer.empty()) {	//int a = 0;
	double interval = ((double)15 / (double)1000);
	while (currtime < until) {
		//double interval = ((double)15 / (double)1000);
		if ((currtime + interval) <= until) {
			currtime += interval;
		}
		else {
			currtime = until;
		}
		//currtime += ((double)15/(double)1000);

		collision_handle();
		while (!organizer.empty()) {
			pqueue_index first = organizer.top();
			if (first.at_time <= currtime) {
				organizer.pop();
				//cout << first.caller->ID << endl;
				if (first.caller->child == nullptr) {
					first.caller->notify(first.at_time, first); //send notification
				}
				else while ((currbots[first.caller->ID]->child != nullptr)&&(currbots[first.caller->ID]->child->start_time <= currtime)) { //make it move through known motions
					currbots[first.caller->ID] = currbots[first.caller->ID]->child; //move down the line
				}
			}
			else {
				break;
			}
		}
	}
}

void trial::show(double until,HDC hdc, RECT * prc) { //use this method to show things on the sketchy renderer.
	double interval = 30 / 1000;
	while (currtime < until) {
		evaluatetime(until + interval);
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

double simp_dform(pair<double, double> a, pair<double, double> b) { //simple distance
	return pow((a.first - b.first), 2) + pow((a.second - b.second), 2);
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
				if (fix_angle(atan2(b.second - a.second, b.first - a.first) - currbots[i]->getrotation(currtime) + M_PI / 2) <= M_PI) { //if other is in front
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
				if (fix_angle(atan2(a.second - b.second, a.first - b.first) - currbots[j]->getrotation(currtime) + M_PI / 2) <= M_PI) { //if other is in front
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
		rw_v = robotSpeed;
		lw_v = robotSpeed;
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
		rw_v = -robotSpeed/2;
		lw_v = robotSpeed/2;
		end_time = start_time+reverseLength;
		addevent(end_time,ENDED);
		break;
	case RANDROT: {
		double randv = rand() % randomMax;
		randv = randv - randomMax / 2;
		randv /= (double)1000.00;
		rw_v = robotSpeed - randv;
		lw_v = robotSpeed + randv;
		end_time = start_time+noiseLength;
		addevent(end_time,ENDED);
		break;
	}
	case TOPTOUCH:
		rw_v = -robotSpeed / 2;
		lw_v = robotSpeed / 2;
		end_time = start_time+reverseLength/4;
		break;
	case OBSTACLERUN:
		rw_v = robotSpeed - (double)9/(double)1000;
		lw_v = robotSpeed + (double)9/(double)1000;
		end_time = runendtime;
		break;
	case OBSTACLESTOPPED:
		rw_v = 0;
		lw_v = 0;
		end_time = runendtime;
		break;
	default:
		break;
	}
}

double roomba_action::getrotation(double at_time) {
	double dt = at_time - start_time;
	if (fabs(rw_v - lw_v) < 1.0e-6) {
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
	if (fabs(rw_v - lw_v) < 1.0e-6) {
		return make_pair(startpos.first + rw_v * dt * cos(initial_angle), startpos.second + rw_v * dt * sin(initial_angle));
	}
	else {
		double R = wheelbase_width * (rw_v * dt + lw_v * dt) / (2 * (rw_v * dt - lw_v * dt)),
		wd = (rw_v * dt - lw_v * dt) / wheelbase_width;
		return make_pair(startpos.first + R * sin(wd + initial_angle) - R * sin(initial_angle), startpos.second - R * cos(wd + initial_angle) + R * cos(initial_angle));
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
		mytype = RUN;
		next_noise = noiseInterval;
		next_reversal = reverseInterval;
	}
	else {
		mytype = OBSTACLERUN;
		next_noise = runendtime;
		next_reversal = runendtime;
	}
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
	initNotify();
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
	ID = par->ID;
	initNotify();
}

void roomba_action::spawn_state(double at_time,state_type st) {
	roomba_action * temp;
	temp = new roomba_action(at_time, this, st);
	end_time = at_time;
	child = temp;
	child->parent = this;
	c_test->currbots[ID] = temp;
}

trial::trial(simulation * sims) {
	c_sim = sims;
	for (int i = 0; i < 10; i++) {
		double ang = fix_angle((2 * M_PI / 10)*i/* + c_sim->roomba_offset*/);
		startbots[i] = new roomba_action(0.0, make_pair(10.0 + cos(ang), 10.0 + sin(ang)), ang, this, i, false);
		currbots[i] = startbots[i];
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
	for (int i = 0; i < 4; i++) {
		double ang = fix_angle((2 * M_PI / 4)*i/* + c_sim->whack_offset*/);
		startbots[10+i] = new roomba_action(0.0, make_pair(10.0 + 5.0*cos(ang), 10.0 + 5.0*sin(ang)), fix_angle(ang-M_PI/2.0), this, i+10.0, true);
		currbots[10+i] = startbots[i+10];
		for (int j = 0; j < 14; j++) {
			clist[i][j] = false;
		}
	}
}

void roomba_action::notify(double at_time, pqueue_index info) {
	if ((start_time <= at_time) && (at_time <= end_time) &&(c_test->currbots[ID] == this)) {
		switch (info.type) {
		case TOUCHED:
			if (((mytype == RUN) || (mytype == RANDROT) || (mytype == TOPTOUCH)) && (info.linked != nullptr)) {
				if (info.linked->end_time >= at_time) {
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
				next_reversal = at_time + reverseInterval;

				spawn_state(at_time, REVERSE);
			}
			break;
		case NOISE:
			if (mytype == RUN) {
				next_noise = at_time + noiseInterval;

				spawn_state(at_time, RANDROT);
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