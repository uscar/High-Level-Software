#include "roomba_action.h"
#include "helper.h"
#include <list>
#include <mkl.h>

roomba_action::roomba_action(double s_time, vec2d spos, double init_an,roomba * contain) {
	container = contain;
	parent = nullptr;
	child = nullptr;
	startpos = spos;
	initial_angle = fix_angle(init_an);
	start_time = s_time;
	saved = new savedValues(this);
}

roomba_action::roomba_action(double at_time, roomba_action * par, state_type type) { //standard constructor
	par->end_time = at_time;
	if (par->child != nullptr) {
		delete par->child; //important
	}
	container = par->container;
	par->child = this;
	par->end_time = at_time;
	mytype = type;
	parent = par;
	child = nullptr;
	next_noise = par->next_noise;
	next_reversal = par->next_reversal;
	startpos = par->getPosition(at_time);
	initial_angle = fix_angle(par->getAngle(at_time));
	start_time = at_time;
	saved = new savedValues(this);
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
	if (dependency != nullptr) {
		dependency->dependents.erase(this); //remove ourself from list o' dependents
	}
	for (roomba_action * a : dependents) {
		a->dependency = nullptr;
		delete a;
	}
	//invalidate events in queue
	for (pqueue_index *& eve : events) {
		//if (eve->type == INVALIDATED) {
		if (eve != nullptr) {
			delete eve;
		}
		//}
		//else {
		//	eve->type = INVALIDATED;
		//}
	}
}

roomba_action::savedValues::savedValues(roomba_action * act) : start(act->startpos), init_ang(act->initial_angle), rwv(act->rw_v), lwv(act->lw_v) {}

void roomba_action::savedValues::init() {
	condition = abs(rwv - lwv) < 1.0e-6;
	if (abs(rwv - lwv) < 1.0e-6) {
		vec1 = (rwv)*vec2d(init_ang);
	}
	else {
		R = wheelbase_width*(rwv + lwv) / (2 * (rwv - lwv));
		wd_init = (rwv - lwv) / wheelbase_width;
		vec1 = vec2d(-sin(init_ang), cos(init_ang));
	}
}

vec2d roomba_action::savedValues::doCalculation(double dt) {
	if (condition) {
		return start + vec1*dt;
	}
	else {
		wd = dt*wd_init + init_ang;
		vdSinCos(1, &wd, &sinv, &cosv);
		return start + R*(vec1 + vec2d(sinv, -cosv));
	}
}

void roomba_action::resetHere() { //kill children and local dependent states

}

vec2d roomba_action::getPosition(double at_time) {
	if (start_time > at_time) { //make it work universally
		if (parent != nullptr) {
			return parent->getPosition(at_time);
		}
		else {
			at_time = start_time; //make sure we don't overshoot dumb estimates
		}
	}
	if (end_time < at_time) {
		if (child != nullptr) {
			return child->getPosition(at_time);
		}
		else {
			at_time = end_time; //make sure we don't overshoot dumb estimates
		}
	}
	double dt = at_time - start_time;
	return saved->doCalculation(dt);
}


double roomba_action::getAngle(double at_time) {
	double dt = at_time - start_time;
	if (abs(rw_v - lw_v) < 1.0e-6) {
		return initial_angle;
	}
	else {
		double wd = saved->wd_init*dt;
		return initial_angle + wd;
	}
}

void roomba_action::addDependentState(roomba_action * other) {
	dependents.insert(other);
}

bool roomba_action::isQuad() { return false; }

bool roomba_action::is_touching(roomba_action * r,double at_time) {
	vec2d a = getPosition(at_time);
	vec2d b = r->getPosition(at_time);
	if (square(roombaDiameter) >= (a - b).square_components()) {
		if (fix_angle(atan2(b.second - a.second, b.first - a.first) - getAngle(at_time) + pi / 2) <= pi) { //if other is in front
			if ((mytype == RUN) || (mytype == RANDROT) || (mytype == TOPTOUCH) || (mytype == OBSTACLERUN)) {
				return true;
			}
		}
	}
	return false;
}

//Nolan's collision code rewritten quite a bit to match the specifics of our implementation, as well as adding some changes for efficiency
double roomba_action::does_collide(roomba_action * r, double epoch = 5.0, double t0 = 0, double res = .01) {
	roomba_action * r1 = this, * r2 = r;
	//personal touch
	double start = std::max(r1->start_time, r2->start_time);
	double finish = std::min(r1->end_time, r2->end_time);
	double epoch = std::min(finish - start, epoch);

	//checks if they couldn't possibly collide anyway
	//rather than get max velocity times 2 I summed their average wheel speeds

	if (((r1->rw_v + r1->lw_v + r2->rw_v + r2->lw_v) / 2.0) < r1->getPosition(start).dist_to(r2->getPosition(start))) { //could make a more elegant check later but this will suffice
		return -1;
	}

	double dt = epoch / 2.0;
	double t = start;
	vec2d p1, p2, v1, v2;
	p1 = r1->getPosition(t);
	p2 = r2->getPosition(t);
	v1 = vec2d(r1->getAngle(t));
	v2 = vec2d(r2->getAngle(t));

	//const int sign = ((p1 - p2).dot(v1 - v2)>0)*2-1;
	while (dt > res && t >= start && t <= finish) {
		t += dt*(2 * ((p1 - p2).dot(v1 - v2) > 0) - 1); //make sure this is really gt not lt ('sign' term ensures no matter what that it's consistent)
		dt /= 2;
		p1 = r1->getPosition(t);
		p2 = r2->getPosition(t);
		v1 = vec2d(r1->getAngle(t));
		v2 = vec2d(r2->getAngle(t));
	}

	if ((t < start) || (t >(start + epoch)) || (p1.dist_to(p2) > roombaDiameter)) {
		return -1;
	}

	dt = epoch / 2.0;
	while (dt > res) {
		t += dt*(2 * (p1.dist_to(p2) < roombaDiameter) - 1);
		dt /= 2;
		p1 = r1->getPosition(t);
		p2 = r2->getPosition(t);
	}
	return t;
}
