#include <iostream>
#include "vec2d.h"
#include "base_classes.h"

#define pretentious false


//also we know you think you're cool but "const double" and "define" both just evaluate completely at compiler time

#if pretentious
	#define RADIUS 0.34*100.0/2.0 //cm
#else
	const double RADIUS = 0.34*100.0 / 2.0;
#endif

const double robotSpeed = (double)330 / (double)1000; //also, the reason I write it like this is to keep it somewhat consistent with the arduino file where I got these numbers



/*
//there's a pure virtual class called robot_state in "base_classes.h" that has these methods:

class robot_state {
	virtual vec2d getPosition(double at_time) = 0;
	virtual double getAngle(double at_time) = 0;
};

I think you should have all you need from this
*/

//Also, there's no such thing as a 'roomba' class, sadly. Only states.

vec2d getVelocity(robot_state * in,double at_time) {
	return (in->getAngle(at_time))*robotSpeed;
}


//I have no idea what you do here but that while loop scares me

// returns time of collision or -1 if no such time
bool does_collide(robot_state * r1, robot_state * r2, double epoch = 5.0, double res = .01) {
    double dt = epoch/2.0;
	double t_init;
    double t = 0;
    while (dt > res && t >= 0) { //what are these shenanigans
		int fact = (r1->getPosition(t) - r2->getPosition(t)).dot(getVelocity(r1, t) - getVelocity(r2, t)) > 0 ? -1 : 1;
        t += fact * dt;
        dt /= 2;
    }
    if (t < 0 || t >= epoch - res)
        return false;
    return s1.pos.dist(s2.pos) < 2*RADIUS;
}

// returns the index of the first roomba it collides with or -1 if none. (we actually need all the collisions, but fortunately this is easy to modify)
int does_collide(int r, robot_state ** others, double epoch) {
    for (int i = 0; i < 14; ++i) {
		if (i != r) {
			if (does_collide(others[r], others[i], epoch)) {
				return i;
			}
		}
    }
    return -1;
}

