#include <iostream>
#include <algorithm>
#include "vec2d.h"
#include "base_classes.h"

#define pretentious false


//also we know you think you're cool but "const double" and "define" both just evaluate completely at compiler time

#if pretentious
	#define RADIUS 0.34*100.0/2.0 //cm
#else
	const double diameter = 0.34;
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
double collide_time(robot_state * r1, robot_state * r2, double epoch = -1, double res = .001) {
	double min = std::max(r1->start_time, r2->start_time); //get latest start and earliest end
	double max = std::min(r1->end_time, r2->end_time);
	if (epoch == -1) {
		epoch = max-min; //get minimum possible span of interaction
		if (epoch <= 0) return -1;
	}
	//checks if they would even hit within the epoch if they were both going full speed right toward each other
	if (epoch*robotSpeed*2 < (r1->getPosition(min) - r2->getPosition(min)).getMagnitude()) { //might update this later to include more cases. this makes our lives a lot easier though
		return -1;
	}
	double epoch;
    double dt = epoch/2.0;
	double t_init;
    double t = 0;
	double tot;
    while (dt > res && t >= 0) { //what are these shenanigans
		tot = min + t;
		int fact = (r1->getPosition(tot) - r2->getPosition(tot)).dot(getVelocity(r1, tot) - getVelocity(r2, tot)) > 0 ? -1 : 1;
        t += fact * dt;
        dt /= 2;
    }
    if (t < 0 || t >= epoch - res)
        return -1;
    return s1.pos.dist(s2.pos) < diameter;
}

// returns the index of the first roomba it collides with or -1 if none.
/* It would probably be easier to implement this within the main code because reasons
int does_collide(int r, robot_state ** others, double epoch) {  //I can prob ably implement something like this in the
    for (int i = 0; i < 14; ++i) {
		if (i != r) {
			if (does_collide(others[r], others[i], epoch)) {
				return i;
			}
		}
    }
    return -1;
}
*/
