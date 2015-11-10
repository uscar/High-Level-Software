#pragma once
#include <string>
#include "vec2d.h"
class robot_state { //I've been writing too much java can someone please send a search and rescue party
public:
	struct color {
		double r=0;
		double g=0;
		double b=0;
	};
	color c;
	virtual double getHeight(double at_time) = 0;
	virtual std::string getName() = 0;
	virtual vec2d getPosition(double at_time) = 0;
	virtual double getAngle(double at_time) = 0;
};