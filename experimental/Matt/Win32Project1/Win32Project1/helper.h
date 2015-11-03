#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
double fix_angle(double in_angle) {
	while (in_angle >= 2 * M_PI) {
		in_angle -= 2 * M_PI;
	}
	while (in_angle < 0) {
		in_angle += 2 * M_PI;
	}
	return in_angle;
}

double square(double a) { //fairly certain pow has a special case for this but if not, doesn't hurt to write something like this
	return a*a;
}

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