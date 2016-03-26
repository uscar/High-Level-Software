#include "quad_action.h"

double quad_action::quadParamLineIntersect(vec2d roombaDisplacement, vec2d roombaVelocity, double quadVelocity) {
	double a = roombaVelocity.square_components() - (quadVelocity*quadVelocity);
	double b = 2 * roombaVelocity.dot(roombaDisplacement);
	double c = roombaDisplacement.getMagnitude();
	return (-b - sqrt((b*b) - (4 * a*c))) / (2 * a); //I feel like this math should not come as a shock to anyone
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
