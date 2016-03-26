#pragma once
#include <utility>
#include <cmath>

//TODO: Overhaul with MKL methods

struct vec2d : public std::pair<double, double> { //phase out pair<double,double>s in favor of this
	vec2d(const double &, const double &);
	vec2d(const double &); //angle constructor. basically the opposite of atan2
	vec2d(const std::pair<double, double> &);
	vec2d();
	double getMagnitude();
	double square_components(); //a^2 + b^2
	double dist_to(const vec2d & other);
	vec2d operator+(const vec2d & other);
	vec2d operator-(const vec2d & other);
	double dot(const vec2d & other);
};

vec2d::vec2d() {}

double vec2d::dot(const vec2d & other) {
	return first*other.first + second*other.second;
}

double vec2d::square_components() {
	return first*first + second*second;
}

double vec2d::getMagnitude() {
	return sqrt(square_components());
}

double vec2d::dist_to(const vec2d & other) {
	return (operator-(other)).getMagnitude();
}

vec2d::vec2d(const double & a, const double & b) {
	first = a;
	second = b;
}

vec2d::vec2d(const double & angle) {
	first = std::cos(angle);
	second = std::sin(angle);
}

vec2d::vec2d(const std::pair<double, double> & a) {
	first = a.first;
	second = a.second;
}

vec2d operator* (const double & a, const vec2d& b) {
	return vec2d(a*b.first, a*b.second);
}

vec2d operator* (const vec2d& a, const double & b) {
	return vec2d(a.first*b, a.second*b);
}

vec2d vec2d::operator+(const vec2d & a) {
	return vec2d(first + a.first, second + a.second);
}

vec2d vec2d::operator-(const vec2d & a) {
	return vec2d(first - a.first, second - a.second);
}