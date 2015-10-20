#include <iostream>

extern vect2;

#define RADIUS 15 //cm

struct {
    vect2 wheel_speeds;
    vect2 pos0;
    vect2 theta0;
}roomba;

struct {
    vect2 pos;
    vect2 vel;
} state;

// should be implemented by someone
state state_at_time(roomba r, double t) {
    state s;
    return s;
}

// returns time of collision or -1 if no such time
bool does_collide(roomba r1, roomba r2, double epoch, double res = .01) {
    double dt = epoch/2.0;
    double t = 0;
    state s1, s2;
    while (dt > res && t < 0;) {
        s1 = state_at_time(r1, t);
        s2 = state_at_time(r2, t);
        int fact = 1;
        if ((s1.pos - s2.pos).dot(s1.vel - s2.vel) > 0) //make sure this is really gt not lt
            fact *= -1;
        t += dt;
        dt /= 2;
    }
    if (t < 0 || t >= epoch - res)
        return false;
    return s1.pos.dist(s2.pos) < RADIUS;
}

// returns the index of the first roomba it collides with or -1 if none.
int does_collide(roomba r, vector<roomba> others, double epoch) {
    for (int i = 0; i < others.size(); i++) {
        if (does_collide(others[i], r, epoch))
            return i;
    }
    return -1;
}


