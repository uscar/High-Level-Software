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
bool does_collide(roomba r1, roomba r2, double epoch,double t0 = 0, double res = .01) {
    double dt = epoch/2.0;
    double t = t0;
    state s1, s2;
    while (dt > res && t < t0;) {
        s1 = state_at_time(r1, t);
        s2 = state_at_time(r2, t);
        int fact = 1;
        if ((s1.pos - s2.pos).dot(s1.vel - s2.vel) > 0) //make sure this is really gt not lt
            fact *= -1;
        t += dt*fact;
        dt /= 2;
    }
    
    if (t < t0 || t >= epoch - res || s1.pos.dist(s2.pos) > 2*RADIUS)
        return -1;
  
    dt = epoch/2.0; 
    while (dt > res) {
        s1 = state_at_time(r1, t);
        s2 = state_at_time(r2, t);
        int fact = 1;
        if (s1.pos.dist(s2.pos) < 2*RADIUS)
            fact *= -1;
        t += dt*fact;
        dt /= 2;
    }
    return t;
}

// returns the first collision or -1 if no collisions
double does_collide(roomba r, vector<roomba> others, double epoch, double t0 = 0, double res = .01) {
    const double m = 100000;
    double collision = m;
    double tmp;
    for (int i = 0; i < others.size(); i++) {
        if ((tmp = does_collide(others[i], r, epoch, t0, res)) != -1)
            collision = collision<tmp?collision:tmp;
    }
    return collision == m ? -1 : collision;
}


