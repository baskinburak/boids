#ifndef BOIDS_COLLISIONAVOIDANCE_H
#define BOIDS_COLLISIONAVIODANCE_H
#include "IVelStrat.h"
#include <map>
using namespace std;

class CollisionAvoidance : public IVelStrat {
  public:
    CollisionAvoidance(double imp, double nt);

    IVelStrat::Suggestion suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status);
};

#endif
