#ifndef BOIDS_VELOCITYMATCHING_H
#define BOIDS_VELOCITYMATCHING_H
#include "IVelStrat.h"
#include <map>
using namespace std;

class VelocityMatching : public IVelStrat {
  public:
    VelocityMatching(double imp, double nt);

    IVelStrat::Suggestion suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status);
};

#endif
