#ifndef BOIDS_FLOCKCENTERING_H
#define BOIDS_FLOCKCENTERING_H
#include "IVelStrat.h"
#include <map>
using namespace std;
class FlockCentering : public IVelStrat {
  public:
    FlockCentering(double imp, double nt);

    IVelStrat::Suggestion suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status);
};

#endif
