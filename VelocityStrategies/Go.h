#ifndef BOIDS_GO_H
#define BOIDS_GO_H
#include "IVelStrat.h"
#include <map>
using namespace std;

class Go : public IVelStrat {
  private:
    Col<double> destination;

  public:
    Go(double imp, double nt, Col<double>& dst);

    IVelStrat::Suggestion suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status);
};

#endif
