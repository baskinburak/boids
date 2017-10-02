#ifndef BOIDS_IVELSTRAT_H
#define BOIDS_IVELSTRAT_H
#include <armadillo>
#include "../BoidStatus.h"
#include <map>

using namespace arma;
using namespace std;

class IVelStrat {
  public:
    double importance;
    double neigh_thresh;

    IVelStrat(double i, double n);

    class Suggestion {
      public:
        Col<double> lin;
        Col<double> ang;
        double weight;

        Suggestion(Col<double> l, Col<double> a, double w);
    };

    bool operator<(const IVelStrat& rhs);

    virtual Suggestion suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status) = 0;
    
};

#endif
