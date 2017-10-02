#include "CollisionAvoidance.h"
#include <algorithm>
using namespace std;

CollisionAvoidance::CollisionAvoidance(double imp, double nt) : IVelStrat(imp, nt) {
  
}

IVelStrat::Suggestion CollisionAvoidance::suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status) {
  double weight = 0;
  Col<double> suggested_linear = zeros<Col<double> >(3);
  for(map<int, BoidStatus>::iterator it = flock_status.begin(); it != flock_status.end(); it++) {
    BoidStatus& oth_status = it->second;
    double dist;
    if((dist = my_status.distance(oth_status)) <= this->neigh_thresh) {
      pair<double, double> closest_distance = my_status.closest_distance(oth_status); // closest distance that will occur 1 sec move
      if(closest_distance.second == numeric_limits<double>::infinity()) {
        continue;
      }
      Col<double> escape_traj = my_status.escape(oth_status);
      double imp = 1.0 / (max(dist, 0.00001) * max(closest_distance.first, 0.00001) * max(closest_distance.second, 0.000001));
      suggested_linear += imp * escape_traj;
      weight += imp;
    }
  }

  Col<double> angsugg = zeros<Col<double> >(3);

  return IVelStrat::Suggestion(normalise(suggested_linear), angsugg, weight);
}
