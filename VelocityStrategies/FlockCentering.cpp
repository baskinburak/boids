#include "FlockCentering.h"
FlockCentering::FlockCentering(double imp, double nt) : IVelStrat(imp, nt) {
  
}


IVelStrat::Suggestion FlockCentering::suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status) {
  Col<double> suggestion = zeros<Col<double> >(3);
  double cnt = 0;


  for(map<int, BoidStatus>::iterator it = flock_status.begin(); it != flock_status.end(); it++) {
    BoidStatus& oth_boid = it->second;
    if(my_status.distance(oth_boid) <= this->neigh_thresh) {
      suggestion += oth_boid.get_position();
      cnt+=1;
    }
  }

  Col<double> angsugg = zeros<Col<double> >(3);

  if(cnt != 0)
    suggestion = suggestion/cnt;

  return IVelStrat::Suggestion(normalise(suggestion), angsugg, norm(suggestion));
}
