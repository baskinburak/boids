#include "Go.h"
#include <algorithm>
using namespace std;

Go::Go(double imp, double nt, Col<double>& dst) : IVelStrat(imp, nt), destination(dst) {
  
}

IVelStrat::Suggestion Go::suggest(BoidStatus& my_status, map<int, BoidStatus>& flock_status) {
  

  Col<double> diff = my_status.diff(this->destination);
  Col<double> suggested_linear = normalise(diff);

  Col<double> angsugg = zeros<Col<double> >(3);

  double weight = norm(diff);

  return IVelStrat::Suggestion(normalise(suggested_linear), angsugg, weight);
}
