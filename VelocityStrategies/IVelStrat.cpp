#include "IVelStrat.h"
IVelStrat::Suggestion::Suggestion(Col<double> l, Col<double> a, double w) : lin(l), ang(a), weight(w) {
  
}

IVelStrat::IVelStrat(double i, double n): importance(i), neigh_thresh(n) {

}

bool IVelStrat::operator<(const IVelStrat& rhs) {
  return importance < rhs.importance;
}
