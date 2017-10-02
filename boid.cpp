#include <ros/ros.h>
#include <armadillo>
#include <iostream>
#include <algorithm>
#include <boids/Status.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "BoidStatus.h"

#include "VelocityStrategies/IVelStrat.h"
#include "VelocityStrategies/CollisionAvoidance.h"
#include "VelocityStrategies/FlockCentering.h"
#include "VelocityStrategies/VelocityMatching.h"
#include "VelocityStrategies/Go.h"

using namespace std;
using namespace arma;

BoidStatus my_status;
map<int, BoidStatus> flock_status;


void status_update(const boids::Status& stat) {
  cout << my_status.get_id() << " " << stat.uav_id << endl;
  if(stat.uav_id == my_status.get_id()) return;
  flock_status[stat.uav_id].repopulate(stat);
}

void pose_update(const geometry_msgs::PoseStamped& pose) {
  my_status.repopulate_pose(pose.pose);
}



int main(int argc, char** argv) {

  vector<IVelStrat*> strategies;

  CollisionAvoidance col_avd(5, 7);
  strategies.push_back(&col_avd);

  FlockCentering fl_cent(1, 10);
  strategies.push_back(&fl_cent);

  VelocityMatching vel_mat(2, 10);
  strategies.push_back(&vel_mat);

  Col<double> dest(3);

  dest[0] = 1;
  dest[1] = 2;
  dest[2] = 3;

  Go go(3, 0, dest);
  strategies.push_back(&go);

  sort(strategies.rbegin(), strategies.rend());


  ros::init(argc, argv, "boid");
  ros::NodeHandle nh;

  int my_id;

  nh.getParam("boid/id", my_id);

  my_status.set_id(my_id);

  ros::Subscriber status_sub = nh.subscribe("/boids_status", 1000, &status_update);
  ros::Subscriber pose_sub = nh.subscribe("ground_truth_to_tf/pose", 1, &pose_update);

  ros::Publisher stat_pub = nh.advertise<boids::Status>("/boids_status", 1);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate rate(10);

  while(ros::ok()) {
    // calculate velocity    

    Col<double> lin_vel = zeros<Col<double> >(3);
    Col<double> ang_vel = zeros<Col<double> >(3);

    // sum all suggestions by weight and importance
    for(int i=0; i<strategies.size(); i++) {
      IVelStrat::Suggestion sugg = strategies[i]->suggest(my_status, flock_status);
      cout << strategies[i]->importance  << " " << sugg.weight  << " " <<  sugg.lin << endl;
      lin_vel += strategies[i]->importance * sugg.weight * sugg.lin;
      ang_vel += strategies[i]->importance * sugg.weight * sugg.ang;
    }


    lin_vel = normalise(lin_vel);


    // update velocity
    my_status.update_velocity(lin_vel, ang_vel);

    // publish velocity
    my_status.publish_velocity(vel_pub);
    my_status.publish_status(stat_pub);
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
