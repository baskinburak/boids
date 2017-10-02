#include "BoidStatus.h"
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std;

BoidStatus::BoidStatus() : id(0), position(3), orientation(4), linear_velocity(3), angular_velocity(3) {
  position = zeros<Col<double> >(3);
  orientation = zeros<Col<double> >(4);
  orientation[3] = 1;

  linear_velocity = zeros<Col<double> >(3);
  angular_velocity = zeros<Col<double> >(3);
}


BoidStatus::BoidStatus(int i, Col<double>& p, Col<double>& o, Col<double>& l, Col<double>& a) : id(i), position(p), orientation(o), linear_velocity(l), angular_velocity(a) {
  
}

BoidStatus::BoidStatus(const boids::Status& stat) : position(3), orientation(4), linear_velocity(3), angular_velocity(3) {
  this->id = stat.uav_id;

  this->position[0] = stat.pose.position.x;
  this->position[1] = stat.pose.position.y;
  this->position[2] = stat.pose.position.z;

  this->orientation[0] = stat.pose.orientation.x;
  this->orientation[1] = stat.pose.orientation.y;
  this->orientation[2] = stat.pose.orientation.z;
  this->orientation[3] = stat.pose.orientation.w;

  this->linear_velocity[0] = stat.velocity.linear.x;
  this->linear_velocity[1] = stat.velocity.linear.y;
  this->linear_velocity[2] = stat.velocity.linear.z;

  angular_velocity[0] = stat.velocity.angular.x;
  angular_velocity[1] = stat.velocity.angular.y;
  angular_velocity[2] = stat.velocity.angular.z;
}

BoidStatus::BoidStatus(const BoidStatus& bstat) : id(bstat.id), position(bstat.position), orientation(bstat.orientation), linear_velocity(bstat.linear_velocity), angular_velocity(bstat.angular_velocity) {
  
}

BoidStatus& BoidStatus::operator=(BoidStatus& rhs) {
  if(&rhs != this) {
    this->id = rhs.id;
    this->position = rhs.position;
    this->orientation = rhs.orientation;
    this->linear_velocity = rhs.linear_velocity;
    this->angular_velocity = rhs.angular_velocity;
  }
  return *this;
}

void BoidStatus::repopulate(const boids::Status& stat) {
  this->id = stat.uav_id;

  this->position[0] = stat.pose.position.x;
  this->position[1] = stat.pose.position.y;
  this->position[2] = stat.pose.position.z;

  this->orientation[0] = stat.pose.orientation.x;
  this->orientation[1] = stat.pose.orientation.y;
  this->orientation[2] = stat.pose.orientation.z;
  this->orientation[3] = stat.pose.orientation.w;

  this->linear_velocity[0] = stat.velocity.linear.x;
  this->linear_velocity[1] = stat.velocity.linear.y;
  this->linear_velocity[2] = stat.velocity.linear.z;

  this->angular_velocity[0] = stat.velocity.angular.x;
  this->angular_velocity[1] = stat.velocity.angular.y;
  this->angular_velocity[2] = stat.velocity.angular.z;
}


void BoidStatus::repopulate_pose(const geometry_msgs::Pose& pose) {
  this->position[0] = pose.position.x;
  this->position[1] = pose.position.y;
  this->position[2] = pose.position.z;

  this->orientation[0] = pose.orientation.x;
  this->orientation[1] = pose.orientation.y;
  this->orientation[2] = pose.orientation.z;
  this->orientation[3] = pose.orientation.w;
}

void BoidStatus::update_velocity(Col<double>& lin, Col<double>& ang) {
  this->linear_velocity = lin;
  this->angular_velocity = ang;

  if(this->linear_velocity_norm() == 0)
    this->linear_velocity[0] = 0.000001;

  /*if(this->linear_velocity[0] != this->linear_velocity[0]) {
    this->linear_velocity[0] = this->linear_velocity[1] = 0;
    this->linear_velocity[2] = 0.000001;
  }*/
}

void BoidStatus::publish_status(ros::Publisher& pub, boids::Status& msg) {
  msg.uav_id = this->id;

  msg.pose.position.x = this->position[0];
  msg.pose.position.y = this->position[1];
  msg.pose.position.z = this->position[2];

  msg.pose.orientation.x = this->orientation[0];
  msg.pose.orientation.y = this->orientation[1];
  msg.pose.orientation.z = this->orientation[2];
  msg.pose.orientation.w = this->orientation[3];

  msg.velocity.linear.x = this->linear_velocity[0];
  msg.velocity.linear.y = this->linear_velocity[1];
  msg.velocity.linear.z = this->linear_velocity[2];

  msg.velocity.angular.x = this->angular_velocity[0];
  msg.velocity.angular.y = this->angular_velocity[1];
  msg.velocity.angular.z = this->angular_velocity[2];


  pub.publish(msg);
}

void BoidStatus::publish_status(ros::Publisher& pub) {
  boids::Status msg;
  this->publish_status(pub, msg);
}

void BoidStatus::publish_velocity(ros::Publisher& pub, geometry_msgs::Twist& msg) {

  
  tf2::Quaternion rot(this->orientation[0], this->orientation[1], this->orientation[2], this->orientation[3]);
  rot = rot.inverse();
  tf2::Vector3 vec(this->linear_velocity[0], this->linear_velocity[1], this->linear_velocity[2]);


  tf2::Vector3 res = tf2::quatRotate(rot, vec);


  msg.linear.x = res[0];
  msg.linear.y = res[1];
  msg.linear.z = res[2];


  msg.angular.x = this->angular_velocity[0];
  msg.angular.y = this->angular_velocity[1];
  msg.angular.z = this->angular_velocity[2];

  pub.publish(msg);
}

void BoidStatus::publish_velocity(ros::Publisher& pub) {
  geometry_msgs::Twist msg;
  this->publish_velocity(pub, msg);
}

int BoidStatus::get_id() const {
  return this->id;
}

void BoidStatus::set_id(int i) {
  this->id = i;
}

Col<double>& BoidStatus::get_position() {
  return this->position;
}

Col<double>& BoidStatus::get_lin_velocity() {
  return this->linear_velocity;
}

pair<double, double> BoidStatus::closest_distance(const BoidStatus& oth) {
  Col<double> pos_diff = oth.position - this->position;
  double dot_prdt;
  if((dot_prdt = dot(pos_diff, this->linear_velocity)) > 0) {
    double vel_norm = this->linear_velocity_norm();
    if(vel_norm > 0) {
      double proj = dot_prdt / vel_norm;
      return make_pair(norm(cross(pos_diff, this->linear_velocity)) / vel_norm, proj / vel_norm);
    }
  }

  return make_pair(numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
}

double BoidStatus::distance(const BoidStatus& oth) {
  return norm(oth.position - this->position);
}

Col<double> BoidStatus::diff(const Col<double>& rhs) {
  return rhs - this->position;
}

Col<double> BoidStatus::distance_vector(const BoidStatus& stat) {
  return stat.position - this->position;
}

Col<double> BoidStatus::escape(const BoidStatus& stat) {
  Col<double> pos_diff = stat.position - this->position;
  return normalise(cross(pos_diff, cross(this->linear_velocity, pos_diff)));
}

double BoidStatus::linear_velocity_norm() {
  return norm(this->linear_velocity);
}
