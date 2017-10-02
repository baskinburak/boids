#ifndef BOIDS_BOIDSTATUS_H
#define BOIDS_BOIDSTATUS_H
#include <armadillo>
#include <ros/ros.h>
#include <boids/Status.h>
#include <geometry_msgs/Pose.h>
#include <utility>
using namespace arma;
using namespace std;

class BoidStatus { // world frame
  private:
    int id;
    Col<double> position;
    Col<double> orientation;
    Col<double> linear_velocity;
    Col<double> angular_velocity;
  public:
    BoidStatus();
    BoidStatus(int i, Col<double>& p, Col<double>& o, Col<double>& l, Col<double>& a);
    BoidStatus(const boids::Status& stat);
    BoidStatus(const BoidStatus& bstat);
    BoidStatus& operator=(BoidStatus& rhs);

    void repopulate(const boids::Status& stat);
    void repopulate_pose(const geometry_msgs::Pose& pose);

    void update_velocity(Col<double>& lin, Col<double>& ang);

    void publish_status(ros::Publisher& pub, boids::Status& msg); // fills the msg and publishes
    void publish_status(ros::Publisher& pub); // generates a new boids::Status object and publishes

    void publish_velocity(ros::Publisher& pub, geometry_msgs::Twist& msg); // fills the msg and publishes
    void publish_velocity(ros::Publisher& pub); // generates a new geometry_msgs::Twist object and publishes

    int get_id() const;
    void set_id(int i);
    Col<double>& get_position();
    Col<double>& get_lin_velocity();

    double speed_norm();
    pair<double, double> closest_distance(const BoidStatus& stat); //calculates the closest anticipated distance and the time it will occur
    double distance(const BoidStatus& stat); // calculates the distance between two positions
    Col<double> diff(const Col<double>& rhs); // calculates the position's difference to rhs
    Col<double> distance_vector(const BoidStatus& stat); // calculates stat.position - this->position
    Col<double> escape(const BoidStatus& stat); // calculates the velocity vector to escpae from stat
    double linear_velocity_norm();
};


#endif
