#include "geometry_msgs/twist.h"

#ifndef ODOMETRY_H
#define ODOMETRY_H

/* SOURCE FILE NOTES */
//1 count of an encoder is roughly equal to 1/3 of a degree.

const double wheelDiameter = 0.3302; //In meters.
const double robotWidth = 0.6731;  //In meters.
const int gearRatio = 20; //20:1 ratio, must divide encoder count by this.

struct RobotPosition()
{
  double xPosition; //In meters.
  double yPosition; //In meters.
  double totalArcDistance; //In meters.
  double angularPosition;  //In radians.
}

struct RobotVelocity()
{
  double linearVelocity; //In meters per second.
  double tangentalVelocity; //In meters per second. (Depends on angularVelocity and wheelRadius)
}

class Odometry
{
  private:
    RobotPosition position;
    RobotVelocity velocity;

  public:
    /* Setters */
    void updatePosition();	
    void updateVelocity();

    /* Getters */
    int getEncoderLeftCount();
    int getEncoderRightCount();
    RobotPosition getPosition();
    std::unique_ptr<geometry_msgs::Twist::ConstPtr> getVelocity();

};

#endif
