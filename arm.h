#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

#include <iostream>
#include <vector>

#include "ros/node_handle.h"
#include "ros/subscriber.h"

class Arm {
   public:
    enum CoordType { CARTESIAN = 0, JOINT = 1 };

    enum CtrlType { PTP = 0, LIN = 1 };

    enum MoveType { Absolute = 0, Relative = 1 };

    Arm(ros::NodeHandle& nh);
    Arm(std::string name, ros::NodeHandle& nh);
    ~Arm();
    int move(std::vector<double> position, int feedRate = 100,
             MoveType moveType = Absolute, CtrlType ctrlType = PTP,
             CoordType coordType = CARTESIAN);

    void setSpeed(int speed);
    void setAccel(int accel);

    int getSpeed();
    int getAccel();

    std::string getArmName();

    std::vector<double> getCartesianPosition();
    std::vector<double> getJointPosition();

   private:
    moveit::planning_interface::MoveGroupInterface* arm;

    bool isMoving();
    void waitForIdle();

    ros::Subscriber feedBackSubscriber;

    int speed;
    int accel;
};
