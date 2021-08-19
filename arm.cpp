#include "arm.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "moveit/move_group_interface/move_group_interface.h"

using namespace std;

Arm::Arm() : Arm("arm") {}

Arm::Arm(string name) {
    this->speed = 20;
    this->accel = 10;
    this->armName = name;
    arm = new moveit::planning_interface::MoveGroupInterface(name);
    arm->setMaxAccelerationScalingFactor((double)accel / 100.);
}

Arm::~Arm() { delete arm; }

int Arm::move(std::vector<double> position, int feedRate, MoveType moveType,
              CtrlType ctrlType, CoordType coordType) {
    moveit::planning_interface::MoveItErrorCode err =
        moveit::planning_interface::MoveItErrorCode::SUCCESS;

    // deal with speed
    double speedFix = (double)this->speed * feedRate / 10000.;
    arm->setMaxVelocityScalingFactor(speedFix);

    if (moveType == MoveType::Absolute) {
    } else {
    }

    if (ctrlType == CtrlType::PTP) {
        err = arm->plan();
    } else {
        err = arm->computeCartesianPath();
    }

    err = arm->move();

    return moveit::planning_interface::MoveItErrorCode::SUCCESS ? 0 : -1;
}
