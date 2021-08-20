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
    arm = new moveit::planning_interface::MoveGroupInterface(name);
    arm->setMaxVelocityScalingFactor((double)speed / 100.);
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

    if (coordType == CoordType::CARTESIAN) {
        geometry_msgs::Pose goal;

        if (moveType == MoveType::Relative) {
            goal = arm->getCurrentPose().pose;
        }
        goal.position.x += position[0];
        goal.position.y += position[1];
        goal.position.z += position[2];
        goal.orientation.x += position[3];
        goal.orientation.y += position[4];
        goal.orientation.z += position[5];

        arm->setPoseTarget(goal);

    } else {
        if (moveType == MoveType::Relative) {
            vector<double> current = arm->getCurrentJointValues();
            for (unsigned int i = 0; i < position.size(); i++) {
                position[i] += current[i];
            }
        }
        arm->setJointValueTarget(position);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    err = arm->plan(plan);
    if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return -1;
    }

    err = arm->move();

    if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return -1;
    }

    return 0;
}

void Arm::setSpeed(int speed) {
    this->speed = speed;
    arm->setMaxVelocityScalingFactor((double)speed / 100.);
}

int Arm::getSpeed() { return this->speed; }

void Arm::setAccel(int accel) {
    this->accel = accel;
    arm->setMaxAccelerationScalingFactor((double)accel / 100.);
}

int Arm::getAccel() { return this->accel; }

string Arm::getArmName() { return arm->getName(); }
