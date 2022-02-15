#include "arm.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/tf.h>

#include <cmath>

#include "ros/node_handle.h"

using namespace std;

vector<double> euler2Quat(double rx, double ry, double rz);
vector<double> quat2Euler(double rx, double ry, double rz, double rw);
double degree2Rad(double degree);
double rad2Degree(double rad);

Arm::Arm() : Arm("manipulator") {}

Arm::Arm(string name) {
    this->speed = 5;
    this->accel = 5;

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
        std::vector<geometry_msgs::Pose> waypoints;

        position[3] = degree2Rad(position[3]);
        position[4] = degree2Rad(position[4]);
        position[5] = degree2Rad(position[5]);

        if (moveType == MoveType::Relative) {
            goal = arm->getCurrentPose().pose;
            vector<double> rpy =
                quat2Euler(goal.orientation.x, goal.orientation.y,
                           goal.orientation.z, goal.orientation.w);
            position[3] += rpy[0];
            position[4] += rpy[1];
            position[5] += rpy[2];
        }

        vector<double> quaternion =
            euler2Quat(position[3], position[4], position[5]);

        goal.position.x += position[0];
        goal.position.y += position[1];
        goal.position.z += position[2];
        goal.orientation.x = quaternion[0];
        goal.orientation.y = quaternion[1];
        goal.orientation.z = quaternion[2];
        goal.orientation.w = quaternion[3];
        waypoints.push_back(goal);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        arm->computeCartesianPath(waypoints, eef_step, jump_threshold,
                                  trajectory);

        err = arm->execute(trajectory);
        if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return -1;
        }

    } else {
        for (double& j : position) {
            j = degree2Rad(j);
        }

        if (moveType == MoveType::Relative) {
            vector<double> current = arm->getCurrentJointValues();
            for (unsigned int i = 0; i < position.size(); i++) {
                position[i] += current[i];
            }
        }
        arm->setJointValueTarget(position);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        err = arm->plan(plan);
        if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return -1;
        }

        err = arm->move();
    }

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

vector<double> Arm::getJointPosition() {
    vector<double> result = arm->getCurrentJointValues();

    for (double& j : result) {
        j = rad2Degree(j);
    }

    return result;
}

vector<double> Arm::getCartesianPosition() {
    geometry_msgs::Pose tmp = arm->getCurrentPose().pose;
    vector<double> rpy = quat2Euler(tmp.orientation.x, tmp.orientation.y,
                                    tmp.orientation.z, tmp.orientation.w);

    return {tmp.position.x,     tmp.position.y,     tmp.position.z,
            rad2Degree(rpy[0]), rad2Degree(rpy[1]), rad2Degree(rpy[2])};
}

vector<double> euler2Quat(double rx, double ry, double rz) {
    tf2::Quaternion quaternion;
    quaternion.setRPY(rx, ry, rz);
    quaternion = quaternion.normalize();

    return {quaternion.getX(), quaternion.getY(), quaternion.getZ(),
            quaternion.getW()};
}

vector<double> quat2Euler(double rx, double ry, double rz, double rw) {
    tf::Quaternion quaternion(rx, ry, rz, rw);
    tf::Matrix3x3 rpy(quaternion);
    vector<double> result(3, .0);
    rpy.getRPY(result[0], result[1], result[2]);

    return result;
}

double degree2Rad(double degree) { return degree * M_PI / 180; }
double rad2Degree(double rad) { return rad * 180 / M_PI; };
