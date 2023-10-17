#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <vector>

using namespace std;

#define OWN_ROBOT 1
#define ENEMY_ROBOT 0

struct fVec3
{
    float x;
    float y;
    float th;

    fVec3(float _x, float _y, float _th) : x(_x), y(_y), th(_th) {}
};

struct iVec3
{
    int x;
    int y;
    int th;

    iVec3(int _x, int _y, int _th) : x(_x), y(_y), th(_th) {}
};

/**
 * @param pose: The pose of the robot
 * @param vel: The velocity of the robot
 * @param type: The type of the robot, is it enemy or own
 */
typedef struct Robot
{
    fVec3 pose;
    fVec3 vel;
    uint8_t type;

    Robot(fVec3 pose, fVec3 vel, uint8_t type) : pose(pose), vel(vel), type(type) {}
    Robot(float pose_x, float pose_y, float pose_th, float vel_x, float vel_y, float vel_th, uint8_t type) : pose(pose_x, pose_y, pose_th), vel(vel_x, vel_y, vel_th), type(type) {}

    void InitRobot(fVec3 pose, fVec3 vel, uint8_t type)
    {
        this->pose = pose;
        this->vel = vel;
        this->type = type;
    }

    void InitRobot(float pose_x, float pose_y, float pose_th, float vel_x, float vel_y, float vel_th, uint8_t type)
    {
        this->pose.x = pose_x;
        this->pose.y = pose_y;
        this->pose.th = pose_th;
        this->vel.x = vel_x;
        this->vel.y = vel_y;
        this->vel.th = vel_th;
        this->type = type;
    }

    void GiveVelocity(fVec3 vel)
    {
        this->vel = vel;
    }

    void GiveXVelocity(float vel)
    {
        this->vel.x = vel;
    }

    void GiveYVelocity(float vel)
    {
        this->vel.y = vel;
    }

    void GiveThetaVelocity(float vel)
    {
        this->vel.th = vel;
    }

    void GiveVelocity(float vel_x, float vel_y, float vel_th)
    {
        GiveXVelocity(vel_x);
        GiveYVelocity(vel_y);
        GiveThetaVelocity(vel_th);
    }
} RobotType;

#endif