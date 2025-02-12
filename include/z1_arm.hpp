#ifndef __Z1_ARM_HPP__
#define __Z1_ARM_HPP__

#include <stdint.h>
#include "unitree_arm_sdk/control/unitreeArm.h"
#include <ctime>

class Z1custom : public UNITREE_ARM::Z1Model
{
public:
    Z1custom(bool isRight) : UNITREE_ARM::Z1Model()
    {
        if (isRight)
        {
            _jointQMax[0] = 0.06;
            _jointQMin[0] = -0.5;

            //_jointQMin[4] = 0;
            //_jointQMin[4] = -.5;
        }
        else
        {
            _jointQMax[0] = 0.5;
            _jointQMin[0] = -0.06;
        }
    }
    ~Z1custom()
    {
    }
    void setJointQMax(int index, float value)
    {
        _jointQMax[index] = value;
    }
    void setJointQMin(int index, float value)
    {
        _jointQMin[index] = value;
    }
};

struct HandRobot
{
public:
    Vec6 origin = {0, 0, 0, 0, 0, 0};                                                  // origin of the hand movement
    Vec6 current = {0, 0, 0, 0, 0, 0};                                                 // current position of the hand
    Vec6 position = {-0.0190173, 1.45547, -0.020939, 0.213274, -0.00024022, 0.218801}; // position of the robot
    Vec6 current_q;                                                                    // joint angles of the robot
    Vec3 robotBase = {0, 0, 0};                                                        // position of the robot base
    bool isGrabbing = false;
    bool justGrabbed = false;
    float grabStrength_threshold = 0.8;
    float openGripper = 0.0;
    std::string name = "HandRobot";
    UNITREE_ARM::unitreeArm *arm;
    clock_t last_t = 0;

    bool period()
    {
        // std::cout << ((double)clock() - last_t) /CLOCKS_PER_SEC  << " " << arm->_ctrlComp->dt << std::endl;
        if (((double)clock() - last_t) / CLOCKS_PER_SEC < arm->_ctrlComp->dt / 2)
            return false;
        else
        {
            last_t = clock();
            return true;
        }
    }

    void homePos()
    {
        position = {-0.0190173, 1.45547, -0.020939, 0.213274, -0.00024022, 0.218801};
        current = {0, 0, 0, 0, 0, 0};
        origin = {0, 0, 0, 0, 0, 0};
    }


    void updatePosition()
    {
        if (justGrabbed)
            for (int i = 0; i < 6; i++)
            {
                origin[i] = current[i] - position[i]; // update the origin of the hand movement
            }
        if (isGrabbing)
        {
            // Vec6 newpos = {current[0] - origin[0], current[1] - origin[1], current[2] - origin[2], current[3] - origin[3], current[4] - origin[4], current[5] - origin[5]};

            for (int i = 0; i < 6; i++)
                position[i] = current[i] - origin[i]; // update the position of the robot
        }
    };
};

#endif /* __Z1_ARM_HPP__ */
