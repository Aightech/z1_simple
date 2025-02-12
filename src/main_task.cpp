#include "z1_arm.hpp"

#include <stdint.h>
#include <iostream>
#include <joystick.h>

#define NB_ARMS 1

class Joystick : public cJoystick
{
public:
    int selectedArm = 0;
    int mode = 1;
    bool switchReleased = true;
    double speed = 0.04;
};

void connectArms(HandRobot *handRobots, int *udpPorts, int n = NB_ARMS)
{
    for (int i = 0; i < NB_ARMS; i++)
    {
        handRobots[i].name = (i == 0 ? "left" : "right");
        auto ctrlComp = new UNITREE_ARM::CtrlComponents(0.002,true);
        ctrlComp->dt = 0.002; // control period: 500Hz
        ctrlComp->udp = new UNITREE_ARM::UDPPort("127.0.0.1", udpPorts[i], udpPorts[i] + 1, UNITREE_ARM::RECVSTATE_LENGTH, UNITREE_ARM::BlockYN::NO, 500000);
        ctrlComp->armModel = new Z1custom((i == 0 ? false : true)); // no UnitreeGripper
        ctrlComp->armModel->addLoad(0.03);                          // add 0.03kg payload to the end joint
        handRobots[i].arm = new UNITREE_ARM::unitreeArm(ctrlComp);
        handRobots[i].arm->directions << 0, 0, 0, 0, 0, 0, 0;
        handRobots[i].arm->sendRecvThread->start();
        handRobots[i].arm->backToStart();
    }
    for (int i = 0; i < NB_ARMS; i++)
        handRobots[i].arm->labelRun("forward");
}



void joystick_mode(Joystick *js, HandRobot *handRobots)
{
    if (js->switchReleased)
    {
        if (abs(js->joystickValue(6)) > 0)
        {
            if (js->selectedArm == 1)
                js->mode = (js->mode + 1) % 2;

            js->selectedArm = (js->selectedArm + 1) % 2;
            std::cout << "Selected arm: " << js->selectedArm << std::endl;
            std::cout << "Mode: " << ((js->mode == 1) ? "translation" : "rotation") << std::endl;
            js->switchReleased = false;
        }
    }
    else if (abs(js->joystickValue(6)) == 0)
        js->switchReleased = true;

    double delta = js->joystickValue(7) / 32767. / 1000000;
    if (delta)
    {
        js->speed += -delta;
        js->speed = (js->speed < 0.001) ? 0.001 : ((js->speed > 0.06) ? 0.05 : js->speed);
        std::cout << "speed: " << js->speed << std::endl;
    }
    double angular_vel = js->speed;
    double linear_vel = js->speed;
    int jIndex[6] = {4, 1, 0, 1, 0, 4};
    for (int i = 0; i < 3; i++)
    {
        double val;
        val = js->joystickValue(jIndex[i + 3 * js->mode]) / 32767.;
        val = (val > 0.05) ? (val - 0.05) : ((val < -0.05) ? val + 0.05 : 0);
        handRobots[js->selectedArm].arm->directions[i + 3 * js->mode] = -val; // reverse direction
    }
    if (handRobots[js->selectedArm].period())
        handRobots[js->selectedArm].arm->cartesianCtrlCmd(handRobots[js->selectedArm].arm->directions, angular_vel, linear_vel);
}

void choree_mode(HandRobot *handRobot)
{
    char n = handRobot->name.c_str()[0];
    std::string name = std::string(1, n);
    handRobot->arm->teachRepeat(name);
}

void teaching_mode(HandRobot *handRobot)
{
    char n = handRobot->name.c_str()[0];
    std::string name = std::string(1, n);
    handRobot->arm->teach(name);
}

typedef enum _runningMode
{
    IDLE,
    PASSIVE,
    HOME,
    FORWARD,
    STOP,
    CHOREE,
    TEACHING,
    TRACKING,
    JOYSTICK
} runningMode;

int main()
{
    HandRobot handRobots[NB_ARMS]; // processing the user hands
    int udpPorts[2] = {8071,8073};
    connectArms(handRobots, udpPorts, NB_ARMS);

    int selectedArm = 0;

    Joystick js;
    try
    {
        js.connect("/dev/input/js0", "");
    }
    catch (std::string err)
    {
        std::cout << err << std::endl;
    }

    bool running = true;
    bool teaching = false;
    runningMode mode = IDLE;
    while (running)
    {
        if (js.buttonPressed(0))
        {
            std::cout << "PASSIVE mode" << std::endl;
        }
        if (js.buttonPressed(1))
        {
            std::cout << "HOME mode" << std::endl;
            mode = HOME;
        }
        if (js.buttonPressed(2))
        {
            std::cout << "FORWARD mode" << std::endl;
            mode = FORWARD;
        }
        if (js.buttonPressed(3))
        {
            if (mode != CHOREE)
                std::cout << "CHOREE mode" << std::endl;
            mode = CHOREE;
        }
        if (js.buttonPressed(4))
        {
            std::cout << "TRACKING mode" << std::endl;
            for (int i = 0; i < NB_ARMS; i++)
            {
                handRobots[i].arm->labelRun("forward");
                handRobots[i].homePos();
            }
            mode = TRACKING;
        }
        if (js.buttonPressed(5))
        {
            if (mode != JOYSTICK)
                std::cout << "JOYSTICK mode" << std::endl;
            mode = JOYSTICK;
        }
        if (js.buttonPressed(6))
        {
            std::cout << "STOP" << std::endl;
            mode = STOP;
        }
        if (js.switchReleased)
        {
            if (js.buttonPressed(8))
            {
                teaching = !teaching;
                if (teaching)
                    std::cout << "TEACHING" << std::endl;
                else
                    std::cout << "STOP TEACHING" << std::endl;
                js.switchReleased = false;
                mode = TEACHING;
            }
        }
        else if (teaching && !js.buttonPressed(8))
            js.switchReleased = true;

        switch (mode)
        {
        case IDLE:
            break;
        case PASSIVE:
            for (int i = 0; i < NB_ARMS; i++)
                handRobots[i].arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
            mode = IDLE;
            break;
        case FORWARD:
            for (int i = 0; i < NB_ARMS; i++)
                handRobots[i].arm->labelRun("forward");
            mode = IDLE;
            break;
        case HOME:
            for (int i = 0; i < NB_ARMS; i++)
                handRobots[i].arm->backToStart();
            mode = IDLE;
            break;
        case STOP:
            running = false;
            break;
        case CHOREE:
            std::thread *t[NB_ARMS];
            for (int i = 0; i < NB_ARMS; i++)
            {
                t[i] = new std::thread(choree_mode, &handRobots[i]);
            }
            for (int i = 0; i < NB_ARMS; i++)
            {
                t[i]->join();
                delete t[i];
            }
            mode = IDLE;
            std::cout << "Finished" << std::endl;

            break;
        case TEACHING:
            if (teaching)
            {
                for (int i = 0; i < NB_ARMS; i++)
                    teaching_mode(&handRobots[i]);
            }
            else
            {
                for (int i = 0; i < NB_ARMS; i++)
                    handRobots[i].arm->setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
            }
            mode = IDLE;
            break;
        case TRACKING:
            // tracking_mode(handRobots);
            break;
        case JOYSTICK:
            joystick_mode(&js, handRobots);
            break;
        }
    }

    for (int i = 0; i < NB_ARMS; i++)
    {
        handRobots[i].arm->backToStart();
        handRobots[i].arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        handRobots[i].arm->sendRecvThread->shutdown();
    }
    return 0;
}