/*  This program is the main program for the arm control.
    It is used to control the arm in different modes:
    - passive
    - back to start
    - joint space control
    - cartesian space control
    - moveJ
    - moveL
    - moveC
    - low level command
    - save state
    - teach
    - teach repeat
    - to state
    - trajectory
    - calibration
    The program is designed to be real-time and can be controlled by keyboard or SDK.
    Usage:
    - Keyboard:
        - 1: passive (All motors get into passive state(default state after power on))
        - 2: joint space control (control each joint speed)
        - 3: cartesian space control (control the end-effector speed)
        - 4: moveJ (move to a position in joint space)
        - 5: moveL (move to a position in joint space with a straight line trajectory)
        - 6: moveC (move to a position in joint space with a circular trajectory)
        - 7: teach (record the current position)
        - 8: teach repeat (repeat the recorded position)
        - 9: save state
        - 0: to state
        - -: trajectory
        - =: calibration
        - `: back to start
        - q: joint 1 +
        - a: joint 1 -
        - w: joint 2 +
        - s: joint 2 -
        - e: joint 3 +
        - d: joint 3 -
        - r: joint 4 +
        - f: joint 4 -
        - t: joint 5 +
        - g: joint 5 -
        - y: joint 6 +
        - h: joint 6 -
        - up: joint 7 +
        - down: joint 7 -
*/

#include <stdint.h>
#include <csignal>
#include <sched.h>
#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Passive.h"
#include "FSM/State_BackToStart.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_Cartesian.h"
#include "FSM/State_JointSpace.h"
#include "FSM/State_MoveJ.h"
#include "FSM/State_MoveL.h"
#include "FSM/State_MoveC.h"
#include "FSM/State_ToState.h"
#include "FSM/State_SaveState.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_Trajectory.h"
#include "FSM/State_LowCmd.h"

bool running = true;

// set real-time program
void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        // std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(5);

    EmptyAction emptyAction((int)ArmFSMStateName::INVALID);
    std::vector<KeyAction *> events;

    CtrlComponents *ctrlComp = new CtrlComponents();
    ctrlComp->ctrl = Control::SDK;
    ctrlComp->ctrl_port = 8881;
    ctrlComp->ctrl_IP = "192.168.123.110";
    int port = 8071;
    std::cout << "Usage: " << std::endl;
    std::cout << "l: left arm" << std::endl;
    std::cout << "r: right arm" << std::endl;
    std::cout << "k: keyboard control" << std::endl;
    
    if (argc > 1)
    {
        for (int i = 1; i < argc; i++)
        {
            std::cout << "argv[" << i << "]: " << argv[i] << std::endl;

            if (argv[i][0] == 'l')
            {
                std::cout << "Left arm" << std::endl;
            }
            else if (argv[i][0] == 'r')
            {
                std::cout << "Right arm" << std::endl;
                ctrlComp->ctrl_IP = "192.168.123.111";
                ctrlComp->ctrl_port = 8882;
                port = 8073;
            }
            else if (argv[i][0] == 'k')
            {
                ctrlComp->ctrl = Control::KEYBOARD;
                std::cout << "Keyboard control" << std::endl;
            }
            else
            {
                std::cout << "Invalid argument" << std::endl;
                return 0;
            }
        }
    }

    ctrlComp->dt = 1.0 / 250.;
    ctrlComp->armConfigPath = "../config/";
    ctrlComp->stateCSV = new CSVTool("../config/savedArmStates.csv");
    ctrlComp->ioInter = new IOUDP(ctrlComp->ctrl_IP.c_str(), ctrlComp->ctrl_port);
    ctrlComp->geneObj();
    std::cout << "IP: " << ctrlComp->ctrl_IP << std::endl;
    std::cout << "Port: " << ctrlComp->ctrl_port << std::endl;
    std::cout << "sdk port: " << port + 1 << std::endl;
    std::cout << "own port: " << port << std::endl;
    if (ctrlComp->ctrl == Control::SDK)
    {
        ctrlComp->cmdPanel = new ARMSDK(events, emptyAction, "127.0.0.1", port + 1, port, 0.002);
        std::cout << "SDK control" << std::endl;
    }
    else if (ctrlComp->ctrl == Control::KEYBOARD)
    {
        std::cout << "Keyboard control" << std::endl;
        events.push_back(new StateAction("`", (int)ArmFSMStateName::BACKTOSTART));
        events.push_back(new StateAction("1", (int)ArmFSMStateName::PASSIVE));
        events.push_back(new StateAction("2", (int)ArmFSMStateName::JOINTCTRL));
        events.push_back(new StateAction("3", (int)ArmFSMStateName::CARTESIAN));
        events.push_back(new StateAction("4", (int)ArmFSMStateName::MOVEJ));
        events.push_back(new StateAction("5", (int)ArmFSMStateName::MOVEL));
        events.push_back(new StateAction("6", (int)ArmFSMStateName::MOVEC));
        events.push_back(new StateAction("7", (int)ArmFSMStateName::TEACH));
        events.push_back(new StateAction("8", (int)ArmFSMStateName::TEACHREPEAT));
        events.push_back(new StateAction("9", (int)ArmFSMStateName::SAVESTATE));
        events.push_back(new StateAction("0", (int)ArmFSMStateName::TOSTATE));
        events.push_back(new StateAction("-", (int)ArmFSMStateName::TRAJECTORY));
        events.push_back(new StateAction("=", (int)ArmFSMStateName::CALIBRATION));

        events.push_back(new ValueAction("q", "a", 0.5));
        events.push_back(new ValueAction("w", "s", 0.5));
        events.push_back(new ValueAction("e", "d", 0.5));
        events.push_back(new ValueAction("r", "f", 0.5));
        events.push_back(new ValueAction("t", "g", 0.5));
        events.push_back(new ValueAction("y", "h", 0.5));
        events.push_back(new ValueAction("down", "up", 1.));

        ctrlComp->cmdPanel = new Keyboard(events, emptyAction);
    }
    std::vector<FSMState *> states;
    states.push_back(new State_Passive(ctrlComp));
    states.push_back(new State_BackToStart(ctrlComp));
    states.push_back(new State_JointSpace(ctrlComp));
    states.push_back(new State_Cartesian(ctrlComp));
    states.push_back(new State_MoveJ(ctrlComp));
    states.push_back(new State_MoveL(ctrlComp));
    states.push_back(new State_MoveC(ctrlComp));
    states.push_back(new State_LowCmd(ctrlComp));
    states.push_back(new State_SaveState(ctrlComp));
    states.push_back(new State_Teach(ctrlComp));
    states.push_back(new State_TeachRepeat(ctrlComp));
    states.push_back(new State_ToState(ctrlComp));
    states.push_back(new State_Trajectory(ctrlComp));
    states.push_back(new State_Calibration(ctrlComp));

    FiniteStateMachine *fsm;
    fsm = new FiniteStateMachine(states, ctrlComp);

    ctrlComp->running = &running;
    signal(SIGINT, [](int signum)
           { running = false; });
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ctrlComp->cmdPanel->start();
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    delete fsm;
    delete ctrlComp;
    return 0;
}