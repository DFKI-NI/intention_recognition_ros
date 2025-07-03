/*
    RRLIB - RAGE-ROS Lib
    This was created as an interface between the RAGE planner and ROS-type environments.
    It facilitates sending/receiving information via messages. For example, send an action and receiving its
    corresponding observation and reward.

    The main steps consist of:
        1. Create instance by providing created POMDP, output file, execution and MCTS parameters.
        2. Plan for the next action using PlanAction. Returns action.
        3. When action is executed onboard robot and results perceived, run Update with action, observation, reward and indicate if a terminal state was reached
        4. Plan again until terminal condition is met.

    By:
    Juan Carlos Saborio
    DFKI-NI, 2023
*/

#ifndef RRLIB_H
#define RRLIB_H

#include "../rageplan/mcts.h"
#include "../rageplan/simulator.h"
#include "../rageplan/statistic.h"
#include <fstream>
#include <cstring>
#include <chrono>

#define TERMINAL 1
#define NON_TERMINAL 0
#define OUT_OF_PARTICLES -1

#define RANDOM_POLICY 1
#define PGS_POLICY 3

class RRLIB{    
    public:
        struct RUN_PARAMS{
            int simSteps = 1000;
            double timeout = 3600;
            double discount;            
            bool autoExploration = true;
            int simDoubles;
            int transformDoubles = -4;
            int transformAttempts = 1000;
            double accuracy = 0.01;
            int undiscountedHorizon = 1000;
        };

        struct RESULTS{
            double time;
            double discount;
            STATISTIC Reward;
            double undiscountedReturn;
            double discountedReturn;
            bool terminated;
        };

        RRLIB(const SIMULATOR& sim, const std::string& outputFile, RUN_PARAMS& RunParams, MCTS::PARAMS& SearchParams);
        
        void Init();
        int PlanAction(int numSims, double timeout); //Plan using mcts w/ POMDP. Return action.
        double Reward(int action, int observation); //Calculate reward from planned action and perceived observation.
        int Update(int action, int observation, double reward, bool terminal); //Update mcts/POMDP. Observation and reward come from previous steps
        RESULTS& getStatistics();
        void InteractiveRun();

    private:
        const SIMULATOR& POMDP;
        MCTS * mcts;
        RUN_PARAMS& runParams;
        MCTS::PARAMS& searchParams;
        RESULTS Results;
        std::ofstream OutputFile;
        STATE dummyState;

        std::chrono::_V2::steady_clock::time_point timer_start;
        std::chrono::duration<double> elapsed_seconds;
};

#endif
