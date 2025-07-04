/*
    RRLIB - Insect Hotel Assembly
    RRLIB interface for the Insect Hotel problem based on Manual Assembly/InCoRAP

    This class interacts directly with the assembly robot POMDP to perform domain specific functions.
    It uses RRLIB to perform planning and simulation but manipulates the underlying POMDP. Uses might include transforming 
    robot observations into POMDP-observations.

    By:
    Juan Carlos Saborio
    DFKI-NI, 2023
*/

#ifndef RRLIB_HOTEL_
#define RRLIB_HOTEL_

#include <iostream>
#include <cstdlib>

//RAGEPlan interface
#include "rrlib.h"
#include "rr_parser.h"
//Domain classes
#include "../rageplan/hotel_robot.h"
#include "../rageplan/hotel_worker.h"

//ROS includes
#include <ros/ros.h>
//Perception actions
#include <intention_recognition_ros/Intention.h>
//Manipulation actions
#include <actionlib/client/simple_action_client.h>
#include <tables_demo_planning/PlanAndExecuteTasksAction.h>
#include <tables_demo_planning/Task.h>

class RRLIB_HOTEL{
    private:
        bool use_mockup_GUI = false;
        HOTEL_ROBOT * POMDP;
        RRLIB * rr;
        int n_objects;
        int n_parts;
        int n_types;
        STATE dummyState; //for POMDP functions that require a state param even though it is not used

        void GetObsFromMSG(intention_recognition_ros::Intention srv_msg, int& observation, double& accuracy, bool& terminal);
        int ParseMSGContent(string target, vector<string> options);

    public:
        RRLIB_HOTEL(char ** argv, int argc); //Command-line constructor
        RRLIB_HOTEL(string inputfile, string outputfile, int timeout, int simDoubles, int verbose, int rolloutPolicy, bool fTable); //Inline constructor
        void InteractiveRun();
        int ROSRun(int argc, char ** argv);

        int PlanAction(int nSims, double timeout);
        void DisplayAction(int action);
        void DisplayObservation(int observation, int action);
        int Update(int action, int observation, double reward, bool terminal);

        // klt_1 = red_part
        // klt_2 = black_part
        // klt_3 = yellow_part
        // klt_4 = bright_green_part
        // klt_5 = dark_green_part
        // klt_6 = purple_part
        // klt_7 = orange_part
        // klt_8 = magenta_part
        vector<string> partsID2partStr = {"bright_green_part_1", "dark_green_part_1","magenta_part_1", "purple_part_1", "red_part_1", "yellow_part_1", "black_part_1", "orange_part_1"};
};

#endif
