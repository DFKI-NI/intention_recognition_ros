/*
    RRLIB - Assembly
    RRLIB interface for the Manual Assembly problem in InCoRAP

    This class interacts directly with the assembly robot POMDP to perform domain specific functions.
    It uses RRLIB to perform planning and simulation but manipulates the underlying POMDP. Uses might include transforming 
    robot observations into POMDP-observations (e.g. triples).

    By:
    Juan Carlos Saborio
    DFKI-NI, 2023
*/

#ifndef RRLIB_ASSEMBLY_
#define RRLIB_ASSEMBLY_

#include <iostream>
#include <cstdlib>

//RAGEPlan interface
#include "rrlib.h"
#include "rr_parser.h"
//Domain classes
#include "../rageplan/assembly_robot.h"
#include "../rageplan/assembly_worker.h"

//ROS includes
#include <ros/ros.h>
//Perception actions
#include <intention_recognition_msgs/Intention.h>
//Manipulation actions
#include <actionlib/client/simple_action_client.h>
#include <intention_recognition_msgs/PlanAndExecuteAction.h>

class RRLIB_ASSEMBLY{
    private:
        ASSEMBLY_ROBOT * POMDP;
        RRLIB * rr;
        int n_objects;
        int n_parts;
        int n_types;
        STATE dummyState; //for POMDP functions that require a state param even though it is not used

        void GetObsFromMSG(intention_recognition_msgs::Intention srv_msg, int& observation, double& accuracy);
        int ParseMSGContent(string target, vector<string> options);

    public:
        RRLIB_ASSEMBLY(char ** argv, int argc); //Command-line constructor
        RRLIB_ASSEMBLY(string inputfile, string outputfile, int timeout, int simDoubles, int verbose, int rolloutPolicy, bool fTable); //Inline constructor
        void InteractiveRun();
        int ROSRun(int argc, char ** argv);

        int PlanAction(int nSims, double timeout);
        void DisplayAction(int action);
        void DisplayObservation(int observation);
        int Update(int action, int observation, double reward, bool terminal);
};

#endif