/*
    NOTE THAT I SWITCHED THE INCLUDE AND CONSTRUCTOR TO THE HOTEL SCENARIO
    WE SHOULD BE ABLE TO USE THIS AS A GENERAL PURPOSE LAUNCHER, PARSE THE PROBLEM NAME HERE AND SELECT WHICH CLASS TO BUILD
*/

#include "rrlib/rrlib_hotel.h"

#include <ros/ros.h>
#include <intention_recognition_msgs/Intention.h>

int main( int argc, char** argv )
{

    RRLIB_HOTEL rrlib(argv, argc);
    rrlib.ROSRun(argc, argv);

    return 0;
}
