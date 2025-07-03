#include <iostream>
#include <cstdlib>
#include "rrlib.h"
#include "rr_parser.h"
#include "rrlib_hotel.h"


int main(int argc, char ** argv){
    RRLIB_HOTEL rrlib(argv, argc);
    rrlib.InteractiveRun();

    return 0;
}