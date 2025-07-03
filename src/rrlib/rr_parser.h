#ifndef RR_PARSER_H
#define RR_PARSER_H

#include <fstream>
#include <iostream>
#include <iomanip>
#include "../rageplan/simulator.h"
#include "../rageplan/assembly_robot.h"
#include "../rageplan/assembly_worker.h"

#include "../rageplan/hotel_robot.h"
#include "../rageplan/hotel_worker.h"

#include "rapidjson/document.h"

using std::cout;
using std::endl;
using std::string;
using rapidjson::Document;
using rapidjson::Value;

namespace RR_PARSER{
    struct COMMAND_LINE{
        string help;
        string problem = "none";
        string inputFile = "none";
        string outputFile = "output.txt";
        int simDoubles;
        int timeout = 1000;        
        int verbose = 0;
        int treeKnowledge = 1;
        int rolloutKnowledge = 1;
        bool fTable = 0;
    };

    void parseCommandLine(char ** argv, int argc, COMMAND_LINE& cl);
    bool parseCommandLine(char ** argv, int argc, string& paramFile, string& problemFile, string& domainFile, bool& use_mockup_GUI);
    bool parseParamsFile(string inputFile, COMMAND_LINE& cl);
    
    //InCoRAP manual assembly
    bool parseAssemblyFile(ASSEMBLY_ROBOT_PARAMS& problem_params, ASSEMBLY_PARAMS& worker_params, string& domainFilename, string inputFile);
    bool ParseManualAssembly(string problemFile, ASSEMBLY_PARAMS& ap);

    //AI Demo Insect Hotel
    bool parseHotelFile(HOTEL_ROBOT_PARAMS& problem_params, HOTEL_PARAMS& worker_params, string& domainFilename, string inputFile);
    bool ParseHotel(string problemFile, HOTEL_PARAMS& ap);
}

#endif
