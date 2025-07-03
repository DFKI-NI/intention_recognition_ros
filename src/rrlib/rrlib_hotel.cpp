#include "rrlib_hotel.h"

RRLIB_HOTEL::RRLIB_HOTEL(string inputfile, string outputfile, int timeout, int simDoubles, int verbose, int rolloutPolicy, bool fTable){
    MCTS::PARAMS searchParams;
    RRLIB::RUN_PARAMS runParams;
    SIMULATOR::KNOWLEDGE knowledge;

    runParams.timeout = timeout;
    runParams.simDoubles = simDoubles;

    searchParams.Verbose = verbose;
    searchParams.useFtable = fTable;

    knowledge.TreeLevel = 1;
    knowledge.RolloutLevel = rolloutPolicy;

    HOTEL_PARAMS worker_params;
    HOTEL_ROBOT_PARAMS problem_params;
    string domainFile;

    if(!RR_PARSER::parseHotelFile(problem_params, worker_params, domainFile, inputfile)){
        std::cerr << "Error parsing file." << endl;
        exit(0);
    }
    //worker_params.HOTEL_AIDEMO(); //Initialize WORKER MWE instead of parsing input for now

    n_parts = worker_params.n_parts;

    POMDP = new HOTEL_ROBOT(problem_params, worker_params);
    POMDP->SetKnowledge(knowledge);

    //Display problem information/setup
    cout << "Running: " << problem_params.description << endl;
    cout << "No. of sims: " << (1 << runParams.simDoubles) << endl;
    cout << "Policy: ";
    if(knowledge.RolloutLevel == 1) cout << "Random";
    else if(knowledge.RolloutLevel >= 3) cout << "PGS";
    cout << ", IRE: " << (searchParams.useFtable)? "Y" : "N";
    cout << endl;

    rr = new RRLIB(*POMDP, "output.txt", runParams, searchParams);
}

/*
	Use this constructor for the demo.
	The paramFile lists all necessary parameters to construct the MCTS search class
	The paramFile also has the problemFile with the params to build the POMDP
*/
RRLIB_HOTEL::RRLIB_HOTEL(char ** argv, int argc){
    MCTS::PARAMS searchParams;
    RRLIB::RUN_PARAMS runParams;
    SIMULATOR::KNOWLEDGE knowledge;
    RR_PARSER::COMMAND_LINE cl;

    string problem, paramFile, problemFile, domainFile, outputfile, policy;

     if(!RR_PARSER::parseCommandLine(argv, argc, paramFile, problemFile, domainFile, use_mockup_GUI)){
        std::cerr << "Error parsing command line parameters." << std::endl;
        exit(0);
    }

    //RR_PARSER::parseCommandLine(argv, argc, cl);
    if(!RR_PARSER::parseParamsFile(paramFile, cl)){
        std::cerr << "Error parsing search parameters file." << std::endl;
        exit(0);
    }

    problem = cl.problem;
    //inputfile = cl.inputFile;
    outputfile = cl.outputFile;

    runParams.timeout = cl.timeout;
    runParams.simDoubles = cl.simDoubles;

    searchParams.Verbose = cl.verbose;
    searchParams.useFtable = cl.fTable;

    knowledge.TreeLevel = cl.treeKnowledge;
    knowledge.RolloutLevel = cl.rolloutKnowledge;

    HOTEL_PARAMS worker_params;
    HOTEL_ROBOT_PARAMS problem_params;
    string domainFilename;

    //Read problem parameters
    if(!RR_PARSER::parseHotelFile(problem_params, worker_params, domainFilename, problemFile)){
        std::cerr << "Error parsing insect hotel file." << endl;
        exit(0);
    }

    //Read Hotel problem description from JSON file

    if(domainFile == ""){
        cout << "Using default Insect Hotel domain" << endl;
        worker_params.HOTEL_AIDEMO(); //Initialize
    }
    else if(!RR_PARSER::ParseHotel(domainFile, worker_params)){
        std::cerr << "Error parsing Insect Hotel domain description file." << endl;
        exit(0);
    }

    n_parts = worker_params.n_parts;

    //TODO: we could maintain a Simulator * POMDP ptr instead and initialize accordingly depending on the problem
    POMDP = new HOTEL_ROBOT(problem_params, worker_params);
    POMDP->SetKnowledge(knowledge);

    //Display problem information/setup
    cout << "Running: " << problem_params.description << endl;
    cout << "No. of sims: " << (1 << runParams.simDoubles) << endl;
    cout << "Policy: ";
    if(knowledge.RolloutLevel == 1) cout << "Random";
    else if(knowledge.RolloutLevel >= 3) cout << "PGS";
    cout << ", IRE: " << (searchParams.useFtable)? "Y" : "N";
    cout << endl;

    rr = new RRLIB(*POMDP, "output.txt", runParams, searchParams);
}

int RRLIB_HOTEL::PlanAction(int nSims, double timeout){
    return rr->PlanAction(nSims, timeout);
}

void RRLIB_HOTEL::DisplayAction(int action){
    std::stringbuf str;
    std::ostream out(&str);
    //Display action on ostream
    POMDP->DisplayAction(action, out);
    //Get char* from stringbuf and send to ros_info
    ROS_INFO("\033[32m%s\033[0m", str.str().c_str());
}

void RRLIB_HOTEL::DisplayObservation(int observation, int action){
    std::stringbuf str;
    std::ostream out(&str);

    //display obs on ostream
    if(action == POMDP->A_PERCEIVE || action == POMDP->A_INSPECT_CONTAINER){
        //Show bool array only for perceive
        POMDP->DisplayObservation(dummyState, observation, out);
    }
    else if(action == POMDP->A_INSPECT_OBJECT){
        out << "\033[32mType " << observation << "\033[0m" << std::endl;
    }
    /*else if(action >= POMDP->A_INSPECT_CONTAINER && action < POMDP->A_BRING_GLUE){
        out << (observation ? "\033[32mNot empty" : "\033[31mEmpty");
        out << "\033[0m" << std::endl;
    }*/
    else
        out << observation << std::endl;

    //Get char* from stringbuf and send to ros_info
    ROS_INFO("%s", str.str().c_str());
}

int RRLIB_HOTEL::Update(int action, int observation, double reward, bool terminal){
    return rr->Update(action, observation, reward, terminal);
}

int RRLIB_HOTEL::ROSRun(int argc, char ** argv){
    // Internal constants for ROS msg types
    int ROS_MSG_PERCEIVE_WORKER = 1;
    int ROS_MSG_INSPECT_CONTAINER = 2;
    int ROS_MSG_INSPECT_OBJECT = 3;
    int UNKNOWN_ACTION = -1;
    int PERCEPTION_ACTION = 0;
    int MANIPULATION_ACTION = 1;

/*
    INITIALIZE PERCEPTION CLIENT
*/
    ros::init(argc, argv, "intention_recognition_node");
    ros::NodeHandle nh;

    if(use_mockup_GUI)
        ROS_INFO("Using Mockup GUI");
    else
        ROS_INFO("NOT using Mockup GUI");
    ros::ServiceClient perceptionClient = nh.serviceClient<intention_recognition_msgs::Intention>("intention_perception");

    bool serviceAvailable = ros::service::waitForService("intention_perception", ros::Duration(5.0));

    if(!serviceAvailable){
        ROS_ERROR("Perception service for IR-POMDP planner not available");
        return -1;
    }

	intention_recognition_msgs::Intention srv_msg;
/*
    INITIALIZE MANIPULATION CLIENT
*/
    actionlib::SimpleActionClient<tables_demo_planning::PlanAndExecuteTasksAction> actionClient("/mobipick/task_planning", true);

    if(!actionClient.waitForServer(ros::Duration(5,0))){
        ROS_ERROR("Action service in IR-POMDP planner not available");
        return -1;
    }

/*
    PROCEED WITH PLANNING
*/
    int actionType;
    int observation = 0;
    double accuracy = 0.5;
    double reward;
    int steps = 0;
    double discount = POMDP->GetDiscount();
    bool terminal = false;

    //Loop until terminal state is reached
    do{
        //1. Plan and select action
        actionType = UNKNOWN_ACTION;
        int action = rr->PlanAction(0, 0); //or e.g. POMDP->A_PERCEIVE to force "observe worker"
        
        //Display Action
        if(use_mockup_GUI){
            cout << "Action selected. Please review and press a key..." << endl;
            DisplayAction(action);
            std::cin.get();
        }
        else{
            cout << "Executing action: " << endl;
            DisplayAction(action);
        }

        tables_demo_planning::PlanAndExecuteTasksGoal goal;

        //2. Execute the action
        //Action can be of type a) Perception or b) Manipulation
        //2.1 Determine type & prepare srv_msg
        if(action == POMDP->A_PERCEIVE){
            //Action is "perceive hotel progress"
            actionType = PERCEPTION_ACTION;            
            srv_msg.request.observation_type = srv_msg.request.OBSERVE_HOTEL_PROGRESS;
        }
        else if(action == POMDP->A_INSPECT_OBJECT){
            //Action is "inspect hotel TYPE"
            actionType = PERCEPTION_ACTION;
            srv_msg.request.observation_type = srv_msg.request.OBSERVE_HOTEL_TYPE;
        }
        else if(action == POMDP->A_INSPECT_CONTAINER){
            //Action is inspect ALL containers
            actionType = PERCEPTION_ACTION;
            srv_msg.request.observation_type = srv_msg.request.OBSERVE_LOCAL_STORAGE;
            //srv_msg.request.id = nContainer;
        }        
        else if(action >= POMDP->A_BRING_PARTS){
            //Action is restock part container
            actionType = MANIPULATION_ACTION;
            int partNumber = action - POMDP->A_BRING_PARTS;
            //TODO: review table order
            string part_str = partsID2partStr[partNumber];
            //TODO: determine destination/location w/o hard coding
            string destination = "table_1";

            
            //Restock parts (box_1, box_2, etc. for each part type)            
            //1. Create search task
            tables_demo_planning::Task searchTask;
            searchTask.task = "search_item";
            searchTask.parameters = {"mobipick", part_str};
            //2. Create bring task
            tables_demo_planning::Task bringTask;
            bringTask.task = "move_item";
            bringTask.parameters = {"mobipick", part_str, destination};
            //3. Add both to goal
            goal.tasks.push_back(searchTask);
            goal.tasks.push_back(bringTask);
        }

        //2.2 Send msg according to type and process contents
        if(actionType == PERCEPTION_ACTION){
            if(perceptionClient.call(srv_msg)){
                bool terminalObs = false;
                GetObsFromMSG(srv_msg, observation, accuracy, terminalObs);
                //Display observation
                DisplayObservation(observation, action);
                if(terminalObs){
                    ROS_INFO("Terminal state observed. Finalizing...");
                    terminal = terminalObs;
                }
            }
            else{
                ROS_ERROR("Failed to call service intention_recognition");
            }
        }
        else if (actionType == MANIPULATION_ACTION){
            ROS_INFO("Attempting to execute action on Mobipick...");
            actionClient.sendGoalAndWait(goal);
            ROS_INFO("Action finished on Mobipick.");
            tables_demo_planning::PlanAndExecuteTasksResult result = *(actionClient.getResult());
            if (result.success.size() >= 2) {
                observation = int(result.success[1]);
                ROS_INFO_STREAM("Success: " << observation << ", Message: " << result.message[1]);
            } else {
                observation = 0;
                ROS_INFO_STREAM("Success: " << observation << ", Message: Returned result has wrong size!");
            }
        }
        else if (actionType == UNKNOWN_ACTION){
            ROS_INFO("Action unknown, an error must have occurred.");
        }

        //3. Assign outcome reward
        /*
            Rewards +/- when:
            + trucks are completed
            - worker waits/fails (e.g. missing parts/glue)
            - parts brought exceed storage capacity (not needed)
        */
        reward = 0; //TODO: Determine external rewards!!
        
        //TODO: Receive terminal state signal from outside!!
        //For now, hack it:
        if(accuracy == 0.0)
            terminal = true;

        //4. Update POMDP with observation and reward
        // Technically, the POMDP is updated with only action and observation.
        // Reward is for performance monitoring
        // Observation accuracy is not really necessary; we assume the model is correct.
        int status = rr->Update(action, observation, reward, terminal); //status = e.g. out of particles, etc.

        steps++;
    }while(!terminal && ros::ok());

    /*RRLIB::RESULTS r = rr->getStatistics();
    ROS_INFO("Terminal state reached after %i steps (%f s.)", steps, r.time);
    ROS_INFO("Undiscounted return: %3.2f", r.undiscountedReturn);
    ROS_INFO("Discounted return: %3.2f", r.discountedReturn);*/

    return 0;
}

/*
    Process ROS MSG and extract observation info
    TODO: Adapt to process correct observation contents, i.e. boolean array
*/
void RRLIB_HOTEL::GetObsFromMSG(intention_recognition_msgs::Intention srv_msg, int& observation, double& accuracy, bool& terminal){
    // Hotel progress, i.e. parts assembled
    if(srv_msg.request.observation_type == srv_msg.request.OBSERVE_HOTEL_PROGRESS){
        ROS_INFO("Observe hotel progress:");
        vector<bool> ot;
        ot.resize(POMDP->NumContainers);

        int container = 0;
        
        terminal = true;
        for(auto p : srv_msg.response.intention_observation.worker_parts){
            bool _p = static_cast<bool>(p);            
            ot[container++] = _p;

            terminal &= _p; //TODO: determine termination criteria, we can NO LONGER use all parts
        }

        //Get obs. index
        observation = POMDP->getObservationIndex(ot);
        accuracy = srv_msg.response.intention_observation.worker_accuracy;
    }

    // Type of hotel    
    else if(srv_msg.request.observation_type == srv_msg.request.OBSERVE_HOTEL_TYPE){
        ROS_INFO("Observe hotel type:");
        observation = srv_msg.response.intention_observation.object_type;
        accuracy = srv_msg.response.intention_observation.object_type_accuracy;
    }

    // Local storage containers, ALL at once
    else if(srv_msg.request.observation_type == srv_msg.request.OBSERVE_LOCAL_STORAGE){
        ROS_INFO("Observe local storage:");
        vector<bool> ot;
        ot.resize(POMDP->NumContainers);
        int container = 0;

        //Get status aray and cast to bool (present, not present)
        for(auto p : srv_msg.response.intention_observation.storage_parts){
            bool _p = static_cast<bool>(p);
            ot[container++] = _p;
        }

        //Get obs. index
        observation = POMDP->getObservationIndex(ot);        
        accuracy = srv_msg.response.intention_observation.storage_accuracy;
    }
}

void RRLIB_HOTEL::InteractiveRun(){
    int A_PERCEIVE = 0;
    int A_INSPECT_TRUCK = 1;
    int A_INSPECT_CONTAINER = 2;

    //rr.InteractiveRun();

    bool terminal = false;
    do{
        int action = rr->PlanAction(0, 0);

        POMDP->DisplayAction(action,cout);

        //TODO
        //SEND ACTION TO ROS
        //RECEIVE observation, reward, terminal

        int observation = -1;

        cout << "Observation options: " << endl;
        STATE dummy;
        //Worker/hotel
        if(action == POMDP->A_PERCEIVE){
            vector<bool> obs;
            obs.resize(POMDP->NumContainers);

            std::string boolArray;
            cout << "Boolean list of assembled parts ("<< POMDP->NumContainers << "): ";
            std::cin >> boolArray;

            for(int i=0; i < obs.size(); i++)
                obs[i] = boolArray[i] - '0';

            observation = POMDP->getObservationIndex(obs);
        }
        //Hotel type
        else if(action == POMDP->A_INSPECT_OBJECT){
            cout << "{0 = N/A, 1 = Type A, 2 = Type B}" << endl;
            cout << "> ";
            while(observation < 0 || observation > 2)
                std::cin >> observation;
        }
        else if(action == POMDP->A_INSPECT_CONTAINER){
            vector<bool> obs;
            obs.resize(POMDP->NumContainers);

            std::string boolArray;
            cout << "Boolean list of available parts ("<< POMDP->NumContainers << "): ";
            std::cin >> boolArray;

            for(int i=0; i < obs.size(); i++)
                obs[i] = boolArray[i] - '0';

            observation = POMDP->getObservationIndex(obs);
        }
        else
            observation = 1;

        this->DisplayObservation(observation, action);

        int status;
        double reward = 0;

        //loop
        //action = rr.PlanAction(0, 0);
        status = rr->Update(action, observation, reward, terminal);
    }while(!terminal);
}
