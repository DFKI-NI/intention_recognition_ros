#include "rrlib_assembly.h"

RRLIB_ASSEMBLY::RRLIB_ASSEMBLY(string inputfile, string outputfile, int timeout, int simDoubles, int verbose, int rolloutPolicy, bool fTable){
    MCTS::PARAMS searchParams;
    RRLIB::RUN_PARAMS runParams;
    SIMULATOR::KNOWLEDGE knowledge;

    runParams.timeout = timeout;
    runParams.simDoubles = simDoubles;    
    
    searchParams.Verbose = verbose;
    searchParams.useFtable = fTable;

    knowledge.TreeLevel = 1;
    knowledge.RolloutLevel = rolloutPolicy;

    ASSEMBLY_PARAMS worker_params;
    ASSEMBLY_ROBOT_PARAMS problem_params;
    string domainFile;
        
    if(!RR_PARSER::parseAssemblyFile(problem_params, worker_params, domainFile, inputfile)){
        std::cerr << "Error parsing file." << endl;
        exit(0);
    }
    worker_params.ASSEMBLY_MWE(); //Initialize WORKER MWE instead of parsing input for now

    n_parts = worker_params.n_parts;
            
    POMDP = new ASSEMBLY_ROBOT(problem_params, worker_params);
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
	Use this constructor for the INCORAP demo.
	The paramFile lists all necessary parameters to construct the MCTS search class
	The paramFile also has the problemFile with the params to build the POMDP
*/
RRLIB_ASSEMBLY::RRLIB_ASSEMBLY(char ** argv, int argc){
    MCTS::PARAMS searchParams;
    RRLIB::RUN_PARAMS runParams;
    SIMULATOR::KNOWLEDGE knowledge;
    RR_PARSER::COMMAND_LINE cl;

    string problem, paramFile, problemFile, domainFile, outputfile, policy;

    if(!RR_PARSER::parseCommandLine(argv, argc, paramFile, problemFile, domainFile)){
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

    ASSEMBLY_PARAMS worker_params;
    ASSEMBLY_ROBOT_PARAMS problem_params;
    string domainFilename;
    
    //Read problem parameters
    if(!RR_PARSER::parseAssemblyFile(problem_params, worker_params, domainFilename, problemFile)){
        std::cerr << "Error parsing manual assembly file." << endl;
        exit(0);
    }

    //Read Manual Assembly problem description from JSON file

    if(domainFile == ""){
        cout << "Using default Manual Assembly domain" << endl;
        worker_params.ASSEMBLY_INCORAP(); //Initialize INCORAP Manual Assembly
    }
    else if(!RR_PARSER::ParseManualAssembly(domainFile, worker_params)){
        std::cerr << "Error parsing Manual Assembly domain description file." << endl;
        exit(0);
    }

    n_parts = worker_params.n_parts;
            
    POMDP = new ASSEMBLY_ROBOT(problem_params, worker_params);
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

int RRLIB_ASSEMBLY::PlanAction(int nSims, double timeout){
    return rr->PlanAction(nSims, timeout);
}

void RRLIB_ASSEMBLY::DisplayAction(int action){
    std::stringbuf str;
    std::ostream out(&str);
    //Display action on ostream
    POMDP->DisplayAction(action, out);
    //Get char* from stringbuf and send to ros_info
    ROS_INFO("%s", str.str().c_str());
}

void RRLIB_ASSEMBLY::DisplayObservation(int observation){
    std::stringbuf str;
    std::ostream out(&str);
    //display obs on ostream
    POMDP->DisplayObservation(dummyState, observation, out);        
    //Get char* from stringbuf and send to ros_info
    ROS_INFO("%s", str.str().c_str());
}

int RRLIB_ASSEMBLY::Update(int action, int observation, double reward, bool terminal){
    return rr->Update(action, observation, reward, terminal);
}

int RRLIB_ASSEMBLY::ROSRun(int argc, char ** argv){
    // Internal constants for ROS msg types
    int ROS_MSG_PERCEIVE_WORKER = 1;
    int ROS_MSG_INSPECT_CONTAINER = 2;
    int ROS_MSG_INSPECT_TRUCK = 3;
    int UNKNOWN_ACTION = -1;
    int PERCEPTION_ACTION = 0;
    int MANIPULATION_ACTION = 1;

/*
    INITIALIZE PERCEPTION CLIENT
*/
    ros::init(argc, argv, "intention_recognition_node");
    ros::NodeHandle nh;
    ros::ServiceClient perceptionClient = nh.serviceClient<intention_recognition_ros::Intention>("intention_recognition");

    bool serviceAvailable = ros::service::waitForService("intention_recognition", ros::Duration(5.0));

    if(!serviceAvailable){
        ROS_ERROR("Perception service for intention recognition not available");
        return -1;
    }

	intention_recognition_ros::Intention srv_msg;	
/*
    INITIALIZE MANIPULATION CLIENT
*/
    actionlib::SimpleActionClient<intention_recognition_ros::PlanAndExecuteAction> actionClient("incorap/planning/plan_and_execute", true);
    
    if(!actionClient.waitForServer(ros::Duration(5,0))){
        ROS_ERROR("Action service for intention recognition not available");
        return -1;
    }

    intention_recognition_ros::PlanAndExecuteGoal goal;
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
        cout << "Action selected. Please review and press a key..." << endl;	
        DisplayAction(action);
        std::cin.get();

        //2. Execute the action
        //Action can be of type a) Perception or b) Manipulation
        //2.1 Determine type & prepare srv_msg
        if(action == POMDP->A_PERCEIVE){
            //Action is "perceive worker"
            actionType = PERCEPTION_ACTION;
            srv_msg.request.observation_type = srv_msg.request.OBSERVE_WORKER;
        }
        else if(action == POMDP->A_INSPECT_TRUCK){
            //Action is "inspect truck"
            actionType = PERCEPTION_ACTION;
            srv_msg.request.observation_type = srv_msg.request.OBSERVE_TRUCK;
        }
        else if(action >= POMDP->A_INSPECT_CONTAINER && action < POMDP->A_BRING_GLUE){
            //Action is inspect container X
            actionType = PERCEPTION_ACTION;
            int nContainer = action - POMDP->A_INSPECT_CONTAINER;
            srv_msg.request.observation_type = srv_msg.request.OBSERVE_CONTAINER;
            srv_msg.request.id = nContainer;
        }
        else if(action >= POMDP->A_BRING_GLUE && action < POMDP->A_BRING_PARTS){
            //Action is bring glue 0, or 1, or...
            actionType = MANIPULATION_ACTION;
            int glueType = action - POMDP->A_BRING_GLUE;

            //Bring glue
            goal.task = "bring_item";
            goal.parameters = {"hot_glue_gun"};
        }
        else if(action >= POMDP->A_BRING_PARTS){
            //Action is restock part container
            actionType = MANIPULATION_ACTION;
            int partNumber = action - POMDP->A_BRING_PARTS;

            //Restock parts (box_1, box_2, etc. for each part type)
            //For now, move box to table.
            goal.task = "move_item";
            goal.parameters = {"box", "table_4"};
        }
        
        //2.2 Send msg according to type and process contents
        if(actionType == PERCEPTION_ACTION){
            if(perceptionClient.call(srv_msg)){
                GetObsFromMSG(srv_msg, observation, accuracy);
                //Display observation
                DisplayObservation(observation);
            }
            else{
                ROS_ERROR("Failed to call service intention_recognition");
            }
        }
        else if (actionType == MANIPULATION_ACTION){
            ROS_INFO("Attempting to execute action on Mobipick...");
            actionClient.sendGoalAndWait(goal);
            ROS_INFO("Action finished on Mobipick.");
            intention_recognition_ros::PlanAndExecuteResultConstPtr result = actionClient.getResult();
//            cout << result->success << "," << result->message << endl;
            ROS_INFO("Success: %i, Message: %s", result->success, result->message.c_str());
            

            //intention_recognition_ros::PlanAndExecuteResult result = *(actionClient.getResult());
            //ROS_INFO("Outcome: %B, Message: %s",result.success, result.message.c_str());
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
    }while(!terminal);

    RRLIB::RESULTS r = rr->getStatistics();
    ROS_INFO("Terminal state reached after %i steps (%f s.)", steps, r.time);
    ROS_INFO("Undiscounted return: %3.2f", r.undiscountedReturn);
    ROS_INFO("Discounted return: %3.2f", r.discountedReturn);

    return 0;
}

/*
    Process ROS MSG and extract observation info
*/
void RRLIB_ASSEMBLY::GetObsFromMSG(intention_recognition_ros::Intention srv_msg, int& observation, double& accuracy){
    // Worker activity
    if(srv_msg.request.observation_type == srv_msg.request.OBSERVE_WORKER){  
        ROS_INFO("Observe worker");
        vector<string> activities = POMDP->activities_str;
        vector<string> objects = POMDP->poses_str;
        vector<string> results = POMDP->outcomes_str;
        OBSERVATION_TRIPLE ot;

        ot.activity = ParseMSGContent(srv_msg.response.intention_observation.worker_action_type, activities);
        ot.pose = ParseMSGContent(srv_msg.response.intention_observation.worker_on_what, objects);
        ot.outcome = srv_msg.response.intention_observation.worker_success;

        accuracy = srv_msg.response.intention_observation.worker_accuracy;
        observation = POMDP->getObservationIndex(ot);
    }

    // Type of truck
    else if(srv_msg.request.observation_type == srv_msg.request.OBSERVE_TRUCK){
        ROS_INFO("Observe truck:");
        //ROS_INFO("T_type = %d", srv_msg.response.intention_observation.truck_type); // int8
        //ROS_INFO("O_accuracy = %f", srv_msg.response.intention_observation.truck_accuracy); // float32

        observation = srv_msg.response.intention_observation.truck_type;
        accuracy = srv_msg.response.intention_observation.truck_accuracy;
    }

    // Storage container
    else if(srv_msg.request.observation_type == srv_msg.request.OBSERVE_CONTAINER){
        ROS_INFO("Observe containers:");
        //ROS_INFO("C_status = %d", srv_msg.response.intention_observation.container_status); // int8, 0 = empty, 1 = not empty
        //ROS_INFO("O_accuracy = %f", srv_msg.response.intention_observation.container_accuracy); // float32

        observation = srv_msg.response.intention_observation.container_status;
        accuracy = srv_msg.response.intention_observation.container_accuracy;
    }
}

int RRLIB_ASSEMBLY::ParseMSGContent(string target, vector<string> options){
    int index = 0;    
    for(auto i : options){    
        if(i == target){    
            return index; //Return correct index
        }
        index++;
    }
    if(index >= options.size()) index = -1; //If exceeds size, target not found
    return index; //Return error code
}

void RRLIB_ASSEMBLY::InteractiveRun(){
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
        vector<string> activities = POMDP->activities_str;
        vector<string> objects = POMDP->poses_str;
        vector<string> results = POMDP->outcomes_str;
        //POMDP->getAORList(activities, objects, results);

        cout << "Observation options: " << endl;
        STATE dummy;
        //Worker
        if(action == POMDP->A_PERCEIVE){

            OBSERVATION_TRIPLE ot;

            cout << "A: ";
            for(int a=0; a<activities.size(); a++){
                cout << "[" << a << "] " << activities[a] << " | ";
            }
            cout << endl;
            cout << "> ";
            std::cin >> ot.activity;

            cout << "O: ";
            for(int o=0; o<objects.size(); o++){
                cout << "[" << o << "] " << objects[o] << " | ";;
            }
            cout << "> ";
            std::cin >> ot.pose;

            cout << "R: ";
            for(int r=0; r<results.size(); r++){
                cout << "[" << r << "] " << results[r] << " | ";;
            }
            cout << "> ";
            std::cin >> ot.outcome;

            observation = POMDP->getObservationIndex(ot);
        }
        //Truck & Container
        else if(action >= POMDP->A_INSPECT_TRUCK && action < POMDP->A_BRING_GLUE){
            cout << "{0, 1}" << endl;
            cout << "> ";
            while(observation != 0 && observation != 1)
                std::cin >> observation;
        }
        else
            observation = 1;

        POMDP->DisplayObservation(dummy, observation, cout);

        int status;    
        double reward = 0;
        
        //loop
        //action = rr.PlanAction(0, 0);
        status = rr->Update(action, observation, reward, terminal);
    }while(!terminal);
}
