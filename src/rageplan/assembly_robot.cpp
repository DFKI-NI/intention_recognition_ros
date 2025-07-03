#include "assembly_robot.h"
#include "utils.h"

#include <iomanip>
#include <fstream>

using namespace std;
using namespace UTILS;

void ASSEMBLY_ROBOT_STATE::activateFeature(int feature, bool status){
    if(feature >= NumTypes && feature < containers.size()){
        containers[feature - NumTypes].active = status;
    }
}


/* Build Ftable mapping every action to its affected feature/object */
/*
    In Mobipick, actions afforded by objects are: pick object, identify object
 */
void ASSEMBLY_ROBOT::initializeFTable(FTABLE& ftable) const{
// Add all corresponding action-feature pairs:
//    ftable.addEntry(action, feature);
    
    for(int f = 0; f < NumTypes; f++){        
        ftable.addEntry(A_BRING_GLUE + f, f);
    }

    //Each feature has one inspect and one bring
    for(int f = 0; f < NumContainers; f++){
        ftable.addEntry(A_INSPECT_CONTAINER + f, f + NumTypes);
        ftable.addEntry(A_BRING_PARTS + f, f + NumTypes);
    }
    
    //Add all perceives
    /*
    for(int f = 0; f < NumFeatures; f++)
        ftable.addEntry(A_PERCEIVE + f, f);
    */

    ftable.setTransitionRate(FTableTransitionRate);
    ftable.setACTIVATION_THRESHOLD(ASSEMBLY_ROBOT::ACTIVATION_THRESHOLD); ///Problem specific activation threshold
    ftable.setNumActions(NumActions);
    ftable.setNumFeatures(NumFeatures);

//	cout << "Done" << endl;
}

ASSEMBLY_ROBOT::ASSEMBLY_ROBOT(PROBLEM_PARAMS& problem_params, ASSEMBLY_PARAMS& ap){
    //Initialize local worker simulator
    worker.setProblem(ap);
    worker.getProblemDescription(parts_str, activities_str, poses_str, outcomes_str);
    this->part_cost = ap.part_cost;
    this->part_priority = ap.part_priority;
    this->initial_storage = ap.storage;
    
    ASSEMBLY_ROBOT_PARAMS& params = safe_cast<ASSEMBLY_ROBOT_PARAMS&>(problem_params);

    ///Read parameters from struct
    Discount = params.discount;  //MCTS discount
    fDiscount = params.fDiscount;  //Feature-value discount.  Probably also problem dependent.
    FTableTransitionRate = params.transitionRate; //FTable transition rate

//    cout << Discount << ", " << fDiscount << endl;

    ///Activity/Scene perception accuracy
    PERCEIVE_ACC = params.perceive;
    
    ///PGS
    BIN_ENTROPY_LIMIT = params.entropy;
    ///Feature activation threshold
    ACTIVATION_THRESHOLD = params.activation;
    ///PGS Alpha scaling factor
    PGSAlpha = params.PGSAlpha;

    //POMDP PARAMETERS
    NumObjects = ap.n_objs;
    NumContainers = ap.n_parts;
    NumTypes = ap.n_types;
    
    NumActivities = activities_str.size();
    NumPoses = poses_str.size();
    NumOutcomes = outcomes_str.size(); //FAIL, OK
    
    NumFeatures = NumTypes + NumContainers; //Storage units and glue types
    NumActions = (1 + 1 + NumContainers) + NumTypes + NumContainers; //Perceive (worker, truck, containers) + bring each glue + bring each object
    
    NumObservations = NumActivities*NumPoses*NumOutcomes; //All APO combinations
    
    RewardRange = 30; //NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts
    
    //Set action markers
    A_PERCEIVE = 0;
    A_INSPECT_TRUCK = 1;
    A_INSPECT_CONTAINER = 2;
    A_BRING_GLUE = A_INSPECT_CONTAINER + NumContainers; //After all observation actions
    A_BRING_PARTS = A_BRING_GLUE + NumTypes; //Extends until + NumContainers

    InitGeneral();
}

void ASSEMBLY_ROBOT::InitGeneral()
{    
    RandomSeed(0);
    //Add any more problem-specific init parameters
}

STATE* ASSEMBLY_ROBOT::Copy(const STATE& state) const
{        
    const ASSEMBLY_ROBOT_STATE& robotState = safe_cast<const ASSEMBLY_ROBOT_STATE&>(state);    
    ASSEMBLY_ROBOT_STATE* newstate = MemoryPool.Allocate();    
    *newstate = robotState;    
    return newstate;
}

void ASSEMBLY_ROBOT::Validate(const STATE& state) const
{
    const ASSEMBLY_ROBOT_STATE& robotState = safe_cast<const ASSEMBLY_ROBOT_STATE&>(state);
    
}

STATE* ASSEMBLY_ROBOT::CreateStartState() const
{
    
    ASSEMBLY_ROBOT_STATE* state = MemoryPool.Allocate();
    
    //Init worker state variables    
    ASSEMBLY_STATE * as = worker.createStartState();
    state->workerState.copy(*as);
    
    delete as;
    
    //Init robot state variables
    state->available = true;
    state->missing_glue = true;
    state->NumTypes = NumTypes;
    
    for(int i=0; i < NumObjects; i++){
        ASSEMBLY_ROBOT_STATE::PO_OBJ po_truck;
        
        po_truck.LikelihoodT0 = 1.0;
        po_truck.LikelihoodNotT0 = 1.0;
        po_truck.ProbT0 = 0.5;
        po_truck.assumedType = false;

        state->po_truck.push_back(po_truck);
    }
    
    //Clear array
    state->containers.clear();
    state->storage.clear();
    
    //Create parts/containers        
    bool names = false;    
    if(parts_str.size() == NumContainers) names = true;
    
    bool initStorage;
    if(initial_storage.size() == NumContainers) initStorage = true;
    
    for (int i=0; i < NumContainers; i++){
        ASSEMBLY_ROBOT_STATE::ELEMENT e;
        
        //Create ground truth storage
        if(initStorage)
            state->storage.push_back(initial_storage[i]);
        else
            state->storage.push_back( Random(1,8) ); //Init randomly
        
        //FO
        e.id = i;
        if(names)
            e.name = parts_str[i];
        else
            e.name = "Part " + std::to_string(e.id);
        
        e.capacity = 15; //TODO determine max capacity separately or externally
        e.cost = part_cost[i]; 
        e.priority = part_priority[i];
        
        e.measured = 0;
        e.count = 0;
        
        e.needed = false;
        //e.low = state->storage[i] < e.capacity/3; //TODO: validate low criteria
        e.empty = false;
        
        //Estimated
        e.LikelihoodEmpty = 1.0;
        e.LikelihoodNotEmpty = 1.0;
        e.ProbEmpty = 0.5;
        
        //IRE
        e.active = true;
        
        //Add PO object
        state->containers.push_back(e);        
    }
    
    return state;
}

void ASSEMBLY_ROBOT::FreeState(STATE* state) const
{
    ASSEMBLY_ROBOT_STATE* robotState = safe_cast<ASSEMBLY_ROBOT_STATE*>(state);
    MemoryPool.Free(robotState);
}

bool ASSEMBLY_ROBOT::Step(STATE& state, int action,
                  int& observation, double& reward) const
{
    //cout << "Rolloutlevel: " << Knowledge.RolloutLevel << endl;
    if(Knowledge.RolloutLevel >= KNOWLEDGE::PGS){
        //cout << "Calling Step PGS" << endl;
        return StepPGS(state, action, observation, reward);
    }
    else
        return StepNormal(state, action, observation, reward);
}

/*
 * Step function with PGS PBRS
 *
 * Performs regular step and adds the reward bonus
 *      gamma*phi(s') - phi(s)
 * where phi(s) = alpha*PGS(s) and gamma = 1
 *
*/
bool ASSEMBLY_ROBOT::StepPGS(STATE& state, int action,
                     int& observation, double& reward) const
{
   double scale = 10.0;
    double r = 0.0;
    double r2 = 0.0;
    STATE* oldstate = Copy(state);

    bool terminal = StepNormal(state, action, observation, reward);

    // Potential-based reward bonus
    //if(!terminal){//Not terminal or out of bounds
        r2 = PGS(*oldstate);
        r = PGS_RO(*oldstate, state, action, r2); //PGS(state);
        
        reward += PGSAlpha*r - PGSAlpha*r2;        
    //}
    FreeState(oldstate);

    return terminal;
}

/*
    Worker simulation. Advances the MC/MDP policy
*/

bool ASSEMBLY_ROBOT::simulateWorker(ASSEMBLY_STATE& state, vector<int>& storage, double& workerReward) const{
    //Simulate transition
	bool done = worker.Step(state);
    //Simulate/populate non-deterministic outcomes    
    worker.generateOutcomes(state, storage, workerReward);
    
    return done;
}


/*
 * Regular step function.  Simulates transition from state with action and returns observation and reward
*/
bool ASSEMBLY_ROBOT::StepNormal(STATE& state, int action,
                        int& observation, double& reward) const
{    
    reward = 0;
    double workerReward = 0;
    observation = 1;
    
    //ROBOT transition rewards
    double reward_perceive = -0.5;
    int terminal_success = 0;
    
    int reward_restock = -2; //-1
    int reward_wrongGoal = -5;
    int reward_bring_glue = -2;
	
    bool terminal = false;
    //if(action > NumActions){
    //    cout << "Starting step function, action no." << action << endl;
    //    DisplayAction(action, cout);
    //     DisplayState(state, cout);
    //}    
	
	ASSEMBLY_ROBOT_STATE& rState = safe_cast<ASSEMBLY_ROBOT_STATE&>(state);
        
    ///Process action and simulate results    
	/*     
		Actions are a sequence of numbers dictated by the following ranges:        
		A_PERCEIVE = 0
		A_BRING_GLUE = 1
		A_BRING_PARTS = A_BRING_GLUE + NumTypes
    */
    
    //cout << "Step -- ";
    //DisplayAction(action, cout);
		
	//cout << "Step with a = " << action << endl;
        
    // Bring obj. number
    double cost = 0; //get cost of bringing object
    if(action >= A_BRING_PARTS){
        int part = action - A_BRING_PARTS; //Part or container no. to refill
        
        /// Simulate robot absence
        for(int i=0; i < rState.containers[part].cost; i++){
            //Move worker one step
            workerReward = 0;
            terminal = this->simulateWorker(rState.workerState, rState.storage, workerReward);
            reward += workerReward; //includes successes and failures
            //Update container status
            if(rState.workerState.activity == worker.A_ASSEMBLE && rState.workerState.outcome == worker.O_FAIL){
                int part = rState.workerState.pose - worker.P_PART;
                rState.containers[part].needed = true;
            }

            if(terminal) break;
        }
        /// End sim absence
        
        //Simulate effects of bringing parts, we KNOW properties of storage containers once action is executed
        int diff = rState.containers[part].capacity - rState.storage[part];
        if(diff < REFILL_AMOUNT){        
            reward += reward_wrongGoal;
            //observation = O_FAIL;
            //If full, set at capacity
            rState.storage[part] = rState.containers[part].capacity;
        }
        else{
            reward += reward_restock;
            rState.storage[part] += REFILL_AMOUNT;
        }
        
        rState.containers[part].ProbEmpty = 0.0;
        rState.containers[part].empty = false; //Now we KNOW it is not empty
        rState.containers[part].needed = false; //Restocked parts are not needed anymore

    }
    else if(action >= A_BRING_GLUE){
        //observation = O_OK;
        reward += reward_bring_glue;
        int glue_type = action - A_BRING_GLUE; //Glue type
                            
        //Assume glue is immediate or within reach, no additional dynamics or cost
        int OK = worker.bringGlue(rState.workerState, glue_type);
    }
    
    else if(action >= A_INSPECT_CONTAINER){
        reward = reward_perceive;
        int container = action - A_INSPECT_CONTAINER;
        
        //Receive observation
        double efficiency = InspectContainer(rState, container, observation);
        
        //update info about containers                                    
        if(observation == 1){ //OK, Container > 0
            //Estimate prob. of container NOT being empty
            rState.containers[container].LikelihoodNotEmpty *= efficiency;
            rState.containers[container].LikelihoodEmpty *= 1 - efficiency;
            
            /*double denom = (0.5 * rState.containers[container].LikelihoodEmpty) + (0.5 * rState.containers[container].LikelihoodNotEmpty);
            rState.containers[container].ProbEmpty = (0.5 * rState.containers[container].LikelihoodEmpty) / denom;*/
            //rState.containers[container].ProbEmpty = 1 - efficiency;
        }
        //Otherwise no parts left
        else{
            //Estimate prob. of container being empty
            rState.containers[container].LikelihoodEmpty *= efficiency;
            rState.containers[container].LikelihoodNotEmpty *= 1 - efficiency;            
           // rState.containers[container].ProbEmpty = efficiency;
        }        
        double denom = (0.5 * rState.containers[container].LikelihoodEmpty) + (0.5 * rState.containers[container].LikelihoodNotEmpty);
        rState.containers[container].ProbEmpty = (0.5 * rState.containers[container].LikelihoodEmpty) / denom;
    }
    
    else if(action >= A_INSPECT_TRUCK){
        reward = reward_perceive;
        
        //Receive observation
        double efficiency = InspectObject(rState, observation);
        
        //update info about truck type
        if(observation == 0){
            rState.po_truck[rState.workerState.truck].LikelihoodT0 *= efficiency;
            rState.po_truck[rState.workerState.truck].LikelihoodNotT0 *= 1 - efficiency;
        }
        else{
            rState.po_truck[rState.workerState.truck].LikelihoodT0 *= 1 - efficiency;
            rState.po_truck[rState.workerState.truck].LikelihoodNotT0 *= efficiency;
        }
        double denom = (0.5 * rState.po_truck[rState.workerState.truck].LikelihoodT0) + (0.5 * rState.po_truck[rState.workerState.truck].LikelihoodNotT0);
        rState.po_truck[rState.workerState.truck].ProbT0 = (0.5 * rState.po_truck[rState.workerState.truck].LikelihoodT0) / denom;

        if(!rState.po_truck[rState.workerState.truck].assumedType)
            if(BinEntropyCheck(rState.po_truck[rState.workerState.truck].ProbT0))
                rState.po_truck[rState.workerState.truck].assumedType = true;
    }
    
    ///Perceive WORKER
    else if(action == A_PERCEIVE){
        reward = reward_perceive;
        OBSERVATION_TRIPLE obs; //Observation tuple
        double efficiency = Perceive(rState, obs); //Generate observation
        observation = getObservationIndex(obs); //Convert to index
                        
        //cout << "Perceived " << obs.activity << ", " << obs.pose << ", " << obs.outcome << endl;
        // If glueing fails, wrong or no glue is available
        /*if(obs.activity == ASSEMBLY_WORKER::A_GLUE){
            if(obs.outcome == ASSEMBLY_WORKER::O_OK)
                reward += reward_rightGoal; //Correct glue type was provided                
            else
                reward += reward_wrongGoal; //Wrong or missing glue
        }*/
        
        //If assembling a part, it either succeeds (part was available) or fails

        //If worker waits, something is wrong or missing
        //else if(obs.activity == ASSEMBLY_WORKER::A_WAIT){
        //    reward += reward_waiting;
       // }
    }
    
    ///Simulate worker transition and outcomes
	terminal = this->simulateWorker(rState.workerState, rState.storage, workerReward);
    reward += workerReward;
    
    //Worker would communicate container is empty, but we should avoid getting here!
    if(rState.workerState.activity == worker.A_ASSEMBLE && rState.workerState.outcome == worker.O_FAIL){
        int part = rState.workerState.pose - worker.P_PART;
        rState.containers[part].needed = true;
    }    
    
    //Update storage status
    for(auto s : rState.storage){
        if(s == 0)
            rState.containers[s].empty = true;
    }
    
    if(terminal){
		reward += terminal_success;
	}
	
    return terminal;
}


/* Local transformations: VALIDATE state updates
 * 1. Change container status
 * 2. Change worker APO
 * 3. Change truck type
*/
// TODO: consider using a diff. belief approximation for this task, no particle filter -> no local transformation necessary
bool ASSEMBLY_ROBOT::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const{
        
    ASSEMBLY_ROBOT_STATE& rState = safe_cast<ASSEMBLY_ROBOT_STATE&>(state);
            
    /*if(Bernoulli(0.5))
		incorapState.workerPose = Random(NumPoses);
	else
		incorapState.workerActivity = Random(NumActivities);*/
    
    int part;
    int action = history.Back().Action;
    
    int t = Random(3); //choose one random transformation
    //1. Modify one random container
    if(t == 0){        
        part = Random(rState.storage.size());        
        
        if(rState.storage[part] <= 0) rState.storage[part] = 1;
        else rState.storage[part]--;        
        if(rState.storage[part] == 0) rState.containers[part].empty = true;
        
        //Validate when action was restocking
        if(action >= A_BRING_PARTS){
            int realPart = action - A_BRING_PARTS;
            
            //If we just restocked, part cannot be empty
            if(realPart == part && rState.containers[part].empty)
                return false;
        }
        //Validate when action was inspecting container
        else if(action >= A_INSPECT_CONTAINER && action < A_BRING_GLUE){
            int container = action - A_INSPECT_CONTAINER;
            int realObs = history.Back().Observation; //External observation
                    
            int newObs;
            InspectContainer(rState, container, newObs); //Inspect previous container after random change
            
            if(realObs != newObs)
                return false;
        }
    }
    //2. Modify worker APO
    else if(t == 1){
        int r = Random(3);
        switch(r){
            case 0:
                rState.workerState.activity = Random(NumActivities);
                break;
            case 1:
                rState.workerState.pose = Random(NumPoses);
                break;
            case 2:
                rState.workerState.outcome = Random(NumOutcomes);
                break;
        }
        
        //Validate perceive worker
        if (action == A_PERCEIVE){
            int realObs = history.Back().Observation; //External observation
            
            OBSERVATION_TRIPLE newT;
            Perceive(rState, newT);
            int newObs = getObservationIndex(newT);
            
            if(realObs != newObs)
                return false;
        }
    }    
    //3. Change truck type
    else if(t == 2){
        rState.workerState.trucks[rState.workerState.truck].type = Random(NumTypes);
        
        //Validate
        if(action == A_INSPECT_TRUCK){
            int realObs = history.Back().Observation; //External observation
                    
            int newObs;
            InspectObject(rState, newObs);
            
            if(realObs != newObs)
                return false;
        }
    }
    
    //Validate:

    
           
        //Must validate change wrt. real obs (not new) because the worker transitioned already
        /*
        OBSERVATION_TRIPLE realT;
        getObservationFromIndex(realObs, realT);
        int workerPart = worker.P_PART + part; //Use part no. marker
        
        // If activity WAS assemble the part we changed
        // and outcome = FAIL transform is invalid, because the part is now at least 1        
        if(realT.activity == ASSEMBLY_WORKER::A_ASSEMBLE && realT.pose == workerPart){
            if(realT.outcome == ASSEMBLY_WORKER::O_FAIL)
                return false;
        }
        
        // If we observe something predictive of obj type, then cannot accept a diff. obj. type
        // If glueing succeeds, the state has the right type of truck
        int glue = worker.P_GLUE + rState.workerState.trucks[rState.workerState.truck].type;
        if(realT.activity == ASSEMBLY_WORKER::A_GLUE && realT.outcome == ASSEMBLY_WORKER::O_OK)
            return false;*/
            
    return true;
}

/* Fast PGS for Rollout policy
 * Simplified PGS point count by using only the specific action changes
 */
double ASSEMBLY_ROBOT::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
    double points = 0.0;
	double oldpoints = 0.0;
	
	//1. Cast to local state
	ASSEMBLY_ROBOT_STATE& rState = safe_cast<ASSEMBLY_ROBOT_STATE&>(state);
	ASSEMBLY_ROBOT_STATE& oldRState = safe_cast<ASSEMBLY_ROBOT_STATE&>(oldstate);
	      
    //2. +1 for truck assembled
    int t = 0;
	if(rState.workerState.activity == worker.A_GLUE && rState.workerState.outcome == worker.O_OK){
        points += PGS_goal;
        oldpoints += PGS_uncertain;
	}
	
	//3. Remove point if container status changed
	if(action >= A_BRING_PARTS){
        int part = action - A_BRING_PARTS;
        if(oldRState.containers[part].needed){
            oldpoints += PGS_notgoal;            
        }
    }

    //4. Remove point if truck uncertainty is lifted
    if(action == A_INSPECT_TRUCK){
        int truck = rState.workerState.truck;
        if( !BinEntropyCheck(rState.po_truck[truck].ProbT0) ) points += PGS_uncertain;

        if( !BinEntropyCheck(oldRState.po_truck[truck].ProbT0) ) oldpoints += PGS_uncertain;
    }


    //4.1 Reward informed bring-glue
/*
    if(action == A_BRING_GLUE){
        if(rState.po_truck[rState.workerState.truck].assumedType)
            if(round(rState.po_truck[rState.workerState.truck].ProbT0) == !rState.workerState.glueType) points += PGS_goal;

        if(oldRState.workerState.glueType > -1)
            if(oldRState.po_truck[oldRState.workerState.truck].assumedType)
                if(round(oldRState.po_truck[rState.workerState.truck].ProbT0) == !oldRState.workerState.glueType)
                oldpoints += PGS_goal;
    }
*/

	//Update difference
	double result = oldpgs - oldpoints + points;

	return result;
}

/*
 * PGS Point scoring for the INCORAP domain
 */
double ASSEMBLY_ROBOT::PGS(STATE& state) const
{    
    double points = 0.0;
	
	//1. Cast to cellarstate
	ASSEMBLY_ROBOT_STATE& rState = safe_cast<ASSEMBLY_ROBOT_STATE&>(state);
	
	//2. Evaluate objects: +1 for each truck assembled, -0.5 for remaining trucks?
	for(auto t : rState.workerState.trucks){
        if(t.complete){
            points += PGS_goal;
        }
        else
            points += PGS_uncertain;
    }
    
    //3. -1 for each empty but needed container
    for(auto c : rState.containers){
        if(c.needed) points += PGS_notgoal;
    }

    //4. Current truck uncertainty (informs bring glue)
    
    if( !BinEntropyCheck(rState.po_truck[rState.workerState.truck].ProbT0) ){
        points += PGS_uncertain;
    }
    
    //4.1 Reward bringing correct glue with good observations
/*    
    if(rState.workerState.glueType > -1)
        if(rState.po_truck[rState.workerState.truck].assumedType)
            if(round(rState.po_truck[rState.workerState.truck].ProbT0) == !rState.workerState.glueType)
                points += PGS_goal;
*/
    //4. Evaluate state: -1 if worker fails (meaning robot did not act in advance)
    /*if(rState.workerState.outcome == worker.O_FAIL){
        points -= PGS_notgoal;
    }*/
	
	return points;
}

// PGS Rollout policy
void ASSEMBLY_ROBOT::GeneratePGS(const STATE& state, const HISTORY& history,
                         vector<int>& legal, const STATUS& status) const
{
    static vector<int> acts;
	acts.clear();
	STATE * newstate;
	STATE * oldstate = Copy(state);
	PGSLegal(state, history, acts, status);
	int numLegal = acts.size();
	
	double pgs_values[numLegal]; //pgs_values[i] <-- legalMove[i]
	double pgs_state = PGS(*oldstate);
		
	int max_p = -1;
	double max_v = -Infinity;	
	
	int observation;
	double reward;
	
	//cout << "Generating PGS values..." << endl;
	//cout << "Found " << numLegal << " legal actions." << endl;
		
	for(unsigned int i=0; i<numLegal; i++){		
		newstate = Copy(state);
		
		StepNormal(*newstate, acts[i], observation, reward); //Simulate transition with action a
		
		// Using regular PGS (slow)
		//pgs_values[i] = PGS(*newstate);
		
		// Adding only PGS differences (fast)
		pgs_values[i] = PGS_RO(*oldstate, *newstate, acts[i], pgs_state); //add differences

		FreeState(newstate);
	}
	
	FreeState(oldstate);
	max_p = std::distance(pgs_values, max_element(pgs_values, pgs_values+numLegal));
	max_v = pgs_values[max_p];
	assert(max_p > -1);
	
	legal.push_back(acts[max_p]); //Add best action to return vector
	// Add other maxima
	for(int i=0; i<numLegal; i++){
		if(i != max_p && pgs_values[i] == max_v)
            legal.push_back(acts[i]);
	}
}

/*
 * Legal actions following PGS policy: avoid actions that do not reduce uncertainty
*/

void ASSEMBLY_ROBOT::PGSLegal(const STATE& state, const HISTORY& history,
                      vector<int>& legal, const STATUS& status) const
{    
    const ASSEMBLY_ROBOT_STATE& rState = safe_cast<const ASSEMBLY_ROBOT_STATE&>(state);
    
    //Allow perceive
    legal.push_back(A_PERCEIVE);

    //Allow inspect truck (if unknown)
    //if(!rState.po_truck[rState.workerState.truck].assumedType)
        //legal.push_back(A_INSPECT_TRUCK);
    
    //Allow bringing all glue types
    //TODO: add active variable to glue
    for(int i=0; i<NumTypes; i++) legal.push_back(A_BRING_GLUE + i);
    
    //Selectively allow restocking containers (but always allow scanning)
    for(int o=0; o < NumContainers; o++){        
        if(rState.containers[o].active){
            //legal.push_back(A_INSPECT_CONTAINER + o);
            legal.push_back(A_BRING_PARTS + o);
        }
    }
}


void ASSEMBLY_ROBOT::GenerateLegal(const STATE& state, const HISTORY& history,
                           vector<int>& legal, const STATUS& status) const
{  
	//Legal allows ALL actions
    for(int i=0; i<NumActions; i++)
        legal.push_back(i);
}

/*
  Preferred actions?
*/
void ASSEMBLY_ROBOT::GeneratePreferred(const STATE& state, const HISTORY& history,
                               vector<int>& actions, const STATUS& status) const
{
    GenerateLegal(state, history, actions, status);
}

///// Domain specific //////

/*
 * Perceive fills an observation tuple <activity, pose, outcome>
 * With some probability (efficiency) returns true values, otherwise noise.
 * 
 * Returns classification efficiency/accuracy
 * 
 * TODO: Determine correctly what kind of perception errors or noise may occur
 * 
 */
double ASSEMBLY_ROBOT::Perceive(const ASSEMBLY_ROBOT_STATE& state, OBSERVATION_TRIPLE& obs) const{
    double efficiency = PERCEIVE_ACC; //Maybe calculate based on state properties
    double pm_efficiency = 1-efficiency;
	    
    //With given efficiency, observe true activity,pose,outcome	
	obs.activity = state.workerState.activity;
	obs.pose = state.workerState.pose;
    obs.outcome = state.workerState.outcome;
	
    //with p - efficiency, randomize
	if(Bernoulli(pm_efficiency)){
        //Otherwise randomize one element
        int r = Random(3);
		switch(r){
            case 0:
                obs.activity = Random(NumActivities);
                break;
            case 1:
                obs.pose = Random(NumPoses);
                break;
            case 2:
                obs.outcome = Random(NumOutcomes);
                break;
        }
	}

	return efficiency;
}

// IR functions beyond INCORAP

//Determine if container is empty or not
double ASSEMBLY_ROBOT::InspectContainer(const ASSEMBLY_ROBOT_STATE& state, int container, int& obs) const{
    double efficiency = PERCEIVE_ACC;
    bool trueState = (state.storage[container] > 0);
    
    if(Bernoulli(efficiency)){
        obs = trueState;
    }
    else
        obs = !trueState;
    
    return efficiency;
}

//Estimate the type of truck from direct observation
//TODO: we are assuming for now 2 types, for simplicity
double ASSEMBLY_ROBOT::InspectObject(const ASSEMBLY_ROBOT_STATE& state, int& obs) const{
    
    double efficiency = PERCEIVE_ACC;
    if(state.workerState.trucks[state.workerState.truck].percentageComplete < 50)
        efficiency = 0.5;
        
    int trueType = state.workerState.trucks[state.workerState.truck].type;
    
    if(Bernoulli(efficiency)){
        obs = trueType;
    }
    else
        obs = !trueType;
    
    return efficiency;
}


/* Obs tuples are <activity, pose, outcome>
 * linear index = act*n_pos*n_out + pos*n_out + out
 */
int ASSEMBLY_ROBOT::getObservationIndex(OBSERVATION_TRIPLE& obs) const{
	int act = obs.activity;
	int pos = obs.pose;
	int out = obs.outcome;
	
	int index = act * NumPoses * NumOutcomes + pos * NumOutcomes + out;
	return index;
}

/*
 * Reconstruct observation tuple from index number
 * */
void ASSEMBLY_ROBOT::getObservationFromIndex(int index, OBSERVATION_TRIPLE& obs) const{	
    int local_index = 0;
	int NPNO = NumPoses*NumOutcomes;
    bool found = false;
	
	for(int a=0; a < NumActivities && !found; a++){
		for(int p=0; p < NumPoses && !found; p++){
			for(int o=0; o < NumOutcomes && !found; o++){
				local_index = a*NPNO + p*NumOutcomes + o;
				if(local_index == index){
					obs.activity = a;
					obs.pose = p;
					obs.outcome = o;
					found = true;
				}
			}
		}
	}

}

/*
 * Return whether value p satisfies the entropy restriction < BIN_ENTROPY_LIMIT in a Bernoulli distribution
 * 
 * true if check is passed, meaning the value satisfies the uncertainty threshold.
 * 
 * */
bool ASSEMBLY_ROBOT::BinEntropyCheck(double p) const {
    double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
    return (binaryEntropy <= ASSEMBLY_ROBOT::BIN_ENTROPY_LIMIT);
}

/*
 * OUTPUT FUNCTIONS
 */

void ASSEMBLY_ROBOT::DisplayBeliefs(const BELIEF_STATE& beliefState,
                            std::ostream& ostr) const
{
    int numSamples = beliefState.GetNumSamples();

    ostr << numSamples << " belief samples: ";
    for (int i=0; i<numSamples; i++)
    {
        const STATE* s = beliefState.GetSample(i);
        cout << "Sample " << i+1 << ":" << endl;
        DisplayState(*s, cout);
    }
    ostr << endl;
}

void ASSEMBLY_ROBOT::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const ASSEMBLY_ROBOT_STATE& rState = safe_cast<const ASSEMBLY_ROBOT_STATE&>(state);
	ostr << endl;
    
    //Display worker state    
    worker.DisplayState(rState.workerState, ostr);
    ostr << endl;
    
    //Truck type belief
    ostr << "Current truck: [T0 = ";
    ostr << std::setprecision(4) << rState.po_truck[rState.workerState.truck].ProbT0 << "]";
    ostr << " [T1 = ";
    ostr << std::setprecision(4) << 1-rState.po_truck[rState.workerState.truck].ProbT0 << "]";
    ostr << endl;
    ostr << endl;
    
    //Display container status    
	bool names = false;
	if(parts_str.size() == NumContainers) names = true;
	ostr << "Containers:" <<endl;
		ostr << "\t[";
		ostr << std::left << std::setw(8) << "Part No." << " | ";
		ostr << std::left << std::setw(16) << "Name" << " | ";
		ostr << std::left << std::setw(8) << "Qty." << " | ";
        ostr << std::left << std::setw(8) << "Needed?" << " | ";
        ostr << std::left << std::setw(9) << "P(empty)" << " | ";
        ostr << std::left << std::setw(5) << "Cost" << "]";
		ostr << endl;
		
	for(auto c : rState.containers){
		ostr << "\t[";
        //Part id
		ostr << std::left << std::setw(8) << c.id << " | ";
        //Part name
		if(names)
			ostr << std::left << std::setw(16) << parts_str[c.id];
		else
			ostr << std::left << std::setw(16) << "Part " << c.id;
		ostr << " | ";
		//the actual amount
		ostr << std::left << std::setw(8) << rState.storage[c.id] << " | ";
        //needed?        
        ostr << std::left << std::setw(8) << (c.needed? "Y" : "N") << " | ";
        //P. empty
        ostr << std::left << std::setw(9) << std::setprecision(4) << c.ProbEmpty << " | ";
        //Part cost
        ostr << std::left << std::setw(5) << c.cost << "]";
        
        ostr << endl;
	}

    ostr << endl;
}

//<activity, pose, outcome>
void ASSEMBLY_ROBOT::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
	OBSERVATION_TRIPLE obs;
	getObservationFromIndex(observation, obs);
	
	ostr << "O = <";
	
	//Activity
	ostr << activities_str[obs.activity] << ", ";
	
	//Pose
	ostr << poses_str[obs.pose] << ", ";
	
	//Outcome
    ostr << outcomes_str[obs.outcome];
	
	ostr << ">" << endl;
}

void ASSEMBLY_ROBOT::DisplayAction(int action, std::ostream& ostr) const
{

    ostr << "A: ";        
	if(action >= A_BRING_PARTS){
		int part = action - A_BRING_PARTS;
        
        ostr << "Bring ";
        if(parts_str.size() == NumContainers)
            ostr << parts_str[part];
        else
            ostr << " Part " << part;
        ostr << endl;
	}
	else if(action >= A_BRING_GLUE){
		int type = action - A_BRING_GLUE;
        ostr << "Bring glue type " << type << endl;
	}
	else if(action >= A_INSPECT_CONTAINER){
        int container = action - A_INSPECT_CONTAINER;
        ostr << "Inspect container " << container << endl;
    }
    else if(action == A_INSPECT_TRUCK){
        ostr << "Inspect truck" << endl;
    }
	else if (action == A_PERCEIVE){
        ostr << "Perceive" << endl;
    }
}

/*****/
/*
void ASSEMBLY_ROBOT::getAORList(vector<string>& act, vector<string>& obj, vector<string>& res) const{
    act = this->activities_str;
    obj = this->poses_str;
    res = this->outcomes_str;
}
*/