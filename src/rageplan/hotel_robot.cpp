#include "hotel_robot.h"
#include "utils.h"

#include <iomanip>
#include <fstream>

using namespace std;
using namespace UTILS;

void HOTEL_ROBOT_STATE::activateFeature(int feature, bool status){
    if(feature >= NumTypes && feature < containers.size()){
        containers[feature - NumTypes].active = status;
    }
}


/* Build Ftable mapping every action to its affected feature/object */
/*
    In Mobipick, actions afforded by objects are: pick object, identify object
 */
void HOTEL_ROBOT::initializeFTable(FTABLE& ftable) const{
// Add all corresponding action-feature pairs:
//    ftable.addEntry(action, feature);
    //ftable.addEntry(A_INSPECT_CONTAINER, f + NumTypes);
    
    //Each feature has one inspect and one bring --old
    for(int f = 0; f < NumContainers; f++){
        //ftable.addEntry(A_INSPECT_CONTAINER + f, f + NumTypes);
        ftable.addEntry(A_BRING_PARTS + f, f + NumTypes);
    }
    
    //Add all perceives
    /*
    for(int f = 0; f < NumFeatures; f++)
        ftable.addEntry(A_PERCEIVE + f, f);
    */

    ftable.setTransitionRate(FTableTransitionRate);
    ftable.setACTIVATION_THRESHOLD(HOTEL_ROBOT::ACTIVATION_THRESHOLD); ///Problem specific activation threshold
    ftable.setNumActions(NumActions);
    ftable.setNumFeatures(NumFeatures);

//	cout << "Done" << endl;
}

HOTEL_ROBOT::HOTEL_ROBOT(PROBLEM_PARAMS& problem_params, HOTEL_PARAMS& ap){
    //Initialize local worker simulator
    worker.setProblem(ap);
    worker.getProblemDescription(parts_str, activities_str, poses_str, outcomes_str);
    this->part_cost = ap.part_cost;
    this->part_priority = ap.part_priority;
    //this->initial_storage = ap.storage;
    
    HOTEL_ROBOT_PARAMS& params = safe_cast<HOTEL_ROBOT_PARAMS&>(problem_params);

    ///Read parameters from struct
    Discount = params.discount;  //MCTS discount
    fDiscount = params.fDiscount;  //Feature-value discount.  Probably also problem dependent.
    FTableTransitionRate = params.transitionRate; //FTable transition rate

//    cout << Discount << ", " << fDiscount << endl;

    ///Activity/Scene perception accuracy
    PERCEIVE_ACC = params.perceive;
    P_BRING_SUCCESS = params.bring_success;
    
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
    
    //Worker params
    NumActivities = activities_str.size();
    NumPoses = poses_str.size();
    NumOutcomes = outcomes_str.size(); //FAIL, OK
    
    NumFeatures = NumContainers; //All storage units/parts
    NumActions = (1 + 1 + 1) + NumContainers; //Perceive (worker, hotel type, containers) + bring each part
    
    NumObservations = pow(2,NumContainers); //MAX obs = all part combinations
    
    RewardRange = 30; //TODO:fine tune param. NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts
    
    //Set action markers
    A_PERCEIVE = 0;
    A_INSPECT_OBJECT = 1;
    A_INSPECT_CONTAINER = 2;
    A_BRING_PARTS = 3; //After all observation actions

    //Initialize support variables: hotel parts mapping, etc.
    //Get hotel part lists
    vector<int> hotelA = worker.type_map[0];
    vector<int> hotelB = worker.type_map[1];
    
    //Get parts unique to A
    vector<int> uniqueA, uniqueB;
	
    //Parts only in A    
	std::set_difference(hotelA.begin(), hotelA.end(), hotelB.begin(), hotelB.end(), std::inserter(uniqueA, uniqueA.begin()));
    uniqueParts.push_back(uniqueA);

    //Parts only in B
    std::set_difference(hotelB.begin(), hotelB.end(), hotelA.begin(), hotelA.end(), std::inserter(uniqueB, uniqueB.begin()));
    uniqueParts.push_back(uniqueB);

    //Parts in both A and B -> generalParts
    std::set_intersection(hotelA.begin(), hotelA.end(), hotelB.begin(), hotelB.end(), std::inserter(generalParts, generalParts.begin()));
    
    hotelA.clear();
    hotelB.clear();
    uniqueA.clear();
    uniqueB.clear();

    InitGeneral();
}

void HOTEL_ROBOT::InitGeneral()
{    
    //RandomSeed(time(NULL));
    RandomSeed(0);
    //Add any more problem-specific init parameters
}

STATE* HOTEL_ROBOT::Copy(const STATE& state) const
{        
    const HOTEL_ROBOT_STATE& robotState = safe_cast<const HOTEL_ROBOT_STATE&>(state);    
    HOTEL_ROBOT_STATE* newstate = MemoryPool.Allocate();    
    *newstate = robotState;    
    return newstate;
}

void HOTEL_ROBOT::Validate(const STATE& state) const
{
    const HOTEL_ROBOT_STATE& robotState = safe_cast<const HOTEL_ROBOT_STATE&>(state);
    
}

STATE* HOTEL_ROBOT::CreateStartState() const
{
    
    HOTEL_ROBOT_STATE* state = MemoryPool.Allocate();
    
    //Init worker state variables    
    HOTEL_STATE * hs = worker.createStartState();
    state->workerState.copy(*hs);    
    delete hs;
    
    //Init robot state variables
    state->available = true;
    state->NumTypes = NumTypes;
    
    //PO Hotels, all initially unknown types
    for(int i=0; i < NumObjects; i++){
        HOTEL_ROBOT_STATE::PO_OBJ po_hotel;
        
        po_hotel.LikelihoodT0 = 1.0;
        po_hotel.LikelihoodNotT0 = 1.0;
        po_hotel.ProbT0 = 0.5;
        po_hotel.assumedType = false;

        state->po_objects.push_back(po_hotel);
    }
    
    //Clear array
    state->containers.clear();
    //state->storage.clear();
    state->containerStatus.clear();
    
    //Create parts/containers        
    bool names = false;    
    if(parts_str.size() == NumContainers) names = true;
    
    //*** This version uses no declared storage ***/
    //bool initStorage;
    //if(initial_storage.size() == NumContainers) initStorage = true;
    
    for (int i=0; i < NumContainers; i++){
        HOTEL_ROBOT_STATE::ELEMENT e;
        
        //Create ground truth storage -- NO STORAGE INFO
        /*
        if(initStorage)
            state->storage.push_back(initial_storage[i]);
        else
            state->storage.push_back( Random(1,8) ); //Init randomly
        */
        
        //FO
        e.id = i;
        if(names)
            e.name = parts_str[i];
        else
            e.name = "Part " + std::to_string(e.id);
        
        e.capacity = 15; //TODO: determine max capacity separately or externally. Currently not used.
        e.cost = part_cost[i]; 
        e.priority = part_priority[i];
        
        e.measured = 0;
        e.count = 0;
        
        e.needed = false;
        e.empty = Bernoulli(0.5); //Container status unknown, initialized randomly
        state->containerStatus.push_back(!e.empty); //Based on the above
        state->p_empty.push_back(0.5); //same as ProbEmpty but shareable
        
        //Estimated
        e.LikelihoodEmpty = 1.0;
        e.LikelihoodNotEmpty = 1.0;
        e.ProbEmpty = 0.5;

        e.LikelihoodAssembled = 1.0;
        e.LikelihoodNotAssembled = 1.0;
        e.ProbAssembled = 0.5;

        //IRE
        e.active = true;
        
        //Add PO object
        state->containers.push_back(e);        
    }
    
    return state;
}

void HOTEL_ROBOT::FreeState(STATE* state) const
{
    HOTEL_ROBOT_STATE* robotState = safe_cast<HOTEL_ROBOT_STATE*>(state);
    MemoryPool.Free(robotState);
}

bool HOTEL_ROBOT::Step(STATE& state, int action,
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
bool HOTEL_ROBOT::StepPGS(STATE& state, int action,
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

bool HOTEL_ROBOT::simulateWorker(HOTEL_STATE& state, vector<bool>& storage, double& workerReward) const{
    //Simulate transition
	bool done = worker.Step(state);
    //Simulate/populate non-deterministic outcomes    
    worker.generateOutcomes(state, storage, workerReward);
    
    return done;
}


/*
 * Regular step function.  Simulates transition from state with action and returns observation and reward
*/
bool HOTEL_ROBOT::StepNormal(STATE& state, int action,
                        int& observation, double& reward) const
{    
    reward = 0;
    double workerReward = 0;
    observation = 1;
    
    //ROBOT transition rewards
    double reward_perceive = -0.5;
    double reward_wrongPerceive = -1; //WAS -5. when it is pointless to perceive, e.g, too few or wrong parts present
    int terminal_success = 0;
    
    int reward_restock = -2; //Try: 0
    int reward_goodRestock = 2;
    int reward_wrongGoal = -10;    
	
    bool terminal = false;
    //if(action > NumActions){
    //    cout << "Starting step function, action no." << action << endl;
    //    DisplayAction(action, cout);
    //     DisplayState(state, cout);
    //}    
	
	HOTEL_ROBOT_STATE& rState = safe_cast<HOTEL_ROBOT_STATE&>(state);
        
    ///Process action and simulate results    
	/*     
		Actions are a sequence of numbers dictated by the following ranges:        
		A_PERCEIVE = 0
		A_BRING_GLUE = 1
		A_BRING_PARTS = A_BRING_GLUE + NumTypes
    */

   /*
    Prepare params for next/current state
   */
    //TODO: how to update prob assembly and prob empty over time.
    //Unassembled parts update after one step    
    
    /*
    double gamma = 0.2; //prob. decay rate
    for(auto& c : rState.containers){
        //Prob. decay for assembly objects: they may get assembled
        if(c.ProbAssembled < 0.5){
            c.LikelihoodNotAssembled = (1 - gamma)*c.LikelihoodNotAssembled + gamma;
            c.LikelihoodAssembled = (1 - gamma)*c.LikelihoodAssembled + gamma;            
            c.ProbAssembled = (0.5 * c.LikelihoodAssembled) / ( (0.5 * c.LikelihoodAssembled) + (0.5 * c.LikelihoodNotAssembled) );
        }

        //Prob. decay for non-empty containers: they might become empty
        if(c.ProbEmpty < 0.5){
            c.LikelihoodEmpty = (1-gamma)*c.LikelihoodEmpty + gamma;
            c.LikelihoodNotEmpty = (1-gamma)*c.LikelihoodNotEmpty + gamma;
            c.ProbEmpty = (0.5 * c.LikelihoodEmpty) / ( (0.5 * c.LikelihoodEmpty) + (0.5 * c.LikelihoodNotEmpty));
        }
    }*/
    
    //cout << "Step -- ";
    //DisplayAction(action, cout);
		
	//cout << "Step with a = " << action << endl;
        
    // Bring obj. number
    double cost = 0; //get cost of bringing object
    if(action >= A_BRING_PARTS){
        int part = action - A_BRING_PARTS; //Part or container no. to refill
        
        /// TODO: Simulate robot absence
        for(int i=0; i < rState.containers[part].cost; i++){
            //Move worker one step
            workerReward = 0;
            terminal = this->simulateWorker(rState.workerState, rState.containerStatus, workerReward);
            reward += workerReward; //includes successes and failures
            //Update container status
            if(rState.workerState.action == worker.A_ASSEMBLE && rState.workerState.result == worker.O_FAIL){
                int part = rState.workerState.object - worker.P_PART;
                rState.containers[part].needed = true;
            }

            if(terminal) break;
        }
        /// End sim absence

        //Sim action failure
        if(Bernoulli(P_BRING_SUCCESS)){
            observation = 1; //Bringing was successful

            //Simulate effects of bringing parts, we KNOW properties of storage containers once action is executed
            //int diff = rState.containers[part].capacity - rState.storage[part];
            //if(rState.containers[part].ProbEmpty){
            
            //1. Reliable: P(empty) and P(assembled) are already estimated
            bool reliable = BinEntropyCheck(rState.p_empty[part]) && BinEntropyCheck(rState.containers[part].ProbAssembled);
            //2. Needed: Worker attempted to assemble part
            bool needed = rState.containers[part].needed;
            //3. Empty: p(empty) is estimated and close to 1
            bool empty = !(rState.containerStatus[part]); //rState.p_empty[part] > 0.5;
            //4. Assembled: p(assembled) is estimated and close to 1
            bool assembled = rState.workerState.allParts[part].assembled; //rState.containers[part].ProbAssembled > 0.5;

            //5. Is part unique?
            bool uniqueA = contains(uniqueParts[0], part);
            bool uniqueB = contains(uniqueParts[1], part);
            //5.1 Do we know the hotel type?
            bool typeKnown = rState.po_objects[rState.workerState.hotel].assumedType;
            //5.2 Does the type match with A or B?
            bool partMatch = (rState.po_objects[rState.workerState.hotel].ProbT0 > 0.5 && uniqueA) ||
                            (rState.po_objects[rState.workerState.hotel].ProbT0 < 0.5 && uniqueB);


            //Robot should bring parts when they are:
            // Already Needed, or
            // Not yet assembled and missing with reliable info
            // We used **  if( (needed) || (!assembled && empty && reliable)) ** before but it makes more sense to have a rule for BAD bring

            //New rule for bringing parts
            // TODO: reevaluate for multiple hotels in a row, because assembled status may affect behavior
            if(!reliable || assembled || !empty)
                reward += reward_wrongGoal; //Cases where bringing IS bad
            else if( (uniqueA || uniqueB) && (!typeKnown || !partMatch) )
                reward += reward_wrongGoal;
            else if(needed)
                reward += reward_goodRestock;
            else
                reward += reward_restock;

            //Should bring give free status knowledge? may cause extra bonus rewards...
            
            //Reset p. empty
            rState.containers[part].LikelihoodEmpty = 1.0;
            rState.containers[part].LikelihoodNotEmpty = 1.0;
            rState.containers[part].ProbEmpty = 0.5;
            rState.p_empty[part] = 0.0;

            //Part is now available
            rState.containerStatus[part] = true;            
            rState.containers[part].empty = false; //Now we KNOW it is not empty            
            rState.containers[part].needed = false; //Restocked parts are not needed anymore
        }
        else{
            //Action fails
            observation = 0;
            reward += reward_restock;
        }
        

    }
    // Inspect container status, approx. p(empty)
    else if(action == A_INSPECT_CONTAINER){
        reward = reward_perceive;
        //int container = action - A_INSPECT_CONTAINER;
        //Receive observation
        //double efficiency = InspectContainer(rState, container, observation);

        std::vector<bool> obs; //Observation tuple
        double efficiency = InspectAllContainers(rState, obs); //Generate observation
        observation = getObservationIndex(obs); //Convert to index
        
        for(int container=0; container < NumContainers; container++){
            //update info about containers
            if(obs[container]){ //OK, Container > 0
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

            rState.p_empty[container] = rState.containers[container].ProbEmpty;
        }
    }
    // Inspect hotel and approximate type
    else if(action == A_INSPECT_OBJECT){
        reward = reward_perceive;
        
        //Receive observation
        double efficiency = InspectObject(rState, observation);
        
        if(observation == 0)
            reward = reward_wrongPerceive;

        //Update hotel type info
        else if(observation == 1){
            rState.po_objects[rState.workerState.hotel].LikelihoodT0 *= efficiency;
            rState.po_objects[rState.workerState.hotel].LikelihoodNotT0 *= 1 - efficiency;
        }
        else if(observation == 2){
            rState.po_objects[rState.workerState.hotel].LikelihoodT0 *= 1 - efficiency;
            rState.po_objects[rState.workerState.hotel].LikelihoodNotT0 *= efficiency;
        }
        double denom = (0.5 * rState.po_objects[rState.workerState.hotel].LikelihoodT0) + (0.5 * rState.po_objects[rState.workerState.hotel].LikelihoodNotT0);
        rState.po_objects[rState.workerState.hotel].ProbT0 = (0.5 * rState.po_objects[rState.workerState.hotel].LikelihoodT0) / denom;

        if(!rState.po_objects[rState.workerState.hotel].assumedType)
            if(BinEntropyCheck(rState.po_objects[rState.workerState.hotel].ProbT0))
                rState.po_objects[rState.workerState.hotel].assumedType = true;
    }
    
    ///Perceive HOTEL STATUS/PROGRESS, by receiving a binary part array
    else if(action == A_PERCEIVE){
        reward = reward_perceive;
        std::vector<bool> obs; //Observation tuple
        double efficiency = Perceive(rState, obs); //Generate observation
        observation = getObservationIndex(obs); //Convert to index
        
        //iterate over detected assembled parts and update their likelihood/prob
        for(int i=0; i < NumContainers; i++){
            bool ass = obs[i]; //Is the current part assembled?
            if(ass){
                rState.containers[i].LikelihoodAssembled *= efficiency;
                rState.containers[i].LikelihoodNotAssembled *= 1 - efficiency;
            }
            else{
                rState.containers[i].LikelihoodNotAssembled *= efficiency;
                rState.containers[i].LikelihoodAssembled *= 1 - efficiency;
            }

            //Speed up updates, don't let likelihoods get so close to zero
            rState.containers[i].LikelihoodAssembled = std::max(0.001, rState.containers[i].LikelihoodAssembled);
            rState.containers[i].LikelihoodNotAssembled = std::max(0.001, rState.containers[i].LikelihoodNotAssembled);;

            double denom = (0.5 * rState.containers[i].LikelihoodAssembled) + (0.5 * rState.containers[i].LikelihoodNotAssembled);
            rState.containers[i].ProbAssembled = (0.5 * rState.containers[i].LikelihoodAssembled) / denom;
        }

    }
    
    ///Simulate worker transition and outcomes
	terminal = this->simulateWorker(rState.workerState, rState.containerStatus, workerReward);
    reward += workerReward;
    
    //Worker would communicate container is empty, but we should avoid getting here!
    if(rState.workerState.action == worker.A_ASSEMBLE && rState.workerState.result == worker.O_FAIL){
        int part = rState.workerState.object - worker.P_PART;
        rState.containers[part].needed = true;
    }       
    
    if(terminal){
		reward += terminal_success;
	}
	
    return terminal;
}


/* Local transformations: VALIDATE state updates
 * 1. Change container status
 * 2. Change worker <AOR>
 * 3. Change hotel type --Also changes their behavior
*/
// TODO: consider using a diff. belief approximation for this task, no particle filter -> no local transformation necessary
bool HOTEL_ROBOT::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const{
        
    HOTEL_ROBOT_STATE& rState = safe_cast<HOTEL_ROBOT_STATE&>(state);
    
    int part;
    int action = history.Back().Action;
    
    int t = Random(3); //choose one random transformation

    //1. Modify one random container
    if(t == 0){        
        part = Random(NumContainers);
        
        rState.containerStatus[part] = !rState.containerStatus[part];
        rState.p_empty[part] = 1 - rState.p_empty[part];
        
        //Validate when action was restocking
        if(action >= A_BRING_PARTS){
            int realPart = action - A_BRING_PARTS;
            
            //If we just restocked, part cannot be empty
            if(realPart == part && !rState.containerStatus[part])
                return false;
        }

        //Validate when action was inspecting container
        //else if(action >= A_INSPECT_CONTAINER && action < A_BRING_PARTS){
        else if(action == A_INSPECT_CONTAINER){
            //int container = action - A_INSPECT_CONTAINER;
            int realObs = history.Back().Observation; //External observation
                                
            std::vector<bool> newObsA;
            InspectAllContainers(rState, newObsA); //Inspect previous container after random change
            int newObs = getObservationIndex(newObsA);
            
            newObsA.clear();

            if(realObs != newObs)
                return false;
        }
    }

    //2. Modify hotel progress
    else if(t == 1){
        int hotelPart = Random(rState.workerState.allParts.size());

        //Change assembly status of random part (from ALL parts)
        rState.workerState.allParts[hotelPart].assembled = !rState.workerState.allParts[hotelPart].assembled;
        
        //Validate perceive worker
        if (action == A_PERCEIVE){
            int realObs = history.Back().Observation; //External observation
            
            std::vector<bool> newT;
            Perceive(rState, newT);
            int newObs = getObservationIndex(newT);
            newT.clear();

            if(realObs != newObs)
                return false;
        }
    }

    //3. Change hotel type (will lead worker to assemble from wrong part list)
    else if(t == 2){
        rState.workerState.hotels[rState.workerState.hotel].type = Random(NumTypes);
        
        //Validate
        if(action == A_INSPECT_OBJECT){
            int realObs = history.Back().Observation; //External observation
                    
            int newObs;
            InspectObject(rState, newObs);
            
            if(realObs != newObs)
                return false;
        }
    }
            
    return true;
}

/* Fast PGS for Rollout policy
 * Simplified PGS point count by using only the specific action changes
 */
double HOTEL_ROBOT::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
    double points = 0.0;
	double oldpoints = 0.0;
	
	//Cast to local state
	HOTEL_ROBOT_STATE& rState = safe_cast<HOTEL_ROBOT_STATE&>(state);
	HOTEL_ROBOT_STATE& oldRState = safe_cast<HOTEL_ROBOT_STATE&>(oldstate);
	      
    //1. +1 for assembling a hotel
    int t = 0;
	if(rState.workerState.action == worker.A_NONE && rState.workerState.result == worker.O_DONE){
        points += PGS_goal;
        oldpoints += PGS_uncertain;
	}
	
	//2. Remove neg. point if container status changed
	if(action >= A_BRING_PARTS){
        int part = action - A_BRING_PARTS;
        if(oldRState.containers[part].needed){
            oldpoints += PGS_notgoal;
        }
    }

    //3. Remove point if hotel type uncertainty is lifted
    if(action == A_INSPECT_OBJECT){
        int hotel = rState.workerState.hotel;
        if( !BinEntropyCheck(rState.po_objects[hotel].ProbT0) ) points += PGS_uncertain;

        if( !BinEntropyCheck(oldRState.po_objects[hotel].ProbT0) ) oldpoints += PGS_uncertain;
    }

    //4. Remove point if container uncertainty if lifted
    if(action == A_INSPECT_CONTAINER){
        //int container = action - A_INSPECT_CONTAINER;
        for(auto container : rState.p_empty)
            if (!BinEntropyCheck(container)) points += PGS_uncertain;
        for(auto oldContainer : oldRState.p_empty)
            if (!BinEntropyCheck(oldContainer)) oldpoints += PGS_uncertain;
    }

	//Update difference
	double result = oldpgs - oldpoints + points;

	return result;
}

/*
 * PGS Point scoring for the INCORAP domain
 */
double HOTEL_ROBOT::PGS(STATE& state) const
{    
    double points = 0.0;
	
	//Cast to cellarstate
	HOTEL_ROBOT_STATE& rState = safe_cast<HOTEL_ROBOT_STATE&>(state);
	
	//1. Evaluate objects: +1 for each hotel assembled, -0.5 for pending hotels
	for(auto t : rState.workerState.hotels){
        if(t.complete){
            points += PGS_goal;
        }
        else
            points += PGS_uncertain;
    }

    //1.5 Grant points for parts assembled
    /*for(auto p : rState.workerState.allParts){
        if(p.assembled)
            points += PGS_goal;
    }*/
    
    //2. -1 for each empty but needed container
    for(auto c : rState.containers){
        if(c.needed) points += PGS_notgoal;
    }

    //3. Hotel type uncertainty    
    if( !BinEntropyCheck(rState.po_objects[rState.workerState.hotel].ProbT0) ){
        points += PGS_uncertain;
    }

    //4. Container status uncertainty
    for(auto p : rState.p_empty){
        if(!BinEntropyCheck(p)) points += PGS_uncertain;
    }
	
	return points;
}

// PGS Rollout policy
void HOTEL_ROBOT::GeneratePGS(const STATE& state, const HISTORY& history,
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

void HOTEL_ROBOT::PGSLegal(const STATE& state, const HISTORY& history,
                      vector<int>& legal, const STATUS& status) const
{
    const HOTEL_ROBOT_STATE& rState = safe_cast<const HOTEL_ROBOT_STATE&>(state);
    
    //Allow perceive worker/hotel progress
    legal.push_back(A_PERCEIVE);

    //TODO: not allowing this yielded good results BEFORE
    //Inspect object TYPE
    //If not already known
    if(!rState.po_objects[rState.workerState.hotel].assumedType)
        legal.push_back(A_INSPECT_OBJECT);
    
    //Allow scanning status of all containers
    legal.push_back(A_INSPECT_CONTAINER);

    //Selectively allow restocking containers

    //IDEA: prefer general parts, and bring exclusive parts if they match known hotel type
    for(int o=0; o < NumContainers; o++){        
        if(rState.containers[o].active){
            //NOT ALLOWING inspect container yielded good results BEFORE
            //legal.push_back(A_INSPECT_CONTAINER + o);
            legal.push_back(A_BRING_PARTS + o);
        }
    }
}

void HOTEL_ROBOT::GenerateLegal(const STATE& state, const HISTORY& history,
                           vector<int>& legal, const STATUS& status) const
{  
	//Legal allows ALL actions
    for(int i=0; i<NumActions; i++)
        legal.push_back(i);
}

/*
  Preferred actions?
*/
void HOTEL_ROBOT::GeneratePreferred(const STATE& state, const HISTORY& history,
                               vector<int>& actions, const STATUS& status) const
{
    GenerateLegal(state, history, actions, status);
}

///// Domain specific //////

//Simple wrapper
bool HOTEL_ROBOT::contains(vector<int> v, int element) const{
    return std::find(v.begin(), v.end(), element) != v.end();
}

/*
 * Hotel Perceive looks at "activities" through the hotel progress.
 * We simulate by generating a boolean array that describes which hotel parts are present, as
 * perceived by the camera/image processing part.
 * 
 */
double HOTEL_ROBOT::Perceive(const HOTEL_ROBOT_STATE& state, std::vector<bool>& obs) const{
    double efficiency = PERCEIVE_ACC; //Maybe calculate based on state properties
    double pm_efficiency = 1-efficiency;
    obs.resize(NumContainers);
	    
    //With given efficiency, observe presence or absence of all possible parts
	for(auto p : state.workerState.allParts)
        obs[p.number] = p.assembled;
	
    //with p - efficiency, randomize ONE
	if(Bernoulli(pm_efficiency)){
        //Otherwise randomize one element
        int r = Random(NumContainers);        
		obs[r] = !obs[r];
	}

	return efficiency;
}

// Inspect container and approximate status: empty, not empty
double HOTEL_ROBOT::InspectContainer(const HOTEL_ROBOT_STATE& state, int container, int& obs) const{
    double efficiency = PERCEIVE_ACC;
    bool trueState = state.containerStatus[container]; //(state.storage[container] > 0);
    
    if(Bernoulli(efficiency)){
        obs = trueState;
    }
    else
        obs = !trueState;
    
    return efficiency;
}

//Inspect ALL containers at once and return bin array with status
double HOTEL_ROBOT::InspectAllContainers(const HOTEL_ROBOT_STATE& state, std::vector<bool>& obs) const{
    double efficiency = PERCEIVE_ACC;

    //obs.resize(state.containerStatus.size());
    obs.clear();
    obs.resize(NumContainers);
    obs = state.containerStatus;
    
    //Modify one value with p(1 - efficiency)
    if(Bernoulli(1 - efficiency)){
        int r = Random(NumContainers);
		obs[r] = !obs[r];
    }
    
    return efficiency;
}


// Inspect and estimate hotel TYPE
//TODO: we are assuming only 2 types
// *** IMPORTANT *** : for simplicity (negation and so on) hotel types are 0 and 1 BUT inspect returns:
// 0 when obs. is indeterminate/ambiguous/incorrect
// 1 when type is A
// 2 when type is B
double HOTEL_ROBOT::InspectObject(const HOTEL_ROBOT_STATE& state, int& obs) const{
    
    double efficiency = PERCEIVE_ACC;

    //Conflicts for obs = 0 include: no unique parts assembled, mutually exclusive parts assembled
    //STRONG ASSUMPTION: ONLY TWO HOTEL TYPES!!!    
    
    //Get parts unique to A    
	bool uniquePartsA = false;
    //Is there at least one unique assembled part from hotel A?
    for(auto p : uniqueParts[0]){        
        uniquePartsA |= state.workerState.allParts[p].assembled;        
    }

    //Get parts unique to B    
	bool uniquePartsB = false;
    //Is there at least one unique assembled part from hotel B?
    for(auto p : uniqueParts[1]){
        uniquePartsB |= state.workerState.allParts[p].assembled;        
    }

    //CONFLICT:
    if((!uniquePartsA && !uniquePartsB) || (uniquePartsA && uniquePartsB)){
        obs = 0;
        efficiency = 0.5;
    }
    else{
    //If no conflict, i.e. unique parts exist for only ONE hotel, reveal true type with prob.        
        int trueType = state.workerState.hotels[state.workerState.hotel].type;
        
        //Return true type with sensor efficiency
        if(Bernoulli(efficiency))
            obs = trueType;
        else
            obs = !trueType;

        obs += 1; //adjust for obs 0 offset
    }
    return efficiency;
}


/* Simply convert bool array into a decimal number
 */
int HOTEL_ROBOT::getObservationIndex(std::vector<bool>& obs) const{
    int pos = obs.size() - 1; //starting position to convert values
    int index = 0; //observation index
    for(auto b : obs){
        if(b) index += boolValueTable[pos];
        pos--;
    }

	return index;
}

/*
 * Reconstruct observation array from index number (i.e. dec to bin)
 * */
void HOTEL_ROBOT::getObservationFromIndex(int index, std::vector<bool>& obs) const{
    obs.resize(NumContainers); //also inits to 0/false
    int pos = NumContainers - 1;

    while(index != 0 && pos >= 0){
        if(index%2 != 0)
            obs[pos] = true;
        pos--;
        index /= 2;
    }

}

bool HOTEL_ROBOT::AssumeStatus(double p) const {
    bool assume = BinEntropyCheck(p) && p > 0.5;
    return assume;
}

/*
 * Return whether value p satisfies the entropy restriction < BIN_ENTROPY_LIMIT in a Bernoulli distribution
 * 
 * true if check is passed, meaning the value satisfies the uncertainty threshold.
 * 
 * */
bool HOTEL_ROBOT::BinEntropyCheck(double p) const {
    double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
    return (binaryEntropy <= HOTEL_ROBOT::BIN_ENTROPY_LIMIT);
}

/*
 * OUTPUT FUNCTIONS
 */

void HOTEL_ROBOT::DisplayBeliefs(const BELIEF_STATE& beliefState,
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

void HOTEL_ROBOT::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const HOTEL_ROBOT_STATE& rState = safe_cast<const HOTEL_ROBOT_STATE&>(state);
	ostr << endl;
    
    //Display worker state
    worker.DisplayState(rState.workerState, ostr);
    ostr << endl;
    
    //Hotel type belief
    ostr << "Hotel Type Prob.: A = ";
    ostr << std::setprecision(4) << rState.po_objects[rState.workerState.hotel].ProbT0;
    ostr << ", B = ";
    ostr << std::setprecision(4) << 1-rState.po_objects[rState.workerState.hotel].ProbT0;
    ostr << endl;
    ostr << endl;
    
    //Display container status    
	bool names = false;
	if(parts_str.size() == NumContainers) names = true;
	ostr << "Containers:" <<endl;
		ostr << "\t[";
		ostr << std::left << std::setw(8) << "Part No." << " | ";
		ostr << std::left << std::setw(16) << "Name" << " | ";
		ostr << std::left << std::setw(8) << "Storage" << " | ";
        ostr << std::left << std::setw(10) << "Assembled" << " | ";
        ostr << std::left << std::setw(8) << "Needed?" << " | ";
        ostr << std::left << std::setw(9) << "P(empty)" << " | ";
        ostr << std::left << std::setw(9) << "P(asm)" << " | ";
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
		//the actual storage status
		ostr << std::left << std::setw(8) << rState.containerStatus[c.id] << " | ";
        //assembly status
        ostr << std::left << std::setw(10) << rState.workerState.allParts[c.id].assembled << " | ";
        //needed?
        ostr << std::left << std::setw(8) << (c.needed? "Y" : "N") << " | ";
        //P. empty
        ostr << std::left << std::setw(9) << std::setprecision(4) << c.ProbEmpty << " | ";
        //P. assembled
        ostr << std::left << std::setw(9) << std::setprecision(4) << c.ProbAssembled << " | ";
        //Part cost
        ostr << std::left << std::setw(5) << c.cost << "]";
        
        ostr << endl;
	}

    ostr << endl;
}

//List of parts only
void HOTEL_ROBOT::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
	std::vector<bool> obs;
	getObservationFromIndex(observation, obs);
	
	ostr << "<-- Obs: (" << observation << ") ";
	
	for(auto p : obs)
        ostr << "[" << p << "]";
	
	ostr << endl;
}

void HOTEL_ROBOT::DisplayAction(int action, std::ostream& ostr) const
{

    ostr << "--> Action: ";        
	if(action >= A_BRING_PARTS){
		int part = action - A_BRING_PARTS;
        
        ostr << "Bring ";
        if(parts_str.size() == NumContainers)
            ostr << parts_str[part];
        else
            ostr << " Part " << part;
        ostr << endl;
	}
	else if(action == A_INSPECT_CONTAINER){
        ostr << " Inspect all containers" << endl;
        //int container = action - A_INSPECT_CONTAINER;
        //if(parts_str.size() == NumContainers)
            //ostr << "Inspect " << parts_str[container] << endl;
        //else
           // ostr << "Inspect " << container;
        
    }
    else if(action == A_INSPECT_OBJECT){
        ostr << "Inspect hotel" << endl;
    }
	else if (action == A_PERCEIVE){
        ostr << "Perceive hotel progress" << endl;
    }
}