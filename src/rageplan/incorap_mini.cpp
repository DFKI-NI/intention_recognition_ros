#include "incorap_mini.h"
#include "utils.h"

#include <iomanip>
#include <fstream>

using namespace std;
using namespace UTILS;

void INCORAPMWE_STATE::activateFeature(int feature, bool status){
    if(feature >= 0 && feature < PO_Objects.size())
        PO_Objects[feature].active = status;
}


/* Build Ftable mapping every action to its affected feature/object */
/*
    In Mobipick, actions afforded by objects are: pick object, identify object
 */
void INCORAPMWE::initializeFTable(FTABLE& ftable) const{
// Add all corresponding action-feature pairs:
//    ftable.addEntry(action, feature);
    
    //Add all brings
    for(int f = 0; f < NumFeatures; f++)
        ftable.addEntry(A_BRING + f, f);
    
    //Add all perceives
    /*
    for(int f = 0; f < NumFeatures; f++)
        ftable.addEntry(A_PERCEIVE + f, f);
    */
    ftable.setACTIVATION_THRESHOLD(INCORAPMWE::ACTIVATION_THRESHOLD); ///Problem specific activation threshold
    ftable.setNumActions(NumActions);
    ftable.setNumFeatures(NumFeatures);

//	cout << "Done" << endl;
}

INCORAPMWE::INCORAPMWE(PROBLEM_PARAMS& problem_params, WORKER_PARAMS& wp){
    //Initialize local worker simulator
    worker.setProblem(wp);
    worker.getProblemDescription(objects_str, activities_str, poses_str, elements);
    this->obj_costs = wp.obj_costs;
    
    INCORAPMWE_PARAMS& params = safe_cast<INCORAPMWE_PARAMS&>(problem_params);

    ///Read parameters from struct
    Discount = params.discount;  //MCTS discount
    fDiscount = params.fDiscount;  //Feature-value discount.  Probably also problem dependent.

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
    NumObjects = objects_str.size();
    NumFeatures = NumObjects;
    NumActions = NumObjects + 1; //Perceive + bring each object
    NumActivities = activities_str.size();
    NumPoses = poses_str.size();
    NumOutcomes = 4; //NOT OK, OK, NONE, FAIL
    NumObservations = NumActivities*NumPoses*NumOutcomes;
    NumElements = NumPoses; //MWE feature
    
    RewardRange = 30; //NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts
    
    //Set action markers
    A_PERCEIVE = 0;
    A_BRING = 1;
            
    InitGeneral();
}

void INCORAPMWE::InitGeneral()
{    
    RandomSeed(0);
    //Add any more problem-specific init parameters
}

STATE* INCORAPMWE::Copy(const STATE& state) const
{        
    const INCORAPMWE_STATE& incorapState = safe_cast<const INCORAPMWE_STATE&>(state);
    INCORAPMWE_STATE* newstate = MemoryPool.Allocate();
    *newstate = incorapState;
    return newstate;
}

void INCORAPMWE::Validate(const STATE& state) const
{
    const INCORAPMWE_STATE& incorapState = safe_cast<const INCORAPMWE_STATE&>(state);    
}

STATE* INCORAPMWE::CreateStartState() const
{
    
    INCORAPMWE_STATE* incorapState = MemoryPool.Allocate();
    
    //Init worker state variables
	incorapState->workerActivity = 0; //Start in "none"
	incorapState->workerPose = 0; //Start at board I guess?
	incorapState->workerOutcome = O_NONE;
    
    incorapState->objects.clear();
	for(auto o : objects_str){
		OBJECT obj;
		obj.name = o;
		obj.present = false;

		incorapState->objects.push_back(obj);
	}

	incorapState->elements.clear();
	for(auto e : elements){
		ELEMENT elem;
		elem.name = e;
		elem.OK = Bernoulli(0.5); //Random true/false
		elem.inspected = false;
		incorapState->elements.push_back(elem);
	}
    
    //Init robot state variables
    incorapState->available = true;
    //Clear array
    incorapState->PO_Objects.clear();

    //Create objects    
    for (int i=0; i < NumObjects; i++){
        INCORAPMWE_STATE::PO_OBJECT o;
        
        //Ground truth:
        o.id = i;
        o.type = i; //TODO: Fix for various objects of same type
        o.type_str = objects_str[i];
        o.needed = false;
        o.present = false;
        o.cost = obj_costs[i]; //Random(4,6);
    
        //Probabilistic properties:
        o.measured = 0;
        o.count = 0;
        
        o.ProbNeeded = 0.5;
        o.LikelihoodNeeded = 1.0;
        o.LikelihoodNotNeeded = 1.0;            
        
        o.active = true;
        
        //Add object
        incorapState->PO_Objects.push_back(o);
    }
    
    return incorapState;
}

void INCORAPMWE::FreeState(STATE* state) const
{
    INCORAPMWE_STATE* incorapState = safe_cast<INCORAPMWE_STATE*>(state);
    MemoryPool.Free(incorapState);
}

bool INCORAPMWE::Step(STATE& state, int action,
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
bool INCORAPMWE::StepPGS(STATE& state, int action,
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

bool INCORAPMWE::simulateWorker(INCORAPMWE_STATE& state) const{
    //Simulate transition    
	bool done = worker.Step(state.workerActivity, state.workerPose, state.workerOutcome, state.objects, state.elements);
    //Simulate/populate non-deterministic outcomes
    
    worker.generateOutcomes(state.workerActivity, state.workerPose, state.workerOutcome, state.objects, state.elements);
       
    //Set needed:
    int o_needed = -1;
    if(state.workerActivity == worker.A_SCREW){
        o_needed = worker.O_SCREW;
    }
    else if(state.workerActivity == worker.A_INSPECT && state.workerPose == worker.P_BOARD){
        o_needed = worker.O_MULTI;
    }
    else if(state.workerActivity == worker.A_REPLACE){
        o_needed = worker.O_PART;
    }

    if(o_needed >= 0 && o_needed < NumObjects){
        //cout << objects_str[o_needed] << " is needed!" << endl; 
        if(!state.PO_Objects[o_needed].present) //Object is needed never needed AGAIN if already present
            state.PO_Objects[o_needed].needed = true;
    }
    
    return done;
}


/*
 * Regular step function.  Simulates transition from state with action and returns observation and reward
*/
bool INCORAPMWE::StepNormal(STATE& state, int action,
                        int& observation, double& reward) const
{    
    reward = 0;
    observation = O_NONE;    
    
    /* Reward distribution
		+ 10 done
		+ 10 right tool/part
		- 10 wrong/unwanted/
		- 5 missing tool/part
    */
    double reward_perceive = -0.5;
    int terminal_success = 10;
    int reward_rightGoal = 5;
    int reward_wrongGoal = -10;
	int reward_missing = -2; //Reward FAIL
    bool terminal = false;
    //if(action > NumActions){
    //    cout << "Starting step function, action no." << action << endl;
    //    DisplayAction(action, cout);
    //     DisplayState(state, cout);
    //}
    
	
	INCORAPMWE_STATE& incorapState = safe_cast<INCORAPMWE_STATE&>(state);
        
    ///Process action and simulate results    
	/*     
		Actions are a sequence of numbers dictated by the following ranges:        
		A_PERCEIVE = 0
		A_BRING = 1 + obj. number
    */
    
//    cout << "Step -- ";
//    DisplayAction(action, cout);
		
//	cout << "Step with a = " << action << endl;
        
    // Bring obj. number
    double cost = 0; //get cost of bringing object
    if(action >= A_BRING){
        observation = O_NONE;
        reward = 0; //reward_wrongGoal; //works both for wrong tool and wrong time
        int obj = action - A_BRING; //Robot chose this object
                            
        //Simulate cost/delay of bringing object
        int timeSteps = round(incorapState.PO_Objects[obj].cost);
        for(int time=0; time < timeSteps; time++){
            //Move worker one step
            simulateWorker(incorapState);
            //Each time add up the punishment for all needed+missing objects
            for(auto o : incorapState.PO_Objects)
                if(o.needed && !o.present) reward += reward_missing;
        }
        //reward -= timeSteps; //Add punishment for absence
        
        if(!incorapState.PO_Objects[obj].present){ //Don't bring twice
            incorapState.PO_Objects[obj].present = true;
            //worker->makeObjectAvailable(incorapState.workerState, obj);
            incorapState.objects[obj].present = true;
            
            //Assume the object is not needed anymore as soon as it is brought over
            //In this problem needed = accepted by the worker!
            if(incorapState.PO_Objects[obj].needed){
                reward += reward_rightGoal;
                //incorapState.PO_Objects[obj].needed = false; //DO not reset needed
                
                //Reinit prob estimation -- Set to 0 because it is not needed anymore?
                incorapState.PO_Objects[obj].ProbNeeded = 0;
                incorapState.PO_Objects[obj].LikelihoodNeeded = 1.0;
                incorapState.PO_Objects[obj].LikelihoodNotNeeded = 1.0;
            }
            else{
                // Let the robot bring whatever as long as it also brings needed tools/parts?
                reward += reward_wrongGoal;
            }
        }
        else
            reward += reward_wrongGoal;
    }
    
    ///Perceive
    else if(action == A_PERCEIVE){
        reward = reward_perceive;
		std::vector<int> obs; //Observation tuple
		double efficiency = Perceive(incorapState, obs);
        observation = getObservationIndex(obs);

        int activity = obs[0];
        int pose = obs[1];
        int outcome = obs[2];
                        
        // Failure indicates a tool or part was missing, otherwise outcome would be OK/NOT OK
        if(outcome == worker.O_FAIL)
            reward += reward_missing;

        //TODO: Receive these dependencies in some type of mapping, e.g. in MDP.
        // For each o in activity.requires, update prob.
        int o;
        bool updateNeeded = false;
        if(activity == worker.A_SCREW){
            o = worker.O_SCREW;
            updateNeeded = true;          
        }
        else if(activity == worker.A_INSPECT && pose == worker.P_BOARD){
            o = worker.O_MULTI;
            updateNeeded = true;
        }
        else if(activity == worker.A_REPLACE){
            o = worker.O_PART;
            updateNeeded = true;
        }

        //Update corresponding object probabilities
        if(updateNeeded){
            incorapState.PO_Objects[o].LikelihoodNeeded *= efficiency;
            incorapState.PO_Objects[o].LikelihoodNotNeeded *= 1 - efficiency;
            double denom = (0.5 * incorapState.PO_Objects[o].LikelihoodNeeded) + (0.5 * incorapState.PO_Objects[o].LikelihoodNotNeeded);
            incorapState.PO_Objects[o].ProbNeeded = (0.5 * incorapState.PO_Objects[o].LikelihoodNeeded) / denom;
        }
    }
    
    ///Simulate worker transition and outcomes
	terminal = simulateWorker(incorapState);
    
    // -1 For each object needed and not present
    /*for(auto o : incorapState.PO_Objects){
        if(o.needed && !o.present)
            reward += reward_missing;
    }*/
    
    if(terminal){
		reward += terminal_success;
	}
	
    return terminal;
}


// Local transformations: change worker activity or pose
// TODO: consider using a diff. belief approximation for this task, no particle filter -> no local transformation necessary
// TODO: try this --LOCAL MOVE changes elem OK status, moves worker, checks observation
bool INCORAPMWE::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const{
    
    INCORAPMWE_STATE& incorapState = safe_cast<INCORAPMWE_STATE&>(state);
    
    /*if(Bernoulli(0.5))
		incorapState.workerPose = Random(NumPoses);
	else
		incorapState.workerActivity = Random(NumActivities);*/
    
    int elem = Random(NumElements);
    incorapState.elements[elem].OK = !incorapState.elements[elem].OK;
    simulateWorker(incorapState);
    
    int action = history.Back().Action;

    //IDENTIFY: validate id observations
    if (action == A_PERCEIVE){
        int realObs = history.Back().Observation;
        
        std::vector<int> obsT;
        
		//Get new observation
		Perceive(incorapState, obsT);
        int newObs = getObservationIndex(obsT);
        obsT.clear();

        //If observations do not match, reject
        if (newObs != realObs){            
            return false;
        }
    }

    return true;
}

/* Fast PGS for Rollout policy
 * Simplified PGS point count by using only the specific action changes
 */
double INCORAPMWE::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
    double points = 0.0;
	double oldpoints = 0.0;
	
	//1. Cast to cellarstate
	INCORAPMWE_STATE& incorapState = safe_cast<INCORAPMWE_STATE&>(state);
	INCORAPMWE_STATE& oldIncorapState = safe_cast<INCORAPMWE_STATE&>(oldstate);
	
	int obj = -1;
    
    //Outcome of 'bring' action is either +1 (needed and not present) or else -1 
	if(action >= A_BRING){
		obj = action - A_BRING;
		if (incorapState.PO_Objects[obj].needed && !oldIncorapState.PO_Objects[obj].present){
                points += PGS_bring_goal;
		}
		else
            points += PGS_bring_notgoal;
	}

	//IF estimating points for reducing uncertainty, add oldpoints and current points. ATM, oldpoints = 0 always.

	//Update difference for current bottle
	double result = oldpgs - oldpoints + points;

	return result;
}

/*
 * PGS Point scoring for the INCORAP domain
 */
double INCORAPMWE::PGS(STATE& state) const
{    
    double points = 0.0;	
	
	//1. Cast to cellarstate
	INCORAPMWE_STATE& incorapState = safe_cast<INCORAPMWE_STATE&>(state);
	
	//2. Evaluate objects: +1 if needed and present, -1 if needed and not present OR if present and not needed
	for(auto o : incorapState.PO_Objects){
        if(o.needed){
            if(o.present) points += PGS_bring_goal;
            else points += PGS_bring_notgoal;
        }
        else{
            if(o.present) points += PGS_bring_notgoal;
        }
    }
	
	return points;
}

// PGS Rollout policy
void INCORAPMWE::GeneratePGS(const STATE& state, const HISTORY& history,
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

void INCORAPMWE::PGSLegal(const STATE& state, const HISTORY& history,
                      vector<int>& legal, const STATUS& status) const
{    
    const INCORAPMWE_STATE& incorapState = safe_cast<const INCORAPMWE_STATE&>(state);
    
    //Allow perceive
    legal.push_back(A_PERCEIVE);
    
    //Selectively allow bringing objects
    for(int o=0; o < NumObjects; o++){
        if(incorapState.PO_Objects[o].active)
            legal.push_back(A_BRING + o);
    }
}


void INCORAPMWE::GenerateLegal(const STATE& state, const HISTORY& history,
                           vector<int>& legal, const STATUS& status) const
{  
	//For now add ALL actions: perceive and bring 1...N
    for(int i=0; i<NumActions; i++)
        legal.push_back(i);
}

/*
  Preferred actions?
*/
void INCORAPMWE::GeneratePreferred(const STATE& state, const HISTORY& history,
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
double INCORAPMWE::Perceive(const INCORAPMWE_STATE& state, std::vector<int>& obs) const{
    double efficiency = PERCEIVE_ACC; //Maybe calculate based on state properties
    double pm_efficiency = 1-efficiency;
	
    obs.clear();
    //With given efficiency, observe true activity,pose,outcome	
	int activity = state.workerActivity;
	int pose = state.workerPose;
    int outcome = state.workerOutcome;
	
    //with p - efficiency, randomize
	if(Bernoulli(pm_efficiency)){
        //Otherwise randomize one element
        int r = Random(3);
		switch(r){
            case 0:
                activity = Random(NumActivities);
                break;
            case 1:
                pose = Random(NumPoses);
                break;
            case 2:
                outcome = Random(NumOutcomes);
                break;
		}
	}
	
	obs.push_back(activity);
	obs.push_back(pose);
	obs.push_back(outcome);
	
	return efficiency;
}

/* Obs tuples are <activity, pose, outcome>
 * linear index = act*n_pos*n_out + pos*n_out + out
 */
int INCORAPMWE::getObservationIndex(const std::vector<int>& obs) const{
	int act = obs[0];
	int pos = obs[1];
	int out = obs[2];
	
	int index = act * NumPoses * NumOutcomes + pos * NumOutcomes + out;
	return index;
}

/*
 * Reconstruct observation tuple from index number
 * */
void INCORAPMWE::getObservationFromIndex(int index, std::vector<int>& obs) const{
	obs.clear();
    int local_index = 0;
	int NPNO = NumPoses*NumOutcomes;
    bool found = false;
	
	for(int a=0; a < NumActivities && !found; a++){
		for(int p=0; p < NumPoses && !found; p++){
			for(int o=0; o < NumOutcomes && !found; o++){
				local_index = a*NPNO + p*NumOutcomes + o;
				if(local_index == index){
					obs.push_back(a);
					obs.push_back(p);
					obs.push_back(o);
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
bool INCORAPMWE::BinEntropyCheck(double p) const {
    double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
    return (binaryEntropy <= INCORAPMWE::BIN_ENTROPY_LIMIT);
}

/*
 * OUTPUT FUNCTIONS
 */

void INCORAPMWE::DisplayBeliefs(const BELIEF_STATE& beliefState,
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

void INCORAPMWE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const INCORAPMWE_STATE& incorapState = safe_cast<const INCORAPMWE_STATE&>(state);    	
	ostr << endl;
    ostr << "WORKER: ";
    //worker->DisplayState(incorapState.workerState, ostr);
    //ostr << "APO: " << incorapState.workerActivity << ", " <<incorapState.workerPose<< ", " << incorapState.workerOutcome << endl;
    ostr << "<Activity: " << activities_str[incorapState.workerActivity] << " | ";
	ostr << "Pose: " << poses_str[incorapState.workerPose] << " | ";	
	ostr << "Outcome: ";
	switch(incorapState.workerOutcome){
		case O_NOT_OK:
			ostr << "Not OK";
			break;
		case O_OK:
			ostr << "OK";
			break;
		case O_NONE:
			ostr << "N/A";
			break;
		case O_FAIL:
			ostr << "Fail";
			break;
	}
	ostr << ">" << endl;
	
	ostr << "Objects: " << endl;
	int o_c = 0;
	for(auto o : incorapState.objects)
		ostr << "\t[" << o_c++ << "] " << o.name << " (" << (o.present ? "Y" : "N") << ")" << endl;

	ostr << "Elements: " << endl;
	for(auto e : incorapState.elements)
		ostr << "\t" << e.name << " - " << (e.OK? "OK" : "Not OK") << " - Inspected? " << (e.inspected? "Y" : "N") << endl;    
    
	ostr << "GOALS:" << endl;
    for(auto o : incorapState.PO_Objects){
		ostr << "\t" << o.id << ": " << o.type_str << ", ";
		
        ostr << "P(needed) = " << o.ProbNeeded << ", ";
        
		ostr << "Needed = ";
        if(o.needed) ostr << "Y";
		else ostr << "N";
        
        ostr << ", Cost = " << o.cost;
        
		ostr << endl;
    }

    ostr << endl;
}

//<activity, pose, outcome>
void INCORAPMWE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
	std::vector<int> obs;
	getObservationFromIndex(observation, obs);
	
	ostr << "O = <";
	
	//Activity
	ostr << activities_str[obs[0]] << ", ";
	
	//Pose
	ostr << poses_str[obs[1]] << ", ";
	
	//Outcome
    switch (obs[2])
    {
        case O_NONE:
            ostr << "NONE";
            break;
        case O_OK:
            ostr << "OK";
            break;
        case O_NOT_OK:
            ostr << "NOT OK";
            break;
        case O_FAIL:
            ostr << "FAIL";
            break;
    }
	
	ostr << ">" << endl;
}

void INCORAPMWE::DisplayAction(int action, std::ostream& ostr) const
{

    ostr << "A: ";
    if (action == A_PERCEIVE){
        ostr << "Perceive" << endl;
    }
    else{
		int obj = action - A_BRING;
        ostr << "Bring " << objects_str[obj] << endl;
	}
}

