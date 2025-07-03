/*
 * Robotic assistant for the Insect Hotel domain
 *
 * POMDP models the interaction between a robotic assistant and a worker and performs intention recogntion through (reverse) POMDP planning,
 * responding with appropriately timed actions.
 * 
 * By Juan Carlos Saborio
 * PBR @ DFKI-NI (2023)
 * Joint Lab KI & DS, UOS (2025)
 
POMDP definition:

Actions:
    - Observe hotel status (list of assembled parts)
    - Observe hotel type (0, 1)
    - Observe container status (empty, not empty)
    - Restock part

Observations:
    - Hotel status <- binary array of parts
    - Hotel type <- type1 | type2 (A or B, etc.)
    - Container status <- empty | not empty
    - Bring: OK|FAIL (bring success)
    
States:
    - Worker activity/state --> hotel assembly stage
    - Hotel type - P(T0)
    - List of storage containers:
        - Priority, Cost
        - P(empty)

Rewards:
    +5 finish truck
    -2 missing part
    -2 worker waits (from sim)
    -0.5 perceive
 
 *
 * */

#ifndef HOTEL_ROBOT_H
#define HOTEL_ROBOT_H

#include "simulator.h"
#include "hotel_worker.h"
#include <cstring>
#include <sstream>

using std::string;
using std::vector;

struct HOTEL_ROBOT_PARAMS : PROBLEM_PARAMS{    
    double perceive = 0.85; //Activity perceive accuracy
    double bring_success = 0.85;
    
    double activation = -6.0; //IRE activation threshold
    double discount = 0.95; //POMDP discount
    double fDiscount = 0.5; //IRE feature discount    
    
    double entropy = 0.5; //PGS entropy
    double PGSAlpha = 10; //PGS scaling factor
    double transitionRate = 1.0; //"Learning rate" for values in f-table upon transitions
    
    HOTEL_ROBOT_PARAMS() : perceive(0.85),
                    activation(-6.0), discount(0.95), fDiscount(0.5),                    
                    entropy(0.5), PGSAlpha(10){}
};

//TODO: Add array destructor
class HOTEL_ROBOT_STATE : public STATE
{
public:
    //Worker state
    HOTEL_STATE workerState; //Maintains <AOR> for worker simulation   
    int NumTypes; //How many types of objects. Default = 2 
            
    //Robot state:
    bool available; //Is robot running an errand or available to execute next action?
    
    //PO object properties, i.e. Hotels
    struct PO_OBJ{
        double ProbT0;
        double LikelihoodT0;
        double LikelihoodNotT0;
        bool assumedType;
    };
    vector<PO_OBJ> po_objects;
    
    //PO container/part properties
    struct ELEMENT{
        //FO
        int id;
        string name = "";
        int capacity; //Not used
        double cost;
        int priority;
        
        int measured, count;
        
        bool needed;
        bool low; //Not used
        bool empty;
        
        //Estimated
        double LikelihoodEmpty;
        double LikelihoodNotEmpty;
        double ProbEmpty;

        double LikelihoodAssembled;
        double LikelihoodNotAssembled;
        double ProbAssembled;
        
        //IRE
        bool active = true;
    };
    vector<ELEMENT> containers; //List of storage containers with parts
    vector<int> storage; //True amount of objs. in containers
    vector<bool> containerStatus; //True status of storage containers
    vector<double> p_empty;
        
    HOTEL_ROBOT_STATE() {}
    ~HOTEL_ROBOT_STATE() {
        containers.clear();
        storage.clear();
        containerStatus.clear();
        p_empty.clear();
        po_objects.clear();
    }

    //For F-table
    void activateFeature(int feature, bool status);
};

class HOTEL_ROBOT : public SIMULATOR{

public:

    HOTEL_ROBOT(PROBLEM_PARAMS& problem_params, HOTEL_PARAMS& ap);    

    //Core Simulator functions
    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
                      int& observation, double& reward) const;

    /*** PGS functions ***/
    //Uses regular POMCP Step
    bool StepNormal(STATE& state, int action,
                    int& observation, double& reward) const;
    //Step with PGS rewards
    bool StepPGS(STATE& state, int action,
                 int& observation, double& reward) const;
    // Simple Step (transition only)
    bool SimpleStep(STATE& state, int action) const;
    //PGS Rollout policy
    void GeneratePGS(const STATE& state, const HISTORY& history,
                     std::vector<int>& legal, const STATUS& status) const;

    //Compute PGS value
    double PGS(STATE& state) const;
    double PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const;  //PGS for rollouts

    ///// Incremental refinement /////
    //std::vector<FTABLE::F_ENTRY>& getInitialFTable() const {}
    void initializeFTable(FTABLE& ftable) const;

    /********************************/

    /* Virtual functions from Simulator class */
    void GenerateLegal(const STATE& state, const HISTORY& history,
                       std::vector<int>& legal, const STATUS& status) const;
    void PGSLegal(const STATE& state, const HISTORY& history,
                  std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
                           std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
                           int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
                                std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;    

protected:

    //Observations
    int O_ACTIVITY, O_OBJECT, O_RESULT; //Markers for APO observations

    void InitGeneral();
    void Init_Demo1();
    
    bool BinEntropyCheck(double p) const; //Verify that p satisfies threshold
    
    /* Domain functions */
    bool AssumeStatus(double p) const; //Use current prob. estimation to make rough sim. assumption
    //Get worker activity/progress. For Insect Hotel it means list of parts
    double Perceive(const HOTEL_ROBOT_STATE& state, std::vector<bool>& obs) const;
    //Convert obs array into int
    int getObservationIndex(std::vector<bool>& obs) const;
    //Convert observation int to array
	void getObservationFromIndex(int index, std::vector<bool>& obs) const;

    //Get container status (empty, not empty)
    double InspectContainer(const HOTEL_ROBOT_STATE& state, int container, int& obs) const;
    //Alternative function: inspect ALL containers at once, similar to perceive worker
    double InspectAllContainers(const HOTEL_ROBOT_STATE& state, std::vector<bool>& obs) const;
    //Get hotel type (A or B)
    double InspectObject(const HOTEL_ROBOT_STATE& state, int& obs) const;
    
    
    /*
     * Domain information/representation
     * 
     */
    HOTEL_WORKER worker; //Worker simulator. Initialized during POMDP construction with parameter list
    double FTableTransitionRate = 1.0;

    //Text descriptions:
    //Receive as parameters, e.g. from WORKER MDP, knowledge graph, domain description, etc.
    std::vector<string> objects_str, activities_str, poses_str, outcomes_str, parts_str;
    vector<double> part_cost;
    vector<int> part_priority, initial_storage;

    //TODO: switch to std::set
    vector<int> generalParts; //Only the parts that belong to all hotels
    vector<vector<int>> uniqueParts; //For each hotel type, list their unique parts
    bool contains(vector<int> v, int element) const; //Find if element is part of one of these lists
    	
	/*
	 * Receive current state and simulate worker action and effects
       Modifies the current POMDP state sample directly
	 * 
	 * Here we can use e.g. the Markov chain
	*/
	bool simulateWorker(HOTEL_STATE& state, vector<bool>& storage, double& workerReward) const;
        
    double PERCEIVE_ACC; //0 - 1 accuracy for perceive table
    double P_BRING_SUCCESS ;//Simulate grasping or manipulation errors
    
    // Relevance estimation parameters
    double BIN_ENTROPY_LIMIT; //0.5 or set to preference
    double ACTIVATION_THRESHOLD;
    double PGSAlpha; //PGS scaling factor
    
    //NumActions and NumObservations inherited from Simulator    
    int NumActivities, NumPoses, NumOutcomes;
    int NumContainers, NumObjects, NumTypes;
    int NumFeatures;
        
    int A_PERCEIVE, A_BRING_PARTS; //Markers for the beginning of action group
    int A_INSPECT_OBJECT, A_INSPECT_CONTAINER; //Markers for the beginning of additional actions
    
    int O_FAIL = 0, O_OK = 1; //local observations
    int REFILL_AMOUNT = 1; //TODO: set externally?
    
    /*
     * PGS point distribution
     * 
     */    
    double PGS_goal = 1;
    double PGS_notgoal = -1;
    double PGS_uncertain = -0.5;

private:
    friend class RRLIB_HOTEL;
    mutable MEMORY_POOL<HOTEL_ROBOT_STATE> MemoryPool;
    int boolValueTable[10] = {1,2,4,8,16,32,64,128,256,512};
};

#endif
