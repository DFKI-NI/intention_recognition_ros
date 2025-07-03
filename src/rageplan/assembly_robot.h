/*
 * Robotic assistant for the assembly task
 * Based on the Smart Assembly use case in InCoRAP
 *
 * POMDP models the interaction between a robotic assistant and a worker and performs intention recogntion,
 * responding with appropriately timed actions.
 * 
 * By Juan Carlos Saborio
 * DFKI-NI, 2023
 
POMDP definition:

Actions:
    - Observe worker / activiies
    - Bring/refill part X
    - Bring glue type Y

Observations:
    - Observe: <APO>
    - Bring: OK|FAIL
    
states:
    - worker APO/state
    - list of storage containers:
        - Capacity, Cost
        - Current Quantity, Status: OK|Low|Empty, P(empty)?

rewards:
    +5 finish truck
    -5 wrong glue
    -2 missing part / worker waits    
    -0.5 perceive
 
Scalable options:
    - No. of trucks, no. of types, no. of parts/containers
 
 *
 * */

#ifndef ASSEMBLY_ROBOT_H
#define ASSEMBLY_ROBOT_H

#include "simulator.h"
#include "assembly_worker.h"
#include <cstring>
#include <sstream>

using std::string;
using std::vector;

struct OBSERVATION_TRIPLE{
    int activity;
    int pose;
    int outcome;
};

struct ASSEMBLY_ROBOT_PARAMS : PROBLEM_PARAMS{    
    double perceive = 0.85; //Activity perceive accuracy
    
    double activation = -6.0; //IRE activation threshold
    double discount = 0.95; //POMDP discount
    double fDiscount = 0.5; //IRE feature discount    
    
    double entropy = 0.5; //PGS entropy
    double PGSAlpha = 10; //PGS scaling factor
    double transitionRate = 1.0; //"Learning rate" for values in f-table upon transitions
    
    ASSEMBLY_ROBOT_PARAMS() : perceive(0.85),
                    activation(-6.0), discount(0.95), fDiscount(0.5),                    
                    entropy(0.5), PGSAlpha(10){}
};

//TODO: Add array destructor
class ASSEMBLY_ROBOT_STATE : public STATE
{
public:
    //Worker state
    ASSEMBLY_STATE workerState; //Includes APO, true storage containers and list of trucks
    bool missing_glue;
    int NumTypes;
            
    //Robot state:
    bool available; //Is robot running an errand or available to execute next action?
    
    //PO truck properties
    struct PO_OBJ{
        double ProbT0;
        double LikelihoodT0;
        double LikelihoodNotT0;
        bool assumedType;
    };
    vector<PO_OBJ> po_truck;
    
    //PO container/part properties
    struct ELEMENT{
        //FO
        int id;
        string name = "";
        int capacity;
        double cost;
        int priority;
        
        int measured, count;
        
        bool needed;
        bool low;
        bool empty;
        
        //Estimated
        double LikelihoodEmpty;
        double LikelihoodNotEmpty;
        double ProbEmpty;
        
        //IRE
        bool active = true;
    };
    vector<ELEMENT> containers;
    vector<int> storage; //True amount of objs. in containers
        
        
    ASSEMBLY_ROBOT_STATE() {}
    ~ASSEMBLY_ROBOT_STATE() {
        containers.clear();
        storage.clear();
    }
    void activateFeature(int feature, bool status);
};

class ASSEMBLY_ROBOT : public SIMULATOR{
public:

    ASSEMBLY_ROBOT(PROBLEM_PARAMS& problem_params, ASSEMBLY_PARAMS& ap);    

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

    /*** ROBOT INTEGRATION FUNCTION ***/
    //void getAORList(vector<string>& act, vector<string>& obj, vector<string>& res) const; //List all worker activities

protected:

    //Observations
    int O_ACTIVITY, O_POSE, O_OUTCOME; //Markers for APO observations

    void InitGeneral();
    void Init_Demo1();
    
    bool BinEntropyCheck(double p) const; //Verify that p satisfies threshold
    
    /* Domain functions */
    double Perceive(const ASSEMBLY_ROBOT_STATE& state, OBSERVATION_TRIPLE& obs) const; //Simulate worker observation tuple and store in "obs".  Return accuracy.        
    int getObservationIndex(OBSERVATION_TRIPLE& obs) const;
	void getObservationFromIndex(int index, OBSERVATION_TRIPLE& obs) const;

    // Beyond INCORAP
    double InspectContainer(const ASSEMBLY_ROBOT_STATE& state, int container, int& obs) const;
    double InspectObject(const ASSEMBLY_ROBOT_STATE& state, int& obs) const;
    
    
    /*
     * Domain information/representation
     * 
     */
    ASSEMBLY_WORKER worker; //Initialized during POMDP construction with parameter list
    double FTableTransitionRate = 1.0;

    //Text descriptions:
    //Receive as parameters, e.g. from WORKER MDP, knowledge graph, domain description, etc.
    std::vector<string> objects_str, activities_str, poses_str, outcomes_str, parts_str;
    vector<double> part_cost;
    vector<int> part_priority, initial_storage;
    	
	/*
	 * Receive current state and simulate worker action and effects
       Modifies the current POMDP state sample directly
	 * 
	 * Here we can use e.g. the Markov chain
	*/
	bool simulateWorker(ASSEMBLY_STATE& state, vector<int>& storage, double& workerReward) const;    
        
    double PERCEIVE_ACC; //0 - 1 accuracy for perceive table
    
    // Relevance estimation parameters
    double BIN_ENTROPY_LIMIT; //0.5 or set to preference
    double ACTIVATION_THRESHOLD;
    double PGSAlpha; //PGS scaling factor
    
    //NumActions and NumObservations inherited from Simulator    
    int NumActivities, NumPoses, NumOutcomes;
    int NumContainers, NumObjects, NumTypes;
    int NumFeatures;
        
    int A_PERCEIVE, A_BRING_GLUE, A_BRING_PARTS; //Markers for the beginning of action group
    int A_INSPECT_TRUCK, A_INSPECT_CONTAINER; //Markers for the beginning of complementary actions
    
    int O_FAIL = 0, O_OK = 1; //local observations
    int REFILL_AMOUNT = 5; //TODO: set externally?
    
    /*
     * PGS point distribution
     * 
     */    
    double PGS_goal = 1;
    double PGS_notgoal = -1;
    double PGS_uncertain = -0.5;

private:
    friend class RRLIB_ASSEMBLY;
    mutable MEMORY_POOL<ASSEMBLY_ROBOT_STATE> MemoryPool;
};

#endif
