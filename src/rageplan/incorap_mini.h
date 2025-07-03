/*
 * Incorap MWE v1.0
 *
 * Simplified model of intention recognition for worker assistance
 * 
 * By Juan Carlos Saborio
 * DFKI-NI, 2022
 
MWE:

states:
    goals/objects: screwdriver, multimeter, board_part
    worker activities: none, inspect, screw/unscrew, replace part, done
    worker poses: board, compartment
    worker elements: board, compartment

observations: activity, pose, outcome (OK, NOT_OK, NONE, FAIL) w/ accuracy

actions: observe, bring tool X.

rewards:
    +10 done
    +5 right tool/part
    -10 wrong/unwanted/no tool/part
    -2 missing part/tool
    -0.5 perceive
 
Scalable options:
    - More activities, more poses/elements, more tools.
 
 *
 * */

#ifndef INCORAPMWE_H
#define INCORAPMWE_H

#include "simulator.h"
#include "incorap_worker.h"
#include <cstring>
#include <sstream>

using std::string;
using std::vector;

struct INCORAPMWE_PARAMS : PROBLEM_PARAMS{    
    double perceive = 0.85; //Activity perceive accuracy
    
    double activation = -6.0; //IRE activation threshold
    double discount = 0.95; //POMDP discount
    double fDiscount = 0.5; //IRE feature discount    
    
    double entropy = 0.5; //PGS entropy
    double PGSAlpha = 10; //PGS scaling factor
    double transitionRate = 1.0; //"Learning rate" for values in f-table upon transitions
    
    INCORAPMWE_PARAMS() : perceive(0.85),
                    activation(-6.0), discount(0.95), fDiscount(0.5),                    
                    entropy(0.5), PGSAlpha(10){}
};

//TODO: Add array destructor
class INCORAPMWE_STATE : public STATE
{
public:
    //Worker state vars:
    int workerPose = 0;
    int workerActivity = 0;
    int workerOutcome = 0;
    vector<ELEMENT> elements;
    vector<OBJECT> objects;
    
    //Robot state:
    bool available; //Is robot running an errand or available to execute next action?
    struct PO_OBJECT{
        //True properties:
        int id;        
        int type;
        string type_str;
        bool needed;
        bool present;
        double cost; //Each tool/part has some cost (e.g. turns it takes to bring)
        
        //POMDP:
        int measured;
        int count;
        
        //Goal estimation
        double ProbNeeded;
        double LikelihoodNeeded;
        double LikelihoodNotNeeded;
        
        //IRE        
        bool active = true;        
    };    
    vector<PO_OBJECT> PO_Objects;
    
    void activateFeature(int feature, bool status);
};

class INCORAPMWE : public SIMULATOR{
public:

    INCORAPMWE(PROBLEM_PARAMS& problem_params, WORKER_PARAMS& wp);

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
    std::vector<FTABLE::F_ENTRY>& getInitialFTable() const {}
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
    /* Obs tuples are <activity, pose, outcome>
     * act = O_ACTIVITY - activity
     * pos = O_POSE - pose
     * out = O_NOTOK - outcome
     * linear index = act*n_pos*n_out + pos*n_out + out
    */
    enum
    {
        O_NOT_OK,
        O_OK,
        O_NONE,
        O_FAIL
    };
    int O_ACTIVITY, O_POSE; //Markers for activity and pose observations

    //Worker Poses
    enum{
        P_BOARD,
        P_COMPARTMENT
    };

    void InitGeneral();
    void Init_Demo1();
    
    bool BinEntropyCheck(double p) const; //Verify that p satisfies threshold
    
    /* Domain functions */
    double Perceive(const INCORAPMWE_STATE& state, std::vector<int>& obs) const; //Simulate observation tuple and store in "obs".  Return accuracy.
    int getObservationIndex(const std::vector<int>& obs) const;
	void getObservationFromIndex(int index, std::vector<int>& obs) const;
    /*
     * Domain information/representation
     * 
     */
    INCORAP_WORKER worker; //Initialized during POMDP construction with parameter list
    
    //Text descriptions:
    //Receive as parameters, e.g. from WORKER MDP, knowledge graph, domain description, etc.
    std::vector<string> objects_str, activities_str, poses_str, elements;
    vector<double> obj_costs;
	
	/*
	 * Receive current state and simulate worker action and effects
       Modifies the current POMDP state sample directly
	 * 
	 * Here we can use e.g. the Markov chain
	*/
	bool simulateWorker(INCORAPMWE_STATE& state) const;
    bool simulateWorker(int& action, int& pose, int& observation, vector<OBJECT>& objects, vector<ELEMENT>& elements) const;
    
    double PERCEIVE_ACC; //0 - 1 accuracy for perceive table
    
    // Relevance estimation parameters
    double BIN_ENTROPY_LIMIT; //0.5 or set to preference
    double ACTIVATION_THRESHOLD;
    double PGSAlpha; //PGS scaling factor
    
    //NumActions and NumObservations inherited from Simulator    
    int Size, NumPoses, NumElements, NumObjects, NumActivities, NumOutcomes, NumFeatures;
        
    int A_PERCEIVE, A_BRING; //Markers for the beginning of action group
    
    /*
     * PGS point distribution
     * 
     */    
    double PGS_bring_goal = 1;
    double PGS_bring_notgoal = -1;
    double PGS_uncertain = -0.5;

private:
    mutable MEMORY_POOL<INCORAPMWE_STATE> MemoryPool;
};

#endif
