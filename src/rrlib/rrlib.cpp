#include "rrlib.h"

RRLIB::RRLIB(const SIMULATOR& sim, const std::string& outputFile, RUN_PARAMS& RunParams, MCTS::PARAMS& SearchParams) : 
    POMDP(sim),
    OutputFile(outputFile.c_str()),
    runParams(RunParams),
    searchParams(SearchParams)
{
    if (runParams.autoExploration){
        searchParams.ExplorationConstant = sim.GetRewardRange();
    }
    MCTS::InitFastUCB(searchParams.ExplorationConstant);
            
    Results.undiscountedReturn = 0.0;
    Results.discountedReturn = 0.0;
    Results.discount = 1.0;
    
    searchParams.MaxDepth = POMDP.GetHorizon(runParams.accuracy, runParams.undiscountedHorizon);
    searchParams.NumSimulations = 1 << runParams.simDoubles;
    searchParams.NumStartStates = 1 << runParams.simDoubles;

    if (runParams.simDoubles + runParams.transformDoubles >= 0)
        searchParams.NumTransforms = 1 << (runParams.simDoubles + runParams.transformDoubles);
    else
        searchParams.NumTransforms = 1;
    searchParams.MaxAttempts = searchParams.NumTransforms * runParams.transformAttempts;


    //Initialize MCTS search class with planning POMDP + params
    mcts = new MCTS(POMDP, searchParams);
    
    //Keep track of total runtime
    timer_start = std::chrono::steady_clock::now();
}

void RRLIB::Init(){    

}

int RRLIB::PlanAction(int numSims, double timeout){    
    //cout << "MCTS select action" << endl;
    int action = mcts->SelectAction(); ///MCTS search

    if(searchParams.Verbose >= 1){
        cout << "Selected action: " << endl;
        POMDP.DisplayAction(action, cout);
    }

    return action;
}

int RRLIB::Update(int action, int observation, double reward, bool terminal){

    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;    
    bool outOfParticles = false;

    Results.Reward.Add(reward);
    Results.undiscountedReturn += reward;
    Results.discountedReturn += reward * Results.discount;
    Results.discount *= POMDP.GetDiscount();

    if (terminal)
    {
        cout << "Terminal state reached." << endl;
        Results.terminated = true;

        auto timer_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = timer_end - timer_start;
        Results.time = duration.count();

        return TERMINAL;
    }

    if(searchParams.Verbose >= 1){
        POMDP.DisplayObservation(dummyState, observation, cout);
        POMDP.DisplayReward(reward, cout);

    }
    //cout << "Updating..." << endl;
    outOfParticles = !mcts->Update(action, observation, reward);       
    //cout << "Finished update" << endl;
    
    if (outOfParticles)
        return OUT_OF_PARTICLES;

    return NON_TERMINAL;
}

RRLIB::RESULTS& RRLIB::getStatistics(){
    RESULTS r;
    
    auto timer_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = timer_end - timer_start;
    r.time = duration.count();
    r.discountedReturn = Results.discountedReturn;
    r.undiscountedReturn = Results.undiscountedReturn;
    r.terminated = Results.terminated;

    return r;
}

void RRLIB::InteractiveRun(){
}