#include "maintenance_worker.h"
#include <iomanip>

/*
	MWE/default constructor
*/
MAINTENANCE_WORKER::MAINTENANCE_WORKER(){
	createMWE();
}

/*
	Main constructor. Receives parameter struct and initializes everything.
 */
MAINTENANCE_WORKER::MAINTENANCE_WORKER(MAINTENANCE_PARAMS& params){
    this->elements_str = params.elements;
    this->parts_str = params.parts;
	this->element_priority = params.element_priority;
	this->part_costs = params.part_costs;
	this->expertise = params.expertise;
	
	N_PARTS = part_costs.size();
	N_ELEMENTS = element_priority.size();
}

void MAINTENANCE_WORKER::createMWE(){
    this->elements_str = {"Module 1", "Module 2", "Module 3", "Module 4", "Module 5", "Module 6"};
    this->parts_str = {"Part A", "Part B", "Part C", "Part D"};
	this->element_priority = {1,2,3,3,2,1};
	this->part_costs = {3,2,3,2};
	this->expertise = 0.95;
	
	N_PARTS = part_costs.size();
	N_ELEMENTS = element_priority.size();
	//srand(time(NULL));
	
	//currentState.copy(createStartState());
}

//Set problem externally (e.g. from POMDP assistant)
void MAINTENANCE_WORKER::setProblem(MAINTENANCE_PARAMS& params){
    this->elements_str = params.elements;
    this->parts_str = params.parts;
	this->element_priority = params.element_priority;
	this->part_costs = params.part_costs;
	this->expertise = params.expertise;
	N_PARTS = part_costs.size();
	N_ELEMENTS = element_priority.size();
}

MAINTENANCE_STATE* MAINTENANCE_WORKER::createStartState(){
	MAINTENANCE_STATE * ms = new MAINTENANCE_STATE();
	
	ms->parts.clear();
	bool names = false;
	if(parts_str.size() == part_costs.size())
		names = true;
	for(int p = 0; p < N_PARTS; p++){
		PART p_;
		if(names) p_.name = parts_str[p];
		p_.cost = part_costs[p];

		ms->parts.push_back(p_);
	}
		
	ms->activity = A_NONE; //Start in "none"
	ms->pose = P_NONE; //Start in pose "none"
	ms->outcome = O_OK;
	ms->partAvailable = -1;
	
	ms->elements.clear();
	names = false;
	if(elements_str.size() == element_priority.size())
		names = true;
	for(int e = 0; e < N_ELEMENTS; e++){
		ELEMENT elem;
		
		if(names) elem.name = elements_str[e];
		elem.OK = Bernoulli(0.5); //Random true/false
		elem.inspected = false;
		elem.part = Random(N_PARTS); //Assign random part
		elem.priority = element_priority[e];
		
		ms->elements.push_back(elem);
	}
		
	return ms;
}

double MAINTENANCE_WORKER::Bernoulli(double p) const{    
    return rand() < p * RAND_MAX;
}

//Step and outcome functions w/o state
bool MAINTENANCE_WORKER::Step(MAINTENANCE_STATE& state){
	bool terminal = true;
	
	//See if terminal conditions are met
	for(auto e : state.elements)
		terminal &= (e.OK && e.inspected);
	
	if(terminal){
		state.activity = A_NONE;
		return true;
	}
		
	//Neutral activity
	if(state.activity == A_NONE){
		
		//Switch to inspect
		state.activity = A_INSPECT;
		//Pick random element
		state.pose = P_ELEMENT + Random(N_ELEMENTS);
		
		//Use expertise to pick first uninspected element, otherwise leave it random
		if(Bernoulli(expertise)){
			int pose_ = P_ELEMENT;
			
			for(auto e : state.elements){
				if(e.inspected) pose_++; //TODO: when switching to probs, use p(OK) to reinspect
				else break;
			}
			
			if(pose_ < N_ELEMENTS) //found at least one uninspected
				state.pose = pose_;
			
			//If all were already inspected, leave the random choice above
		}
		
		return false;
	}
	
	//INSPECT activity, only at element
	if(state.activity == A_INSPECT && state.pose >= P_ELEMENT){
		
		if(state.outcome == O_OK){
			if(Bernoulli(expertise)){
				state.activity = A_NONE;
				state.pose = P_NONE;
			}
		}
		else if(state.outcome == O_NOT_OK){
			if(Bernoulli(expertise)) state.activity = A_REPLACE;
		}
		return false;
	}
	
	//REPLACE element
	if(state.activity == A_REPLACE && state.pose >= P_ELEMENT){
		//If OK move on
		if(state.outcome == O_OK){
			state.activity = A_NONE;
		}
		//If NOT OK, keep trying with low expertise
		if(Bernoulli(expertise)){
			state.activity = A_NONE;
			state.pose = P_NONE;
		}
	}
	
	//RECEIVE part
	//Upon receiving, look for first element that can use the part
	if(state.activity == A_RECEIVE){
		int elem = 0;
		state.activity = A_NONE;
		for(elem=0; elem < N_ELEMENTS; elem++){
			//Attempt to replace matching element right away
			if(state.elements[elem].inspected && !state.elements[elem].OK && state.elements[elem].part == state.partAvailable){
				state.activity = A_REPLACE;
				state.pose = P_ELEMENT + elem;
				break;
			}
		}
	}
	
	return false;
}

void MAINTENANCE_WORKER::generateOutcomes(MAINTENANCE_STATE& state)
{
	
	//INSPECT
	if(state.activity == A_INSPECT){
		if(state.pose >= P_ELEMENT){
			int elem = state.pose - P_ELEMENT;
			state.elements[elem].inspected = true;
			state.outcome = state.elements[elem].OK; //Reveal true state of element. TODO: Add Bernoulli(expertise) to inspection
		}
		
		if(state.pose == P_NONE){
			state.outcome = O_OK;
		}
	}
	
	//REPLACE
	if(state.activity == A_REPLACE){
		state.outcome = O_FAIL; //default, either wrong pose or wrong part
		
		if(state.pose >= P_ELEMENT){
			int elem = state.pose - P_ELEMENT;
			if(state.partAvailable == state.elements[elem].part){
				state.elements[elem].OK = true;
				state.outcome = O_OK;
				state.partAvailable = -1; //remove part
			}
		}
	}
	
	//RECEIVE sets part available externally
	if(state.activity == A_RECEIVE){
		state.outcome = O_OK;
	}
}

const MAINTENANCE_STATE MAINTENANCE_WORKER::getCurrentState(){
	return currentState;
}

void MAINTENANCE_WORKER::DisplayCurrentState(){
	DisplayState(currentState, cout);
}

void MAINTENANCE_WORKER::DisplayState(const MAINTENANCE_STATE& state, std::ostream& ostr) const{
	ostr << "<Activity: ";
	switch(state.activity){
		case A_NONE:
			ostr << "NONE";
			break;
		case A_INSPECT:
			ostr << "INSPECT";
			break;
		case A_REPLACE:
			ostr << "REPLACE";
			break;
		case A_RECEIVE:
			ostr << "RECEIVE";
			break;
	}
	ostr << " | ";

	ostr << "Pose: ";
	
	if(state.pose >= P_ELEMENT) ostr << (state.pose - P_ELEMENT);
	else ostr << "none";	
	ostr << " | ";
	
	ostr << "Outcome: ";
	switch(state.outcome){
		case O_NOT_OK:
			ostr << "Not OK";
			break;
		case O_OK:
			ostr << "OK";
			break;		
		case O_FAIL:
			ostr << "Failed";
			break;
	}
	ostr << ">" << endl;
	
	ostr << "No. of parts: " << N_PARTS << " - ";
	ostr << "Current part: ";
	if(state.partAvailable >= 0){
		ostr << state.partAvailable;
		if(parts_str.size() == N_PARTS) ostr << " (" << parts_str[state.partAvailable] << ")";
	}
	else
		ostr << "None";
	
	ostr << endl;

	ostr << "Elements: " << endl;
	int count = 0;
	ostr << "\t[";
	ostr << std::left << std::setw(4) << "Pose" << " | ";
	if(parts_str.size() == N_PARTS) ostr << std::left << std::setw(10) << "Name" << " | ";
	ostr << std::left << std::setw(6) << "Status" << " | ";
	ostr << std::left << std::setw(8) << "Priority" << " | ";
	ostr << std::left << std::setw(4) << "Part" << " | ";
	ostr << std::left << std::setw(10) << "Inspected";
	ostr << "]" << endl;
	
	for(auto e : state.elements){
		ostr << "\t[";
		ostr << std::left << std::setw(4) << count++ << " | ";
		if(e.name != "") ostr << std::left << std::setw(10) << e.name << " | ";
		ostr << std::left << std::setw(6) << (e.OK? "OK" : "Not OK") << " | ";
		ostr << std::left << std::setw(8) << e.priority << " | ";
		ostr << std::left << std::setw(4) << e.part << " | ";
		ostr << std::left << std::setw(10) << (e.inspected? "Y" : "N");
		ostr << "]" << endl;
	}
}

/*** POMDP interface ***/
void MAINTENANCE_WORKER::makePartAvailable(MAINTENANCE_STATE& state, int part){	
		
	if(part >= -1 && part < N_PARTS){
		state.activity = A_RECEIVE;
		state.partAvailable = part;
	}
	
}
