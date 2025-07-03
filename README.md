Juan Carlos Saborío<sup>1</sup>, Marc Vinci<sup>2</sup>, Oscar Lima<sup>2</sup>  
<sup>1</sup> Joint Lab for AI & DS, University of Osnabrück  
<sup>2</sup> Collaborative Autonomous Systems, DFKI Niedersachsen

# intention_recognition_ros

C++ ROS wrapper for RAGE, used as an AGR-POMDP solver.

This package acts as a client for both a perception and manipulation server, required to execute the respective type of actions.

The perception server receives information gathering tasks and returns observations.

The manipulation server uses actionlib to delegate task requests and execute them onboard the Mobipick.

## To run

```
roscore
rosrun rqt_incorap_kl_mockup rqt_incorap_kl_mockup_node
rosrun incorap_planning plan_and_execute_node
rosrun intention_recognition_ros intention_recognition_ros_node --paramFile [path] --problemFile [path] --domainFile [path]
```

- Since this version includes RAGEPlan, several parameter files are necessary. These can be found in ```./params```

- ```--paramFile``` specifies the planning/search parameters (no. of simulations, rollout type, timeout conditions, etc.). For example ```params/params.input```
- ```--problemFile``` specifies the problem parameters, such as default perception accuracy, POMDP discount and various PGS/IRE variables. For example ```params/manualAssembly.prob```
- ```--domainFile``` is the domain description file, with e.g., types of parts and part mapping to objects.

NOTE that plan_and_execute_node requires the Mobipick labs software available separately (e.g., GitHub).

## Description

The AGR planner can request actions such as:

- Inspect containers -> `<`status`>` where status = {empty,not empty}
- Inspect assembly progress -> `<`assembly`>`, a binary array over the list of parts: <pre> `assembly = <v_p ∈ {0,1} for all p ∈ P>` </pre>
- Perceive hotel type -> `<`type`>` where type = {type1, type2}

## Testing/validation methodology
A human worker assembles various parts in any order until one known type is successfully completed.

The worker is simulated by a task model that performs actions such as ASSEMBLE, WAIT, REMOVE or NONE.

The worker begins by randomly picking and assembling a part from it a list matching some assigned hotel type.

If a part is missing, the worker proceeds to WAIT -> NONE -> ASSEMBLE (random selection).

With a small probability the simulated worker may pick a part from the wrong list, and if the error is detected (also with probability) it will remove it.

At each step/turn, the robot may choose to either observe (see above) or bring a part (missing or not). Gathering real-world observations helps the robot refine its internal belief until it closely matches the external environment, therefore making informed action choices.

### ECMR methodology

Results can be replicated using the complete ROS/Gazebo simulated pipeline, or onboard a robot and system that matches all functional requirements: observations, task execution, AGR.
