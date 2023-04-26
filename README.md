# `PlanSys2-Exercise`
[![main](https://github.com/Docencia-fmrico/bump-and-stop-looping/actions/workflows/main.yaml/badge.svg)](https://github.com/Docencia-fmrico/bump-and-stop-looping/actions/workflows/main.yaml)

<p align="center">
   <img src="https://user-images.githubusercontent.com/86266311/218310781-c180a553-a43e-4309-a8be-66c29d33adbd.png" width="100%" height="100%">
</p>

## Summary
1. [Overview](#Overview)
2. [What is PDDL?](#What-is-PDDL?)
3. [What is Plansys2?](#What-is-Plansys2?)
4. [Behavior-trees](#behavior-trees)
4. [Map](#Map)
5. [License](#License)

## Overview
We have executed one of the plans generated in the [task handled before](https://github.com/Docencia-fmrico/ejercicio-pddl-looping). In this ocassion we have created a problem similar to the ones of the task handled before. Moreover, he have obtained Tigao navigating through our custom map.  
 
## What is PDDL?

Planning Domain Definition Language (PDDL) is a family of languages which allow us to define a planning problem. As planning has evolved, so to has the language used to describe it and as such there are now many versions of PDDL available.

In our case, we define a set of objects which have predicate properties (properties which are either true or false). Therefore, we define a set of actions which condition on, and then change these properties. We use logic expressions such as: and, or, not and we have introduced two key elements. Besides, we use time and numbers. We have learnt up to PDDL 2.1 version.

<a href="https://planning.wiki/guide/whatis/pddl" target="Resource Information">Resource Information</a>

## What is Plansys2?

ROS2 Planning System (plansys2 in short) is a project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system. It is implemented in ROS2, applying the latest concepts developed in this currently de-facto standard in Robotics.

<p align="center">
   <img src="https://github.com/Docencia-fmrico/plansys2-gpsr-looping/blob/origin/Readme/images/plansys2.png" width="70%" height="70%">
</p>

PlanSys2 has a modular design. It is basically composed of 4 nodes:
* Domain Expert: Contains the PDDL model information (types, predicates, functions, and actions).
* Problem Expert: Contains the current instances, predicates, functions, and goals that compose the model.
* Planner: Generates plans (sequence of actions) using the information contained in the Domain and Problem Experts.
* Executor: Takes a plan and executes it by activating the action performers (the ROS2 nodes that implement each action).

Each of these nodes exposes its functionality using ROS2 services. Even so, in PlanSys2 are created a client library that can be used in any application and hides the complexity of using ROS2 services.

For more information visit the <a href="https://plansys2.github.io/design/index.html" target="official website">Resource Information</a>.

## Behavior Trees 
Behaviour tree is a mathematical model of plan execution. They have probed more flexible and suitable that state machines for defining robots' simple behaviours. This task uses Behavior Trees to execute all plans. 

## Map
Here you can see the map we have used for handling this exercise: 
<p align="center">
   <img src="https://github.com/Docencia-fmrico/ejercicio-pddl-looping/blob/main/images/map.jpg" width="100%" height="100%">
</p>

For this task is a requirement to make the robot move using navigation and so, we have created a custom map which is similar to the image above. This is how it looks like: 

<p align="center">
   <img src="https://github.com/Docencia-fmrico/plansys2-gpsr-looping/blob/origin/Readme/images/custom_map_1.jpeg" width="100%" height="100%">
</p>

<p align="center">
   <img src="https://github.com/Docencia-fmrico/plansys2-gpsr-looping/blob/origin/Readme/images/custom_map_2.jpeg" width="100%" height="100%">
</p>

### How to create a custom map? 
Set steps : 

Signpost Map 

<p align="center">
   <img src="https://github.com/Docencia-fmrico/plansys2-gpsr-looping/blob/origin/Readme/images/signpost_image.jpeg" width="100%" height="100%">
</p>


## Objects Organized 

These are the objects that must be organized: 

* Towels: bathroom 
* Cutlery: kitchen
* Tools: garage
* Clothes: grandma bedroom
* Plants: yard
* Magazines: living room 
                                                                                                                                   
## Grandma Priorities

 Anything that grandma asks for, must have a higher priority than the other tasks.
 These are the high-priority tasks: 
 
* Close/Open main door.
* Bring her a cup of milk from the kitchen.
* Bring her pills from the living room.
* Make bed from the guest's room.

## Domain
 ### Predicates
  ### Robot predicates
    (robot_at ?r - robot ?l - location)    - states where the robot is
    (robot_free ?r - robot)                - states that the robot isn't carrying anything.
    (robot_carrying ?r - robot ?o - item)  - states that the robot is carrying an item.
    
  ### Grandma predicates
    (grandma_at ?g - grandma ?l - location)         - states where grandma is
    (grandma_wants ?i - item)                       - states that grandma wants the robot to bring her an item.
    (grandma_has_item)                              - states that grandma has all the desired items.
    (grandma_req_open_door ?g - grandma ?d - door)  - states that grandma wants the robot to open a door for her.
    (grandma_get_open_door)                         - states that grandma requires no more doors to be opened.
    (no_grandma_chores)                             - states that grandma has no more requests.
    
  ### Item predicates
    (item_at ?o - item ?l - location)         - states where an item is.
    (item_should_be ?i - item ?l - location)  - states where an item should be in order to be considered organized.
    (item_organized ?i - item)                - states that an item is considered organized.
    (item_disorganized ?i - item)             - states that an item is considered disorganized.
    
   ### Locations and doors predicates
    (connected_door ?l1 ?l2 - location ?d - door) - states that two locations are connected by a door
    (connected ?l1 ?l2 - location)                - states that two locations are connected (without door).
    (open ?d - door)                              - states that a door is opened.
    (close ?d - door)                             - states that a door is closed.
    
   ### Actions
   All actions are durative-actions. The main actions are:
   
   ### organize-item
   If **no_grandma_chroes** is true, move a disorganized item to where it should be and mark it as organized. By setting **no_grandma_chores** as a condition, we can assure that the robot will only organize an item when grandma has no more requests for it. organize-item is a "low-priority" action.
   
   ### item_for_grandma
   Brings the requested item to grandma. Sets **grandma_has_item** as true and **grandma_wants** as false.
    
   ### request_open_door
   Opens the requested door for grandma. Sets **grandma_req_open_door** as false and **grandma_get_open_door** as true.
   
   ### grandma_chores
   If both **grandma_get_open_door** and **grandma_has_item** are *true*, sets **no_grandma_chores** as true. Stating that grandma has no more requests for the robot and it can start working on the low-priority requests.

## Problem
Set problem defined

## What it is necessary to launch?


## Videos

### Nav sim 


### Real nav
## License

The code and documentation in this project are released under the [Apache 2](LICENSE) license.
