# `PlanSys2-Exercise`
[![main](https://github.com/Docencia-fmrico/bump-and-stop-looping/actions/workflows/main.yaml/badge.svg)](https://github.com/Docencia-fmrico/bump-and-stop-looping/actions/workflows/main.yaml)

<p align="center">
   <img src="https://user-images.githubusercontent.com/86266311/218310781-c180a553-a43e-4309-a8be-66c29d33adbd.png" width="100%" height="100%">
</p>

## Summary
1. [Overview](#Overview)
2. [What is PDDL?](#What-is-PDDL?)
3. [What is Plansys2?](#What-is-Plansys2?)
4. [Map](#Map)
5. [License](#License)

## Overview

We have created a [PDDL](https://planning.wiki/) domain and six PDDL problems that will be solved using [POPF](https://planning.wiki/ref/planners/popf) and [Plansys2](https://plansys2.github.io/) implemented in ros2. The problem represents a house with different items and a grandma that will ask the robot to do some of the chores for her. We will solve the problem in the gazebo simulator with the robot tiago.

## What-is-PDDL?

Planning Domain Definition Language (PDDL) is a family of languages which allow us to define a planning problem. As planning has evolved, so to has the language used to describe it and as such there are now many versions of PDDL available.

In our case, we define a set of objects which have predicate properties (properties which are either true or false). Therefore, we define a set of actions which condition on, and then change these properties. We use logic expressions such as: and, or, not and we have introduced two key elements. Besides, we use time and numbers. We have learnt up to PDDL 2.1 version.

<a href="https://planning.wiki/guide/whatis/pddl" target="Resource Information">Resource Information</a>

## What-is-Plansys2?

ROS2 Planning System (plansys2 in short) is a project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system. It is implemented in ROS2, applying the latest concepts developed in this currently de-facto standard in Robotics.

PlanSys2 has a modular design. It is basically composed of 4 nodes:
* Domain Expert: Contains the PDDL model information (types, predicates, functions, and actions).
* Problem Expert: Contains the current instances, predicates, functions, and goals that compose the model.
* Planner: Generates plans (sequence of actions) using the information contained in the Domain and Problem Experts.
* Executor: Takes a plan and executes it by activating the action performers (the ROS2 nodes that implement each action).

Each of these nodes exposes its functionality using ROS2 services. Even so, in PlanSys2 are created a client library that can be used in any application and hides the complexity of using ROS2 services.

For more information visit the <a href="https://plansys2.github.io/design/index.html" target="official website">Resource Information</a>.

## Map
Here you can see the map we have used for handling this exercise: 

## License

The code and documentation in this project are released under the [Apache 2](LICENSE) license.
