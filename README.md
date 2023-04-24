# `PlanSys2-Exercise`
[![main](https://github.com/Docencia-fmrico/bump-and-stop-looping/actions/workflows/main.yaml/badge.svg)](https://github.com/Docencia-fmrico/bump-and-stop-looping/actions/workflows/main.yaml)

<p align="center">
   <img src="https://user-images.githubusercontent.com/86266311/218310781-c180a553-a43e-4309-a8be-66c29d33adbd.png" width="100%" height="100%">
</p>

## Summary
1. [Overview](#Overview)
2. [What is PDDL?](#What-is-PDDL?)
3. [Map](#Map)
4. [Objects Organized](#Objects-Organized)
5. [Grandma Priorities](#Grandma-Priorities)
6. [Domain](#Domain)
7. [Problems](#Problems)
8. [Running the program](#Run-the-program)
10. [License](#License)

## Overview

We have created a [PDDL](https://planning.wiki/) domain and six PDDL problems that will be solved using [POPF](https://planning.wiki/ref/planners/popf) and [Plansys2](https://plansys2.github.io/) implemented in ros2. The problem represents a house with different items and a grandma that will ask the robot to do some of the chores for her. We will solve the problem in the gazebo simulator with the robot tiago.

## What-is-PDDL?

Ejercicio 4 de Planificación y Sistemas Cognitivos 2023

En grupos de 4, haced una aplicación en ROS 2 usando PlanSys2 que use el dominio de la [Práctica 3](https://github.com/Docencia-fmrico/planning-exercise/blob/main/README.md). El robot debe poder realizar en un simulador, navegando con Nav2, goals similares a los siguientes:

(ordena_casa robot1)
(abrir_puerta puerta_principal)
(dar robot1 vaso_leche abuelita)
(dar robot1 medicina abuelita)

Puntuación (sobre 10):   
* +5 correcto funcionamiento en el robot simulado.
* +2 Readme.md bien documentado con videos.
* +2 CI con Test de los nodos BT
* -3 Warnings o que no pase los tests.
