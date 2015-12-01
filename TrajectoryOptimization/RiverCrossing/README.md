# README.md

## Problem Statement:
A boat needs to cross a river between two docks. The water is flowing in the river with a parabolic velocity profile. The boat travels at a fixed speed, but can choose its orientation. What is the minimum-time path between the two points?

## NOTES:
This turns out to be a rather tricky trajectory optimization, and I've not completely solved it here. There are two key difficulties:
 - The solution is discontinuous
 - The solution is not unique

There are probably some modifications that can be made to the problem statement to adress one or both of these problems. 

This code is still in progress. Let me know if you want to improve it, or if you find a bug!

## Methods:
0. Multiple Shooting - Euler's Method. This is included to get an idea of what multiple shooting is, but it is not an efficient implementation.
0. Direct Collocation - Trapazoid Rule. This is a pretty good implementation of the trapazoid rule, and gets reasonable solutions for most of the problems.
0. Hermite-Simpson Collocation. Seems a bit buggy, which I think is due to the nature of the problem, although there could be a bug.



