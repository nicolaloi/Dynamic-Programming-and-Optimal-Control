# Dynamic Programming and Optimal Control 
The goal of this programming exercise was to pick up and deliver a package with a drone as quickly as possible when the system dynamics are subject to probabilistic uncertainties. 
##
Full grade granted.
## Problem set up
<img align="right" height="140" src="https://github.com/loinicola/Dynamic-Programming-and-Optimal-Control/blob/main/Images/UAV.png"></img>

The drone operates in a discretized world of M x N cells and its dynamics at a certain time step are modelled through three state variables, namely its position on the <i>x</i> and <i>y</i>-axis and a binary variable telling if the UAV is carrying the package or not. 
At time step <i>k</i> the system evolves in the following way:
<ul>
  <li>One of the allowable control inputs <i>u<sub>k</sub></i> is applied. In particular <i>u<sub>k</sub></i> &isin; {North, South, East, West, Hover}. However the control input is not allowed if it leads ouside the world boundaries or against a tree (green cells); angry citizens are present scattered around the map and have the possibility to shoot and destroy the drone with a probability proportional to their distance from the drone. </li>
  <img align="right" height="140" src="https://github.com/loinicola/Dynamic-Programming-and-Optimal-Control/blob/main/Images/Small_map.png"></img>
  <li>From the new desired position, a gust of wind can occur, moving the drone either North, South, East and West uniformly at random;</li>
  <li>The drone crashes if it ends up outside the grid world boundaries or against a tree, or if it is shooted by an angry citizen;</li>
  <li>Whenever a drone crashes it is brought to a base station carrying no package. The fixing procedure takes <i>N<sub>c</sub></i> time steps.</li>
</ul>

## Tasks
The drone starts at the base station (yellow cell) and needs to pickup the package at the pickup point (purple cell) and delivery it to the delivery point (blue cell).
Find the policy minimizing the expected number of time steps needed to achieve the goal by using the following approaches for the Infinite Horizon problem:
<ul>
  <li>Value Iteration;</li>
  <li>Policy Iteration;</li>
  <li>Linear Programming.</li>
</ul>

## Setup

Further info in [Assignment.pdf](Assignment.pdf).  
  
Run the whole project from [main.m](main.m).  
  
Example with a random generated map:  
source map (left) &nbsp;  **-->** &nbsp;  costs and policy to pickup (center) &nbsp;  **-->** &nbsp;  costs and policy to deliver (right).

<p align="center">
 <img height="250" src="https://github.com/loinicola/Dynamic-Programming-and-Optimal-Control/blob/main/Images/Map.png"/>
 <img height="250" src="https://github.com/loinicola/Dynamic-Programming-and-Optimal-Control/blob/main/Images/Pickup.png"/>
 <img height="250" src="https://github.com/loinicola/Dynamic-Programming-and-Optimal-Control/blob/main/Images/Delivery.png"/>
</p>
 
