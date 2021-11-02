# Dynamic Programming and Optimal Control 
Optimize a flying policy for a drone to pick up and deliver a package as quickly as possible when the system dynamics are subject to probabilistic uncertainties (Markov Decision Processes). 

#

Project rated with full marks.

#

## Problem set up
<img align="right" height="140" src="https://user-images.githubusercontent.com/79461707/139326683-485a8a9d-d14b-4130-9b95-feca32b94927.png"></img>

The drone operates in a discretized world of M x N cells and its dynamics at a certain time step are modelled through three state variables, namely its position on the <i>x</i> and <i>y</i>-axis and a binary variable telling if the UAV is carrying the package or not. 
At time step <i>k</i> the system evolves in the following way:
<ul>
  <li>One of the allowable control inputs <i>u<sub>k</sub></i> is applied. In particular <i>u<sub>k</sub></i> &isin; {North, South, East, West, Hover}. However the control input is not allowed if it leads ouside the world boundaries or against a tree (green cells); angry citizens are present scattered around the map and have the possibility to shoot and destroy the drone with a probability proportional to their distance from the drone. </li>
  <img align="right" height="140" src="https://user-images.githubusercontent.com/79461707/139326643-773743b5-140e-4539-a2ba-a918f6e2d487.png"></img>
  <li>From the new desired position, a gust of wind can occur, moving the drone either North, South, East and West uniformly at random;</li>
  <li>The drone crashes if it ends up outside the grid world boundaries or against a tree, or if it is shooted by an angry citizen;</li>
  <li>Whenever a drone crashes it is brought to a base station carrying no package. The fixing procedure takes <i>N<sub>c</sub></i> time steps.</li>
</ul>

Further info: [Assignment.pdf](Assignment.pdf).

## Tasks
The drone starts at the base station (yellow cell) and needs to pickup the package at the pickup point (purple cell) and delivery it to the delivery point (blue cell).
Find the policy minimizing the expected number of time steps needed to achieve the goal by using the following approaches for the Infinite Horizon problem:
<ul>
  <li>Value Iteration;</li>
  <li>Policy Iteration;</li>
  <li>Linear Programming.</li>
</ul>

## Setup  
  
Run the whole project from [main.m](main.m).  
  
Example with a random generated map:  
source map (left) &nbsp;  **->** &nbsp;  costs and policy up to the pickup (center) &nbsp;  **->** &nbsp;  costs and policy for the delivery (right).

<p align="center">
 <img height="250" src="https://user-images.githubusercontent.com/79461707/139326567-9031236e-9442-45a7-a85a-d370a1081db7.png"/>
 <img height="250" src="https://user-images.githubusercontent.com/79461707/139326604-40491ea1-1e59-4852-b841-e67f0f7df213.png"/>
 <img height="250" src="https://user-images.githubusercontent.com/79461707/139326541-513cdeff-b761-49b3-8783-c927e17a4849.png"/>
</p>
 
