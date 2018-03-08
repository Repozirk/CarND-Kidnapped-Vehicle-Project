﻿[//]: # (Image References)
[image1]: ./result/image1.PNG



# Particle Filter
This is Project 3 of Udacity Self-Driving Car Nanodegree program. 
The goal of the project is to apply a 2 Dimensional Particle Filter to correctly localize a vehicle with observation data and landmarks using C++.

The project was created with the Udacity [Starter Code](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

## [Rubric](https://review.udacity.com/#!/rubrics/747/view) Points

## Content of this repo
- `scr` a directory with the project code:
  - `main.cpp` - reads in data, calls the function to run the particle filter
  - `particle_filter.cpp` - initializes the filter, calls the predict function, calls the data association and the weights update feature
- `data`  a directory with two input files, provided by Udacity
- `results`  a directory with output of the Simulator
- `CMakeLists.txt`
- `README.md` Description of the Project by Udacity
- `writeup.md` for submission of this Project

##Result
The Particlue Filter hit the criteria automatically checked by the terminal. A variaton of the particle number `num_particles` leads to a number of 100 for best results.

![alt text][image1] 




