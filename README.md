# [ForceControlCollection](https://github.com/isri-aist/ForceControlCollection)
Force control functions for robot control

[![CI-standalone](https://github.com/isri-aist/ForceControlCollection/actions/workflows/ci-standalone.yaml/badge.svg)](https://github.com/isri-aist/ForceControlCollection/actions/workflows/ci-standalone.yaml)
[![CI-catkin](https://github.com/isri-aist/ForceControlCollection/actions/workflows/ci-catkin.yaml/badge.svg)](https://github.com/isri-aist/ForceControlCollection/actions/workflows/ci-catkin.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/ForceControlCollection/)

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)
- [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)

## Technical details
[Wrench distribution](https://isri-aist.github.io/ForceControlCollection/doxygen/classForceColl_1_1WrenchDistribution.html#details) is a common method in robot control that, given a resultant wrench, calculates the equivalent contact wrench at the contact patches. For example, section III.B of the following paper describes the formulas for wrench distribution.
- M Murooka, et al. Centroidal trajectory generation and stabilization based on preview control for humanoid multi-contact motion. RA-Letters, 2022. [(available here)](https://hal.science/hal-03720407)
