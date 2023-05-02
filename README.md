Project Author: Charl van de Merwe  
Date: 2 May 2023

# Overview

This project contains a library with classes to implement discrete PI or PID control, with or without anti-windup. 

# Getting Started

This is a CMake project, meaning that it is IDE independent. The project should open, configure, compile, and run without issues. For more information on CMake projects and Catch2, see https://github.com/CharlvdM/CMakeCatch2AndGccDemo. 

# MATLAB Simulation and Test Cases Data

A MATLAB simulation is configured to simulate the PI & PID controllers without and with anti-windup (PID_ControllerTest.m & ValidatePiWithAntiWindup.m). Continuous and discrete simulations are implemented and compared. The discrete simulation output is used to generate the test case data used to validate that the C++ implementation (test_PID_Controller.cpp).

# PID C++ Implementation

The PID and PI controller implementation is given in PID_Controller.cpp and PID_Controller.cpp. As mentioned, the implementation is tested in test_PID_Controller.cpp, using Catch2.
