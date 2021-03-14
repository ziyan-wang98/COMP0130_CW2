% This script can be used to develop your SLAM system. It runs a "little
% map" which should be a lot faster to work with

import minislam.slam.g2o.*;

% Configure to disable unneed sensors
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;

% This is how to change the covariance in the simulated laser outputs
%parameters.RLaser(2,2) = (5*pi/180)^2;

% Development or test mode?
%scenarioDirectory = 'q3-small-develop';
scenarioDirectory = 'q3-large-test';

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, scenarioDirectory);

% Create and run the different localization systems
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, g2oSLAMSystem);

% You will need to add your analysis code here

% Here's how to print out the number of edges and vertices
g2oGraph = g2oSLAMSystem.optimizer();

numVertices = length(g2oGraph.vertices())
numEdges = length(g2oGraph.edges())

