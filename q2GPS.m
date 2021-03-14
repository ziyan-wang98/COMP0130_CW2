% This script runs the GPS

import minislam.slam.kalman.*;
import minislam.slam.g2o.*;

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = true;
parameters.enableLaser = false;

% This says how much simulation time happens between each GPS measurement.
% Change as per the coursework instructions
parameters.gpsMeasurementPeriod = 5;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'q2');

% Create and run the different localization systems
kalmanFilterSLAMSystem = KalmanFilterSLAMSystem();
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, {kalmanFilterSLAMSystem, g2oSLAMSystem});

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
plot(results{2}.vehicleCovarianceHistory', '--')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory', 'LineWidth', 2)
hold on
plot(results{2}.vehicleStateHistory'-results{2}.vehicleTrueStateHistory','--', 'LineWidth', 2)