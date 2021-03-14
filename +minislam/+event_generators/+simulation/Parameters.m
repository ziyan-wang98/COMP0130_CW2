classdef Parameters < handle
   
    % This class just contains a set of parameters
    
    properties(Access = public)
        
        % Time step length
        DT = 0.1;
        
        % If set to a finite value, the terminator artificially terminates
        % at this step number
        maximumStepNumber = inf;
        
        % Vehicle wheelbase. See https://en.wikipedia.org/wiki/Wheelbase.
        % Average wheelbase value from
        % Chassis Handbook: Fundamentals, Driving Dynamics, Components, Mechatronics, Perspectives
        B = 2.5;
        
        % Flag to show if noises should be added
        perturbWithNoise = true;
        
        % Steer angle controls; made up
        maxDiffDeltaRate = 20*pi/180;
        maxDelta = 40 * pi / 180;
        
        % Speed controls; made up
        maxSpeed = 10;
        minSpeed = 1;
        maxAcceleration = 0.5;

        % Flag to show if odometry is available
        enableOdometry = true;
        
        % Control input measurement noise
        ROdometry = diag([0.1 pi/180 0.1*pi/180]).^2;
        
        % Flag to show if GPS available
        enableGPS = true;        
        
        % The period between GPS measurements
        gpsMeasurementPeriod = 1;
        
        % GPS noise measurement
        RGPS = 9 * eye(2);
        
        % Flag to show if the laser is available
        enableLaser= true;
        
        % The period between laser measurements
        laserMeasurementPeriod = 0.1;
        
        % Detection range - same for all landmarks
        laserDetectionRange = 10;
        
        % Noise on each measurement
        RLaser = diag([0.1 pi/180]).^2;
        
    end
    
end