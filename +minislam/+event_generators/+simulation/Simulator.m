classdef Simulator < minislam.event_generators.EventGenerator
    
    % This class is a simple simulator which predicts the movement of a
    % car-like vehicle and generates result from it. For the second part of
    % the coursework, we will use a different and more general set of
    % classes.
    
    properties(Access = protected)
        
        % The current simulation time
        currentTime;
        
        % The state of the vehicle (x, y, phi)
        x;
        
        % The control inputs (speed, steer angle)
        u;
        
        % The set of landmarks
        landmarks;
        
        % The set of waypoints
        waypoints;
        
        % Index to the waypoint
        waypointIndex;
        
        % Flag indicates we're done
        carryOnRunning;
        
        % Time of next GPS event
        nextGPSTime;
        
        % Time of next range-bearing sensor event
        nextLaserTime;
        
        % Debug. Set to 0 to disable noise.
        noiseScale = 1;
        
        % Directory containing the scenario files.
        scenarioDirectory;
        
        % Flag to show if starting
        initialConditionsSent;
         
    end
    
    methods(Access = public)
        
        function this = Simulator(parameters, scenarioDirectory)
            
            this = this@minislam.event_generators.EventGenerator(parameters);
            
            % Setup default
            this.scenarioDirectory = scenarioDirectory;
            
            if (parameters.perturbWithNoise == true)
                this.noiseScale = 1;
            else
                this.noiseScale = 0;
            end
            
            this.start();
        end
        
        % Start the simulator
        function start(this)
            this.currentTime = 0;
            this.waypointIndex = 1;
            
            this.loadLogFiles();
            
            this.x = [0, 0, 0]';
            this.u = zeros(2, 1);
            
            % Start the sensors off at some random time
            this.nextGPSTime = this.currentTime + rand * this.parameters.gpsMeasurementPeriod;
            this.nextLaserTime = this.currentTime + rand * this.parameters.laserMeasurementPeriod;
            
            this.stepNumber = 0;
            
            this.carryOnRunning = true;
            
            this.mostRecentEvents = minislam.event_generators.OrderedEventQueue();
                        
            % Set up the initial condition event to send out
            initialConditionEvent = minislam.event_types.InitialConditionEvent(this.currentTime, ...
                    this.x, zeros(3));
            this.mostRecentEvents.insert(initialConditionEvent);
            
            % Set up the first odometry value to send out
            if (this.parameters.enableOdometry == true)
                odometryEvent = this.simulateOdometryEvent();
                this.mostRecentEvents.insert(odometryEvent);
            end
        end
        
        % Return whether the simulator has finished
        function carryOn =  keepRunning(this)
            carryOn = ((this.carryOnRunning) && (this.stepNumber < this.parameters.maximumStepNumber));
        end
        
        function T = time(this)
            T = this.currentTime;
        end
        
        % Step the simulator and return events
        function step(this)

            % Bump the step number
            this.stepNumber = this.stepNumber + 1;
            this.mostRecentEvents.clear();

            % Predict forwards to the next step
            vDT = this.u(1) * this.parameters.DT;
            phi = this.x(3);
            this.x(1) = this.x(1) + vDT * cos(phi);
            this.x(2) = this.x(2) + vDT * sin(phi);            
            this.x(3) = g2o.stuff.normalize_theta(this.x(3) + vDT * sin(this.u(2)) / this.parameters.B);

            % Bump the time step
            this.currentTime = this.currentTime + this.parameters.DT;

            % Compute the GPS observation if necessary
            gpsEvents = this.simulateGPSEvents();            
            this.mostRecentEvents.insert(gpsEvents);

            % Compute the laser observation if necessary
            laserEvents = this.simulateLaserEvents();            
            this.mostRecentEvents.insert(laserEvents);

            % Determine the wheel speed and steer angle for the robot which
            % will be applied next time
            this.computeControlInputs();
            
            % Create the odometry message for this next update step
            if ((this.carryOnRunning == true) && (this.parameters.enableOdometry == true))
                odometryEvent = this.simulateOdometryEvent();
                this.mostRecentEvents.insert(odometryEvent);
            end
        end
        
        % Get the ground truth state of the vehicle; not available in the
        % real world (alas)
        function groundTruthState = groundTruth(this, getFullStateInformation)
            groundTruthState = minislam.event_generators.simulation.SimulatorState();
            
            % Required information
            groundTruthState.currentTime = this.currentTime;
            groundTruthState.xTrue = this.x;
            groundTruthState.uTrue = this.u;
            
            % Optional information
            if (getFullStateInformation == true)
                groundTruthState.waypoints = this.waypoints;
                groundTruthState.mTrue = this.landmarks;
            end
        end
    end
            
    methods(Access = protected)

        function computeControlInputs(this)
            
            % Work out distance to the target waypoint
            dX = this.waypoints(:, this.waypointIndex) - this.x(1:2);
            d = norm(dX);
            
            % If sufficiently close, switch to the next waypoint;
            if (d < 1)
                this.waypointIndex = this.waypointIndex + 1;
                
                % If we've reached the end of the list of waypoints, return
                if (this.waypointIndex > size(this.waypoints, 2))
                    this.carryOnRunning = false;
                    return;
                end
                
                % Update to the new waypoint
                dX = this.x(1:2) - this.waypoints(:, this.waypointIndex);
                d = norm(dX);
            end
            
            % Compute the speed. We first clamp the acceleration, and then
            % clamp the maximum and minimum speed values.
            diffSpeed = 0.1 * d - this.u(1);
            maxDiffSpeed = this.parameters.maxAcceleration * this.parameters.DT;
            diffSpeed = min(maxDiffSpeed, max(-maxDiffSpeed, diffSpeed));
            this.u(1) = max(this.parameters.minSpeed, min(this.parameters.maxSpeed, this.u(1) + diffSpeed));

            % Compute the steer angle. We first clamp the rate of change,
            % and then clamp the maximum and minimum steer angles.
            diffDelta = g2o.stuff.normalize_theta(atan2(dX(2), dX(1)) - this.x(3) - this.u(2));
            maxDiffDelta = this.parameters.maxDiffDeltaRate * this.parameters.DT;            
            diffDelta = min(maxDiffDelta, max(-maxDiffDelta, diffDelta));
            this.u(2) = min(this.parameters.maxDelta, max(-this.parameters.maxDelta, this.u(2) + diffDelta));
            
            % Flag that we shoul d keep running
            this.carryOnRunning = true;            
        end
        
        % Simulate the odometry measurement. We have to produce an angular
        % velcoity measurement and corrupt by noise
        function odometryEvent = simulateOdometryEvent(this)
            
            psiDot = this.u(1) * sin(this.u(2)) / this.parameters.B;
            
            odometryMeasurement = [this.u(1); 0; psiDot] + this.noiseScale * sqrtm(this.parameters.ROdometry) * randn(3, 1);
            odometryEvent = minislam.event_types.VehicleOdometryEvent(this.currentTime, ...
                odometryMeasurement, this.parameters.ROdometry);            

        end
        
        function gpsEvents = simulateGPSEvents(this)
            
            if ((this.parameters.enableGPS == false) || (this.currentTime < this.nextGPSTime))
                gpsEvents = {};
                return
            end
                
            this.nextGPSTime = this.nextGPSTime + this.parameters.gpsMeasurementPeriod;
            gpsMeasurement = [this.x(1); this.x(2)] + this.noiseScale * sqrtm(this.parameters.RGPS) * randn(2, 1);
            gpsEvents = {minislam.event_types.GPSObservationEvent(this.currentTime, gpsMeasurement, ...
                this.parameters.RGPS)};
        end
            
        function laserEvents = simulateLaserEvents(this)
            
            laserEvents = {};
            if ((this.parameters.enableLaser== false) || (this.currentTime < this.nextLaserTime))
                return
            end
            
            this.nextLaserTime = this.nextLaserTime + this.parameters.laserMeasurementPeriod;
            
            % Find the landmarks which are in range
            
            % Compute the relative distance. Note the vehicle always sits
            % with z=0.
            dX = this.landmarks;
            dX(1, :) = dX(1, :) - this.x(1);
            dX(2, :) = dX(2, :) - this.x(2);
            
            % Squared range to each landmark
            R2 = sum(dX.^2,1);
            R = sqrt(R2);
            
            ids = find(R <= this.parameters.laserDetectionRange);
            
            % If nothing to see, return
            if (isempty(ids))
                return
            end
            
            numLandmarks = length(ids);
            
            % Create observations
            r = R(ids) + this.noiseScale * sqrt(this.parameters.RLaser(1,1)) * randn(1, numLandmarks);
            beta = g2o.stuff.normalize_thetas(atan2(dX(2, ids), dX(1, ids)) - this.x(3) ...
                + this.noiseScale * sqrt(this.parameters.RLaser(2,2)) * randn(1, numLandmarks));
            
            % Package into a single event
            laserEvents = {minislam.event_types.LandmarkObservationEvent(this.currentTime, ...
                [r; beta], this.parameters.RLaser, ids)};
        end
        
        function loadLogFiles(this)
            
            % Find the full directory for the data
            fullDirectoryData = what('coursework_data');
            
            % Search string for the right directory
            searchString = [filesep, this.scenarioDirectory];
            
            % Directory
            files = subdir(fullDirectoryData.path);
            
            fullPath = [];
            
            % Search for the directory
            for f = 1 : length(files)
                % If it doesn't contain the search path, continue
                if (isempty(strfind(files(f).folder, searchString)) == true)
                    continue
                else
                    fullPath = [files(f).folder, filesep];
                end
            end
            
            assert (isempty(fullPath) == false, 'simulator:loadLogFiles:directorycannotfinddirectory', ...
                'Cannot find data directory %s', this.scenarioDirectory);
            
            % Get the path
            
            this.x = load(fullfile(fullPath, 'x0.txt'))';
            this.landmarks = load(fullfile(fullPath, 'lm.txt'))';
            this.waypoints = load(fullfile(fullPath, 'wp.txt'))';
            
            % Strip z dimension off the landmarks.
            this.landmarks = this.landmarks(1:2, :);
        end
    end
end