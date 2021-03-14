% This class implements the event generator for blender-based log files.
% The events are pulled from a log file.

classdef LogFileEventGenerator < minislam.event_generators.EventGenerator
   
    properties(Access = protected)
        
        % The current simulation time
        currentTime;

        % The directory containing the data files
        logFileDirectory;

        % Debug flag to plot data as it's imported
        plotFramesOnImport = true;        
        
        % The loaded data
        logData;
        
        logIndex;
        
        % The timestep between frames
        dT;
        
        % Debugging!
        noiseScale = 0;
        
        % Time of next GPS event
        nextGPSTime;
        
        % Time of next range-bearing sensor event
        nextLaserTime;

    end
        
    methods(Access = public)
        
        function this = LogFileEventGenerator(parameters, logFileDirectory)
            
            this = this@minislam.event_generators.EventGenerator(parameters);
            
            this.logFileDirectory = logFileDirectory;
                        
            this.start();
        end

        function start(this)
            
            this.loadLogFiles();
            this.logIndex = 0;
            this.currentTime = 0;
            this.stepNumber = 0;
            
            % Start everything off at the same time
            this.nextGPSTime = this.currentTime;
            this.nextLaserTime = this.currentTime;            
        end
    
        % This method returns true as long as we should keep running
        function carryOn = keepRunning(this)
            carryOn = (this.stepNumber < this.logData.numberOfCameraPoses);
        end

        % Get the next event from the generator
        function step(this)
            
            % Create the odometry message first. We do this because the
            % measurement is used to predict to the next time and do the
            % update
            
            this.mostRecentEvents = {};
            
            % If this is start time, create an initialisation event. If
            % odometry is enabled, we have to send an initial empty
            % odometry event. This is so the estimators know to expect
            % odometry when the first odometry event appears.
            if (this.stepNumber == 0)
                
                if (this.parameters.enableOdometry == true)
                    odometryEvent = minislam.event_types.VehicleOdometryEvent(this.currentTime, ...
                        zeros(2,1), this.parameters.ROdometry);            
                    this.mostRecentEvents = cat(1, this.mostRecentEvents, {odometryEvent});
                end
                
                initialConditionEvent = minislam.event_types.InitialConditionEvent(this.currentTime, ...
                    this.logData.poses2(1, 2:4)', zeros(3));
                this.mostRecentEvents = cat(1, this.mostRecentEvents, {initialConditionEvent});
                this.stepNumber = 1;
                return
            end
            
            if (this.stepNumber == 1)
                if (this.parameters.enableOdometry == true)
                    odometryEvent = minislam.event_types.VehicleOdometryEvent(this.currentTime, ...
                        zeros(2,1), this.parameters.ROdometry);            
                    this.mostRecentEvents = cat(1, this.mostRecentEvents, {odometryEvent});
                end
                this.stepNumber = 2;
                this.logIndex = 1;
                return
            end
            
            % Bump the step number
            this.stepNumber = this.stepNumber + 1;

            % Bump the time step
            this.currentTime = this.currentTime + 1;
            
            this.logIndex = this.logIndex + 1;
            
            % If requested, create the odometry event
            if (this.parameters.enableOdometry == false)
                return
            end
            
            % Odometry is only approximately correct
            currentPose = this.logData.poses2(this.stepNumber, 2:4);
            lastPose = this.logData.poses2(this.stepNumber-1, 2:4);
            s = norm(currentPose(1:2)-lastPose(1:2));
            dPhi = g2o.stuff.normalize_theta(currentPose(3) - lastPose(3));
            odometryMeasurement = [s; dPhi] + this.noiseScale * sqrtm(this.parameters.ROdometry) * randn(2, 1);
            odometryEvent = minislam.event_types.VehicleOdometryEvent(this.currentTime, ...
                odometryMeasurement, 1e-6 * this.parameters.ROdometry);            
            this.mostRecentEvents = cat(1, this.mostRecentEvents, {odometryEvent});
            
            % Compute the GPS observation if necessary
            gpsEvents = this.simulateGPSEvents();            
            this.mostRecentEvents = cat(1, this.mostRecentEvents, gpsEvents);

            % Compute the laser observation if necessary
            laserEvents = this.simulateLaserEvents();            
            this.mostRecentEvents = cat(1, this.mostRecentEvents, laserEvents);
        end
        
        function groundTruthState = getGroundTruth(this, getFullStateInformation)
            groundTruthState = minislam.event_generators.simulation.SimulatorState();
            
            % Required information
            groundTruthState.currentTime = this.currentTime;
            
            if (this.logIndex == 0)
                groundTruthState.xTrue = this.logData.poses2(1, 2:4)';
            else
                groundTruthState.xTrue = this.logData.poses2(this.logIndex, 2:4)';
            end
            groundTruthState.uTrue = zeros(2, 1);
            
            % Optional information
            if (getFullStateInformation == true)
                groundTruthState.waypoints = this.logData.poses2(:, 2:3)';
                groundTruthState.mTrue = this.logData.landmarks3(17:17:end, 1:2)';
            end
        end
            
    end
    
    methods(Access = protected)
        
        function loadLogFiles(this)
            
            % Find the full directory
            fullDirectoryWhat = what(this.logFileDirectory);
            
            fullPath = fullDirectoryWhat.path();
            
            % If the cache file exists, load it and return
            cacheFileName = fullfile(fullPath, 'logDataCache.mat');
            if (exist(cacheFileName, 'file') == 2)
                load(cacheFileName);
                this.logData = logData;
                return
            end
                        
            disp('Loading log files and building cache');
            
            % Load the files
            disp('Loading ground truth 3D landmarks')
            logData.landmarks3 = load(fullfile(fullPath, 'landmarks_3d.txt'));

            % Load the poses. Note there is an error in the exporter. The
            % computed heading is atan2(y, x) where (x,y) is the position
            % of the robot. We change it to be the heading of the velocity
            % vector which links subsequent poses together. We set the
            % heading at the first step to be the same as the second step.
            disp('Loading ground poses')
            logData.poses2 = load(fullfile(fullPath, 'poses_2d.txt'));
            dPose = diff(logData.poses2(:,2:3));
            logData.poses2(2:end, 4) = atan2(dPose(:, 2), dPose(:, 1));
            logData.poses2(1, 4) = logData.poses2(2, 4);
            disp('Loading camera parameters')
            logData.cameraParams = load(fullfile(fullPath, 'calibration.txt'));
            
            disp('Loading camera poses');
            logData.cameraPoses = load(fullfile(fullPath, 'camera_poses.txt'));
            
            logData.numberOfCameraPoses = size(logData.cameraPoses, 1);

            disp('Loading tracks');
            
            if (this.plotFramesOnImport == true)
                minislam.graphics.FigureManager.getFigure('Log File 3D Map View');
                hold off
                H = plot(NaN, NaN, 'b.');
                hold on
                axis([0 1024 0 768]);
            end
            
            % Load the tracks file; this specifies how individual pixels
            % are tracked from frame to frame.
            fileID = fopen(fullfile(fullPath, 'tracks.txt'));
            frames = {};

            tline = fgetl(fileID);
            stepNumber = 1;
            while (tline~= -1)
                fields = textscan(tline, '%f');
                frames{stepNumber}.id = fields{1}(1:3:end);
                frames{stepNumber}.pixels = [fields{1}(2:3:end)';fields{1}(3:3:end)'];
                if (this.plotFramesOnImport == true)
                    set(H, 'XData', frames{stepNumber}.pixels(1,:), 'YData', frames{stepNumber}.pixels(2,:));
                    drawnow
                end
                tline = fgetl(fileID);
                stepNumber = stepNumber + 1;
            end
            logData.frames = frames;
            
            % Load 2D measurements. We have to rebuild the observations to
            % account for some issues with exporting the measurement.
            fileID = fopen(fullfile(fullPath, 'measurements_2d.txt'));
            measurements2 = {};

            tline = fgetl(fileID);
            stepNumber = 1;
            while (tline~= -1)
                fields = textscan(tline, '%f');
                measurements2{stepNumber}.ids = fields{1}(1:5:end);
                numLandmarks = length(measurements2{stepNumber}.ids);
                heading = logData.poses2(stepNumber, 4);
                dX = logData.landmarks3(measurements2{stepNumber}.ids, 1) - logData.poses2(stepNumber, 2);
                dY = logData.landmarks3(measurements2{stepNumber}.ids, 2) - logData.poses2(stepNumber, 3);
                dZ = logData.landmarks3(measurements2{stepNumber}.ids, 3);
                measurements2{stepNumber}.r = sqrt(sum(dX.^2+dY.^2+dZ.^2,2));
                measurements2{stepNumber}.az =  minislam.utils.pi_to_pi(atan2(dY, dX) - heading);
                measurements2{stepNumber}.el = atan2(dZ, sqrt(sum(dX.^2+dY.^2,2)));
                tline = fgetl(fileID);
                stepNumber = stepNumber + 1;
            end
            logData.measurements2 = measurements2;
            
            this.logData = logData;
            
            save(cacheFileName, 'logData');
        end
        
        function gpsEvents = simulateGPSEvents(this)
            
            if ((this.parameters.enableGPS == false) || (this.currentTime < this.nextGPSTime))
                gpsEvents = {};
                return
            end
            
            return;
        end
            
        function laserEvents = simulateLaserEvents(this)
            
            laserEvents = {};
            if ((this.parameters.enableLaser == false) || (this.currentTime < this.nextLaserTime))
                return
            end
            
            this.nextLaserTime = this.nextLaserTime + this.parameters.laserMeasurementPeriod;
            
            measurements2 = this.logData.measurements2{this.stepNumber};
            
            idx = find(rem(measurements2.ids, 17) == 0);
            
            numLandmarks = length(idx);

            r = measurements2.r(idx)'  + this.noiseScale * sqrt(this.parameters.RLaser(1,1)) * randn(1, numLandmarks);
            az = minislam.utils.pi_to_pi(measurements2.az(idx)' + this.noiseScale * sqrt(this.parameters.RLaser(2,2)) * randn(1, numLandmarks));
            el = measurements2.el(idx)' + this.noiseScale * sqrt(this.parameters.RLaser(3,3)) * randn(1, numLandmarks);

            % Package into a single event
            laserEvents = {minislam.event_types.LaserObservationEvent(this.currentTime, ...
                [r; az; el], this.parameters.RLaser, measurements2.ids(idx))};
        end

    end
        
end
