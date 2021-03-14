% This class implements an event-based estimation system for building up a
% minimal, ideal SLAM system. The system is event-based and responds to a
% sequence of events which are time stamped and served in order.

classdef VehicleSLAMSystem < handle
        
    properties(Access = protected)
        
        % Step number - how many times have we iterated?
        stepNumber;
        
        % The last time an event was processed.
        currentTime;
        
        % The vehicle control inputs. These are wheel speed and steer
        % angle. These are assumed to be held constant until the next
        % control input event comes along.
        u;
        uCov;
                
        % Flag to show if debugging is enabled
        debug = true;
        
        % Flag to show if the system has been initialized or not
        initialized;
    end
       
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = VehicleSLAMSystem()
            
            % Set the start time to 0.
            this.currentTime = 0;
            
            % Set the current step number to zero
            this.stepNumber = 0;
            
            % Default odometry. This will stay the same until new
            % measurements are provided.
            this.u = zeros(3, 0);
            this.uCov = zeros(3, 3);            
            
            this.initialized = false;
        end
        
        % Process a cell array which is a sequence of events. Each event is
        % processed in the order it appears in the array.
        function processEvents(this, eventQueue)
            
            % Increment the step number
            this.stepNumber = this.stepNumber + 1;
           
            events = eventQueue.events();
            
            % Get the next event
            for k = 1 : length(events)                
                event = events{k};

                % First check it's of the right class.
                assert(isa(event, 'minislam.event_types.Event'));
                
                % Now do all the actual work
                this.processEvent(event);
            end
        end
        
        function processEvent(this, event)

            % If initialized, predict if the timestep length is
            % sufficiently long
            if (this.initialized == true)
                dT = event.time() - this.currentTime;

                % Nothing to do if it's really close to the last time or we
                % have no odometry
                if (abs(dT) < 1e-3)
                    this.handleNoPrediction();
                else
                    this.handlePredictToTime(event.time(), dT);                                
                end
            end

            this.currentTime = event.time;

            % Handle the event on the basis of its type. Note that if we
            % are not initialized yet, we can only handle the
            % odometry and initialization events.
            switch(event.type)
                
                case minislam.event_types.Event.INITIAL_CONDITION
                    assert(this.initialized == false)
                    this.handleInitialConditionEvent(event);
                    this.initialized = true;

                case minislam.event_types.Event.VEHICLE_ODOMETRY
                    this.handleVehicleOdometryEvent(event);
                    
                case minislam.event_types.Event.GPS
                    if (this.initialized == true)
                        this.handleGPSObservationEvent(event);
                    end

                case minislam.event_types.Event.LANDMARK
                    if (this.initialized == true)
                        this.handleLandmarkObservationEvent(event);
                    end
                    
                otherwise
                    error('Unknown observation type %d', event.type)     
            end            
        end
    end
        
    methods(Access = protected)
        
        % Handle a new odometry event. We simply change the vehicle
        % odometry to the new value.
        function handleVehicleOdometryEvent(this, event)
            this.u = event.data;
            this.uCov = event.covariance;
        end
    end
    
    methods(Access = public, Abstract)
        
        % Get the mean and covariance of the estimate at the current time.
        % This is used for output purposes.
        [x, P] = robotEstimate(this);
        
        % Get the mean and covariance history of the robot across the whole
        % run. This is used for analysis purposes.
        [T, X, PX] = robotEstimateHistory(this);
        
        % Get the current landmarks estimates.
        [x, P, landmarkIds] = landmarkEstimates(this);
        
        % Recommend if an optimization is required
        recommendation = recommendOptimization(this);
        
        % Optimize
        optimize(this, maximumNumberOfOptimizationSteps);
        
    end
    
    methods(Access = protected, Abstract)
       
        % Handle the initial conditions
        handleInitialConditionEvent(this, event);
                
        % Handle when there is no prediction between events
        handleNoPrediction(this);
        
        % Handle everything needed to predict to the current state
        handlePredictToTime(this, time);
 
        % Handle a GPS measurement
        handleGPSObservationEvent(this, event);
            
        % Handle a set of measurements of landmarks
        handleLandmarkObservationEvent(this, event);
    end
end
