% This class generates a stream of events. It could, for example, run a
% simulator, or could (eventually) be a wrapper on ROS.

classdef EventGenerator < handle
    
    properties(Access = protected)
        parameters;
        stepNumber;
        
        T;
        
        % The set of events just created
        mostRecentEvents;

    end
    
    methods(Access = public)
        
        % Construct the object
        function this = EventGenerator(parameters)
            
            assert(isa(parameters, 'minislam.event_generators.simulation.Parameters'), ...
                'eventgenerator:wrongparametertype', ...
                'Parameter of class %s; should be minislam.event_generators.simulation.Parameters', ...
                class(parameters));

            this.parameters = parameters;
            
        end
        
        function nextEvents = events(this)
            nextEvents = this.mostRecentEvents;
        end
        
        % Get the step number; this starts at 0 and advances by 1 after
        % each call to step.
        
        function stepNumber = stepCount(this)
            stepNumber = this.stepNumber;
        end
       
    end
    
    methods(Access = public, Abstract = true)
        
        % This method returns true as long as we should keep running
        carryOn = keepRunning(this);

        % Step the event generator
        step(this);
        
        % Get the ground truth; empty if not available. Flag shows if this
        % should be full and contain all the landmarks
        groundTruthState = groundTruth(this, getFullStateInformation);
        
        % Get the current simulation time
        T = time(this);
        
    end
end