% This class wraps an initial condition for the vehicle. Specifically, it's
% the starting position and orientation. The covariance is always set to
% zero.

classdef InitialConditionEvent < minislam.event_types.Event
    
    methods(Access = public)
        
        function this = InitialConditionEvent(time, data, covariance)
            this = this@minislam.event_types.Event(time, minislam.event_types.Event.INITIAL_CONDITION, data, covariance);
        end
        
    end
end
