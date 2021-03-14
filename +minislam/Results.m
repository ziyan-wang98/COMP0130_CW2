classdef Results < handle
    
    % This structure stores potentially useful information.
    
    properties(Access = public)
    
        % For each time step, time required to run the optimization. Is NaN
        % when no optimization was run.
        optimizationTimes;
        
        % For each time step, the ground truth of the vehicle
        vehicleTrueStateTime;
        vehicleTrueStateHistory;
        
        % For each time step, the predicted pose
        vehicleStateTime;        
        vehicleStateHistory;
        vehicleCovarianceHistory;
        
    end
    
end