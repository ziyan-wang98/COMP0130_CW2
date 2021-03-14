classdef KalmanFilterSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    % This is a very minimal Kalman filter; its purpose is to let me do
    % some debugging and comparison. It is not fully featured.
    
    properties(Access = protected)
        
        % Kalman filter mean
        xEst;
        PEst;
        
        % Kalman filter covariance
        xPred;
        PPred;
        
        % Store of the mean and covariance values for the vehicle
        timeStore;
        xEstStore;
        PEstStore;
    end
    
    methods(Access = public)
        
        function this = KalmanFilterSLAMSystem()
            this = this@minislam.slam.VehicleSLAMSystem();
            this.xEstStore = NaN(3, 1);
            this.PEstStore = NaN(3, 1);
            this.xEst = NaN(3, 1);
            this.PEst = NaN(3, 3);
        end
        
        function [x,P] = robotEstimate(this)
            x = this.xEst(1:3);
            P = this.PEst(1:3, 1:3);
        end
        
        function [T, X, PX] = robotEstimateHistory(this)
            T = this.timeStore;
            X = this.xEstStore;
            PX = this.PEstStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(this)
            landmarkIds = [];
            x = NaN(2, 0);
            P = NaN(2, 2, 0);
        end
        
        function recommendation = recommendOptimization(this)
            recommendation = true;
        end
        
        function processEvents(this, events)
            % Handle the events
            processEvents@minislam.slam.VehicleSLAMSystem(this, events);
            
            % Store the estimate for the future
            this.timeStore(:, this.stepNumber) = this.currentTime;
            this.xEstStore(:, this.stepNumber) = this.xEst(1:3);
            this.PEstStore(:, this.stepNumber) = diag(this.PEst(1:3, 1:3));
        end
        
        
        function optimize(~, ~)
            % Nothing to do
        end
    end
       
    methods(Access = protected)
                    
        function handleInitialConditionEvent(this, event)
            this.xEst = event.data;
            this.PEst = event.covariance;
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handleNoPrediction(this)
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handlePredictToTime(this, time, dT)

            % You will need to write the code to implement the process
            % model which:
            % 1. Computes this.xPred
            % 2. Computes the Jacobian
            % 3. Computes the process noise
            error('handlePredictToTime: implement');
        end
        
        function handleGPSObservationEvent(this, event)
            
            % You will need to write the code to implement the fusing the
            % platform position estimate with a GPS measurement
            error('handleGPSObservationEvent: implement');
        end
        
        function handleLandmarkObservationEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end
 
    end
end