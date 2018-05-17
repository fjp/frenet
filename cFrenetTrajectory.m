classdef cFrenetTrajectory
    %CFRENETPATH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % time stamps
        t
        % Longitudinal coordinate
        s    % Longitudinal position
        ds   % Longitudinal velocity
        dds  % Longitudinal acceleration
        ddds % Longitudinal jerk
        % Lateral coordinate
        d    % Lateral position
        dd   % Lateral velocity
        ddd  % Lateral acceleration
        dddd % Lateral jerk
        
        % Costs
        Jd % Lateral cost
        Js % Longitudinal cost
        J  % Total cost
        
        % World coordinates
        x     % x position
        y     % y position
        theta % Orientation
        kappa % Curvature
        dL    % Running length / arc length
        v     % Tangential velocity
        a     % Tangential acceleration
    end
    
    methods
    end
    
end

