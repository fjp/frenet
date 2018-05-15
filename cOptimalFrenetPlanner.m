classdef cOptimalFrenetPlanner
    %COPTIMALFRENETPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Parameter
        MAX_SPEED = 50.0 / 3.6  % maximum speed [m/s]
        MAX_ACCEL = 2.0  % maximum acceleration [m/ss]
        MAX_CURVATURE = 1.0  % maximum curvature [1/m]
        MAX_ROAD_WIDTH = 7.0  % maximum road width [m]
        D_ROAD_W = 1.0  % road width sampling length [m]
        DT = 0.2  % time tick [s]
        MAXT = 5.0  % max prediction time [m]
        MINT = 4.0  % min prediction time [m]
        TARGET_SPEED = 30.0 / 3.6  % target speed [m/s]
        D_T_S = 5.0 / 3.6  % target speed sampling length [m/s]
        N_S_SAMPLE = 1  % sampling number of target speed
        ROBOT_RADIUS = 2.0  % robot radius [m]

        % Cost weights
        KJ = 0.1
        KT = 0.1
        KD = 1.0
        KLAT = 1.0
        KLON = 1.0
    end
    
    methods (Access = public)
        
        function CalcFrenetTrajectory
    end
    
end

