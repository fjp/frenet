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
        
        function frenetTrajectories = CalcFrenetTrajectory(v, d, dd, ddd, s0)
            frenetTrajectories = {};
            
            % Generate path for each offset goal
            for di = -obj.MAX_ROAD_WIDTH:obj.D_ROAD_W:obj.MAX_ROAD_WIDTH
                % Lateral motion planning
                for Ti = obj.MINT:obj.DT:obj.MAXT
                    ft = cFrenetTrajectory();
                    
                    latPoly5 = cQuinticPoly(d, dd, ddd, di, 0.0, 0.0, Ti);
                    
                    ft.t = 0.0:obj.DT:Ti;
                    ft.d = latPoly5.X(ft.t);
                    ft.dd = latPoly5.dX(ft.t);
                    ft.ddd = latPoly5.ddX(ft.t);
                    ft.dddd = latPoly5.dddX(ft.t);
                    
                    % Longitudinal motion planning (velocity keeping)
                    for tv = (obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE):obj.D_T_S:(obj.TARGET_SPEED + obj.D_T_S * obj.N_S_SAMPLE)
                        targetft = ft;
                        lonPoly5 = cQuinticPoly(s0, v, 0.0, tv, 0.0, 0.0, Ti);
                        
                        targetft.s = lonPoly5.X(t);
                        targetft.ds = lonPoly5.dX(t);
                        targetft.dds = lonPoly5.ddX(t);
                        targetft.ddds = lonPoly5.dddX(t);
                        
                        % Square of lateral jerk
                        Jp = sum(targetft.dddd.^2);
                        % Square of longitudinal jerk
                        Js = sum(targetft.ddds.^2);
                        
                        % Square of diff from target speed
                        ds = (obj.TARGET_SPEED - targetft.ds(end)).^2;
                        
                        targetft.Jd = obj.KJ * Jp + obj.KT * Ti + obj.KD * targetft.d(end)^2;
                        targetft.Js = obj.KJ * Js + obj.KT * Ti + obj.KD * ds;
                        targetft.J = obj.KLAT * targetft.Jd + obj.KLON * targetft.Js;
                        
                        frenetTrajectories{end+1} = targetft;
                    end
                end
            end
        end
        
        function CalcGlobalPaths(frenetTrajectories, referencePath)
            for ft = frenetTrajectories
                % TODO
            end
        end
    end
    
end

