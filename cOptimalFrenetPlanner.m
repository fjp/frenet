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
        KJ = 0.1   % Jerk
        KT = 0.1   % Time
        KD = 2.0   % Distance
        KV = 0.5   % Target speed
        KLAT = 1.0 % Lateral
        KLON = 1.0 % Longitudinal
    end
    
    methods (Access = public)
        
        function frenetTrajectories = CalcFrenetTrajectories(obj, v, d, dd, ddd, s0)
            frenetTrajectories = {};
            
            % Generate path for each offset goal
            % Lateral sampling space
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
                        %lonPoly5 = cQuinticPoly(s0, v, 0.0, tv, 0.0, 0.0, Ti);
                        %lonPoly5 = cQuinticPoly(s0, v, 0.0, s0+tv, 0.0, 0.0, Ti);
                        lonPoly4 = cQuarticPoly(s0, v, 0.0, tv, 0.0, Ti);
                        
                        targetft.s = lonPoly4.X(ft.t);
                        targetft.ds = lonPoly4.dX(ft.t);
                        targetft.dds = lonPoly4.ddX(ft.t);
                        targetft.ddds = lonPoly4.dddX(ft.t);
                        
                        % Square of lateral jerk
                        Jp = sum(targetft.dddd.^2);
                        % Square of longitudinal jerk
                        Js = sum(targetft.ddds.^2);
                        
                        % Square of diff from target speed
                        ds = (obj.TARGET_SPEED - targetft.ds(end)).^2;
                        
                        targetft.Jd = obj.KJ * Jp + obj.KT * Ti + obj.KD * targetft.d(end)^2;
                        targetft.Js = obj.KJ * Js + obj.KT * Ti + obj.KV * ds;
                        targetft.J = obj.KLAT * targetft.Jd + obj.KLON * targetft.Js;
                        
                        frenetTrajectories{end+1} = targetft;
                    end
                end
            end
        end
        
        function frenetTrajectories = CalcGlobalTrajectories(obj, frenetTrajectories, referencePath)
            for iTarj = 1:length(frenetTrajectories)
                ft = frenetTrajectories{iTarj};
                % calc global positions
                for i = 1:(length(ft.s))
                    [ix, iy] = referencePath.calc_position(ft.s(i));
                    if isnan(ix)
                        break
                    end
                    iyaw = referencePath.calc_yaw(ft.s(i));
                    di = ft.d(i);
                    fx = ix + di * cos(iyaw + pi / 2.0);
                    fy = iy + di * sin(iyaw + pi / 2.0);
                    ft.x(end+1) = fx;
                    ft.y(end+1) = fy;
                end

                % calc theta and dL (running length)
                for i = 1:(length(ft.x) - 1)
                    dx = ft.x(i+1) - ft.x(i);
                    dy = ft.y(i+1) - ft.y(i);
                    ft.theta(end+1) = atan2(dy, dx);
                    ft.dL(end+1) = sqrt(dx^2 + dy^2);
                end

                ft.theta(end+1) = ft.theta(end);
                ft.dL(end+1) = ft.dL(end);

                % calc curvature
                for i = 1:(length(ft.theta) - 1)
                    ft.kappa(end+1) = (ft.theta(i+1) - ft.theta(i)) / ft.dL(i);
                end
                ft.kappa(end+1) = ft.kappa(end);
                
                frenetTrajectories{iTarj} = ft;

            end
        end
        
        
        function collision = CheckCollision(obj, ft, ob)

            for i = 1:(length(ob(:, 1)))
                ox = ob(i, 1);
                oy = ob(i, 2);
                d = zeros(length(ft.x), 1);
                for iPoint = 1:length(ft.x)
                    ix = ft.x(iPoint);
                    iy = ft.y(iPoint);
                    d(iPoint) = ((ix - ox)^2 + (iy - oy)^2);
                end
                collision = any(d+1 <= obj.ROBOT_RADIUS^2);
                if collision
                    plot(ft.x, ft.y)
                    plot(ox, oy, 'ro');
                    return;
                end
            end
            
        end


        function okTrajectories = CheckTrajectories(obj, frenetTrajectories, ob)

            okTrajectories = {};
            for i = 1:(length(frenetTrajectories))
                ft = frenetTrajectories{i};
                if any(ft.ds > obj.MAX_SPEED)  % Max speed check
                    continue
                elseif any(abs(ft.dds) > obj.MAX_ACCEL)  % Max accel check
                    continue
                elseif any(abs(ft.kappa) > obj.MAX_CURVATURE)  % Max curvature check
                    continue
                elseif obj.CheckCollision(ft, ob)
                    continue
                end

                okTrajectories{end+1} = ft;
            end
        end


        function bestpath = FrenetOptimalPlanning(obj, referencePath, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

            frenetTrajectories = obj.CalcFrenetTrajectories(c_speed, c_d, c_d_d, c_d_dd, s0);
            frenetTrajectories = obj.CalcGlobalTrajectories(frenetTrajectories, referencePath);
            frenetTrajectories = obj.CheckTrajectories(frenetTrajectories, ob);

            % Find minimum cost trajectory
            mincost = inf;
            bestpath = NaN;
            for iTraj = 1:(length(frenetTrajectories))
                ft = frenetTrajectories{iTraj};
                if (mincost >= ft.J)
                    mincost = ft.J;
                    bestpath = ft;
                end
            end
        end


        function referencePath = CalcReferencePath(x, y, ds)
            referencePath = CalcSplineCourse(x, y, ds);
        end
    end
    
end

